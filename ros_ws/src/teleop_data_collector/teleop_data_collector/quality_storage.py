"""Episode allocation and same-filesystem, non-overwriting quality archival."""
from __future__ import annotations

import ctypes
from contextlib import contextmanager
import errno
import fcntl
import json
import os
from pathlib import Path
import re
import tempfile

STAGING_FOLDER = "pending"

QUALITY_FOLDERS = {"优等": "A", "一般": "B", "报错": "C", "放弃": "D"}


def storage_roots() -> list[str]:
    return json.loads(os.environ.get("COLLECTION_STORAGE_ROOTS", "[]"))


def prepare_parent(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise ValueError("保存父目录必须是绝对路径")
    path = path.resolve()
    repository = Path(__file__).resolve().parents[4]
    if path == repository or repository in path.parents:
        raise ValueError("录制数据请保存在工程目录以外")
    roots = [Path(root).resolve() for root in storage_roots()]
    if roots and not any(path == root or root in path.parents for root in roots):
        raise ValueError("采集器可写的主机路径范围：" + "、".join(map(str, roots)))
    if (path / "metadata.yaml").exists() or (path / "collection_state.json").exists():
        raise ValueError("请选择保存父目录，不能选择已有 episode 的内部目录")
    path.mkdir(parents=True, exist_ok=True)
    for name in [STAGING_FOLDER, *QUALITY_FOLDERS.values()]:
        child = path / name
        if child.is_symlink():
            raise ValueError(f"归档目录不能是符号链接：{child}")
        child.mkdir(exist_ok=True)
        if child.stat().st_dev != path.stat().st_dev:
            raise ValueError(f"暂存与归档目录必须在同一文件系统：{child}")
        with tempfile.TemporaryFile(dir=child):
            pass
    return path


def allocate_episode(output: Path, prefix: str, parent: Path | None) -> Path:
    """Keep numbering across moves, restarts, destination changes and profiles.

    Allocation is persisted in the original recording group's directory, which
    stays fixed even when the user selects another storage parent.
    """
    output.mkdir(parents=True, exist_ok=True)
    counter = output / f".{prefix}.next"
    pattern = re.compile(rf"^{re.escape(prefix)}(\d+)$")
    with (output / f".{prefix}.allocation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        next_index = int(counter.read_text()) if counter.exists() else 0
        directories = [output]
        if parent is not None:
            directories += [parent / name for name in [STAGING_FOLDER, ".pending", *QUALITY_FOLDERS.values()]]
        for directory in directories:
            if not directory.exists():
                continue
            for child in directory.iterdir():
                match = pattern.fullmatch(child.name)
                if match and child.is_dir():
                    next_index = max(next_index, int(match[1]) + 1)
        temporary = counter.with_suffix(".tmp")
        with temporary.open("w") as stream:
            stream.write(str(next_index + 1))
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(counter)
        descriptor = os.open(output, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        staging = parent / STAGING_FOLDER if parent is not None else output
        return staging / f"{prefix}{next_index}"


def archive_episode(source: Path, parent: Path, quality: str) -> Path:
    destination = parent / QUALITY_FOLDERS[quality] / source.name
    if destination.parent.is_symlink():
        raise ValueError(f"归档目录不能是符号链接：{destination.parent}")
    if source == destination:
        if not source.is_dir():
            raise FileNotFoundError(source)
        return destination
    if source.parent.stat().st_dev != destination.parent.stat().st_dev:
        raise OSError(errno.EXDEV, "归档目录与暂存目录不在同一文件系统，原始数据保留", str(source))
    # Linux renameat2 ensures no existing bag (even an empty directory) can be
    # overwritten between a preflight check and the actual rename.
    libc = ctypes.CDLL(None, use_errno=True)
    rename = libc.renameat2
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        code = ctypes.get_errno()
        raise OSError(code, os.strerror(code), str(destination))
    return destination


@contextmanager
def episode_file_lock(parent: Path):
    """Shared with host deletion workers; never held for any network IO."""
    with (parent / ".episode-files.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield
