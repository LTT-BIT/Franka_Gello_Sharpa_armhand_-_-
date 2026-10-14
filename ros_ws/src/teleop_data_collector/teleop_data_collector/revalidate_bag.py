from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any, Callable

from .bag_validation import inspect_bag
from .collector_contract import recording_outcome, topic_contracts


InspectBag = Callable[..., tuple[tuple[str, ...], dict[str, Any]]]


def revalidate_bag(
    bag_path: Path,
    *,
    trim_start_sec: float | None = None,
    trim_end_sec: float | None = None,
    inspect: InspectBag = inspect_bag,
) -> dict[str, Any]:
    bag_path = bag_path.expanduser().resolve()
    if not (bag_path / "metadata.yaml").is_file():
        raise ValueError(f"Not a ROS bag directory: {bag_path}")
    state_path = bag_path / "collection_state.json"
    if not state_path.is_file():
        raise ValueError(f"Bag is missing collection_state.json: {bag_path}")

    state = json.loads(state_path.read_text(encoding="utf-8"))
    previous_state = str(state.get("state", ""))
    if previous_state not in {"incomplete", "finalized"}:
        raise ValueError(
            f"Refusing to revalidate bag in {previous_state!r} state: {bag_path}"
        )
    raw_contract = state.get("source_topic_contract")
    contracts = topic_contracts(raw_contract)
    if trim_start_sec is None:
        trim_start_sec = float(state.get("trim_start_sec", 0.0))
    if trim_end_sec is None:
        trim_end_sec = float(state.get("trim_end_sec", 0.0))
    if trim_start_sec < 0.0 or trim_end_sec < 0.0:
        raise ValueError("trim values must be non-negative")
    continuity_kwargs = {"continuity_enabled": True} if state.get("continuity_enabled") else {}
    failures, report = inspect(
        bag_path,
        contracts,
        trim_start_sec=trim_start_sec,
        trim_end_sec=trim_end_sec,
        **continuity_kwargs,
    )
    failures = tuple(failures) + tuple(state.get("live_continuity", {}).get("failures", []))
    if state.get("auto_stop_reason") and not failures:
        failures = ("recording automatically stopped for a continuity failure",)
    outcome = recording_outcome(exit_code=0, stream_failures=failures)

    history = list(state.get("revalidation_history") or [])
    history.append(
        {
            "revalidated_at_ns": time.time_ns(),
            "previous_state": previous_state,
            "previous_finalized": bool(state.get("finalized", False)),
            "previous_failures": list(state.get("failures") or []),
        }
    )
    state.update(
        {
            "state": outcome.state,
            "finalized": outcome.finalized,
            "failures": list(outcome.failures),
            "validation_report": report,
            "validation_policy": "source_header_continuity_trimmed_boundary_v3",
            "trim_start_sec": trim_start_sec,
            "trim_end_sec": trim_end_sec,
            "source_warnings": list(dict.fromkeys([*report.get("source_warnings", []), *state.get("live_continuity", {}).get("source_warnings", [])])),
            "boundary_warnings": list(report.get("boundary_warnings") or []),
            "transport_warnings": list(report.get("transport_warnings") or []),
            "revalidation_history": history,
            "updated_at_ns": time.time_ns(),
        }
    )
    _write_json_atomic(state_path, state)
    return state


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Revalidate completed ROS bags with source-header continuity and "
            "atomically update collection_state.json."
        )
    )
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--trim-start-sec", type=float)
    parser.add_argument("--trim-end-sec", type=float)
    args = parser.parse_args(argv)
    if args.trim_start_sec is not None and args.trim_start_sec < 0.0:
        parser.error("--trim-start-sec must be non-negative")
    if args.trim_end_sec is not None and args.trim_end_sec < 0.0:
        parser.error("--trim-end-sec must be non-negative")

    failed = False
    for bag_path in args.bags:
        try:
            state = revalidate_bag(
                bag_path,
                trim_start_sec=args.trim_start_sec,
                trim_end_sec=args.trim_end_sec,
            )
        except Exception as error:
            failed = True
            print(f"ERROR {bag_path}: {error}")
            continue
        print(
            f"{bag_path}: state={state['state']} "
            f"finalized={state['finalized']} "
            f"boundary_warnings={len(state['boundary_warnings'])} "
            f"transport_warnings={len(state['transport_warnings'])}"
        )
        for warning in state["boundary_warnings"]:
            print(f"  BOUNDARY {warning}")
        for warning in state["transport_warnings"]:
            print(f"  WARNING {warning}")
        for failure in state["failures"]:
            print(f"  FAILURE {failure}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
