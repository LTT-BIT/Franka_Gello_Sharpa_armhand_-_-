#!/usr/bin/env bash
# Install this self-contained GELLO + Sharpa + tactile recording checkout.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LITCHI_REPO="$REPO_ROOT/portable_deps/litchi_hardware"
GELLO_REPO="$REPO_ROOT/portable_deps/gello_software"
DATA_ROOT="${HOME}/franka_teleop_data"
ACCEPT_ORBBEC=false
INSTALL_SYSTEM=true
CONDA_ENV="${TELEOP_CONDA_ENV:-gello-upper-body-teleop}"

usage() {
  cat <<'EOF'
用法：./一键安装环境.sh [选项]

直接在复制到新电脑的 gello_upper_body_teleop 目录中执行本脚本。

选项：
  --accept-orbbec-eula   非交互确认已阅读并接受 Orbbec SDK EULA
  --data-root PATH       录制数据目录，默认 ~/franka_teleop_data
  --skip-system-packages 不自动安装缺失的 Ubuntu 软件包
  -h, --help             显示帮助

脚本只安装软件，不启动机器人、手或相机，也不会自动修改 FR3 IP、网卡、
CPU/IRQ、相机序列号或 GELLO 序列号。
EOF
}

while (($#)); do
  case "$1" in
    --accept-orbbec-eula) ACCEPT_ORBBEC=true; shift ;;
    --data-root) DATA_ROOT="${2:?缺少数据目录}"; shift 2 ;;
    --skip-system-packages) INSTALL_SYSTEM=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知选项：$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || {
  echo "当前只支持 Linux x86-64。检测到：$(uname -s) $(uname -m)" >&2
  exit 1
}
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  case "${ID:-}:${VERSION_ID:-}" in
    ubuntu:22.04|ubuntu:24.04) ;;
    *) echo "警告：目前仅在 Ubuntu 22.04/24.04 验证；当前为 ${PRETTY_NAME:-未知系统}。" >&2 ;;
  esac
fi

[[ -f "$LITCHI_REPO/pixi.lock" && -d "$LITCHI_REPO/src/litchi_hardware" ]] || {
  echo "缺少内置 Sharpa/MANUS 依赖：$LITCHI_REPO" >&2
  echo "请确认复制的是完整工程目录，且 portable_deps 没有被遗漏。" >&2
  exit 1
}
[[ -d "$GELLO_REPO/gello" && -d "$GELLO_REPO/third_party/DynamixelSDK/python" ]] || {
  echo "缺少内置 GELLO/DynamixelSDK 依赖：$GELLO_REPO" >&2
  echo "请确认复制的是完整工程目录，且 portable_deps 没有被遗漏。" >&2
  exit 1
}

if [[ "$ACCEPT_ORBBEC" != true ]]; then
  eula="$REPO_ROOT/ros_ws/src/orbbec_camera/SDK/End User License Agreement.txt"
  [[ -r "$eula" ]] || { echo "找不到 Orbbec EULA：$eula" >&2; exit 1; }
  if [[ ! -t 0 ]]; then
    echo "非交互安装必须在阅读 EULA 后传入 --accept-orbbec-eula。" >&2
    exit 2
  fi
  echo "使用三相机前必须阅读并接受："
  echo "  $eula"
  read -r -p "确认已经阅读并接受该 EULA？请输入 YES：" answer
  [[ "$answer" == YES ]] || { echo "未接受 EULA，安装已取消。" >&2; exit 2; }
  ACCEPT_ORBBEC=true
fi

[[ "$DATA_ROOT" == /* ]] || DATA_ROOT="$(realpath -m -- "$PWD/$DATA_ROOT")"

install_system_packages() {
  local compose_package=docker-compose-v2
  sudo apt-get update
  if ! apt-cache show "$compose_package" >/dev/null 2>&1; then
    compose_package=docker-compose-plugin
  fi
  sudo apt-get install -y \
    build-essential ca-certificates cmake curl ethtool git gzip iproute2 \
    libusb-1.0-0 lsof python3 python3-yaml rsync tar util-linux usbutils \
    docker.io "$compose_package"
  sudo systemctl enable --now docker
}

missing_system=()
for command_name in cmake curl docker git lsof python3 rsync taskset; do
  command -v "$command_name" >/dev/null 2>&1 || missing_system+=("$command_name")
done
if command -v docker >/dev/null 2>&1 && ! docker compose version >/dev/null 2>&1; then
  missing_system+=(docker-compose-v2)
fi
if ((${#missing_system[@]})); then
  if [[ "$INSTALL_SYSTEM" == true ]]; then
    echo "安装缺失的系统工具：${missing_system[*]}"
    install_system_packages
  else
    echo "缺少系统工具：${missing_system[*]}" >&2
    exit 1
  fi
fi

groups_changed=false
for group_name in docker dialout; do
  getent group "$group_name" >/dev/null || sudo groupadd "$group_name"
  if [[ " $(id -nG) " != *" $group_name "* ]]; then
    sudo usermod -aG "$group_name" "$USER"
    groups_changed=true
  fi
done
if [[ "$groups_changed" == true ]]; then
  cat >&2 <<'EOF'
已将当前用户加入 docker/dialout 用户组。
请完整注销桌面并重新登录，然后再次执行同一条安装命令。
当前尚未修改项目配置或启动任何硬件。
EOF
  exit 3
fi

docker info >/dev/null 2>&1 || {
  echo "当前用户无法访问 Docker daemon，请启动 Docker 后重试。" >&2
  exit 1
}
docker compose version >/dev/null 2>&1 || {
  echo "缺少 Docker Compose v2 插件。" >&2
  exit 1
}

tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "$tmp_dir"' EXIT
mkdir -p -- "$HOME/.local/bin"
if ! command -v conda >/dev/null 2>&1; then
  if [[ -x "$HOME/miniforge3/bin/conda" ]]; then
    export PATH="$HOME/miniforge3/bin:$PATH"
  else
    echo "安装 Miniforge 到 $HOME/miniforge3 ..."
    curl -fL \
      https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh \
      -o "$tmp_dir/miniforge.sh"
    bash "$tmp_dir/miniforge.sh" -b -p "$HOME/miniforge3"
    export PATH="$HOME/miniforge3/bin:$PATH"
  fi
fi
ln -sfn "$(command -v conda)" "$HOME/.local/bin/conda"
export PATH="$HOME/.local/bin:$PATH"

if ! command -v pixi >/dev/null 2>&1; then
  echo "安装 Pixi ..."
  curl -fsSL https://pixi.sh/install.sh -o "$tmp_dir/install-pixi.sh"
  bash "$tmp_dir/install-pixi.sh"
  export PATH="$HOME/.pixi/bin:$PATH"
fi

set_env_value() {
  local file=$1 key=$2 value=$3 tmp
  tmp="$(mktemp "${file}.XXXXXX")"
  awk -F= -v key="$key" -v value="$value" '
    BEGIN { written = 0 }
    $1 == key {
      if (!written) print key "=" value
      written = 1
      next
    }
    { print }
    END { if (!written) print key "=" value }
  ' "$file" > "$tmp"
  mv -- "$tmp" "$file"
}

ENV_FILE="$REPO_ROOT/docker/.env"
if [[ -f "$ENV_FILE" ]]; then
  backup="$REPO_ROOT/docker/.env.before-install.$(date +%Y%m%d_%H%M%S)"
  cp -- "$ENV_FILE" "$backup"
  echo "已备份原配置：$backup"
else
  cp -- "$REPO_ROOT/docker/.env.example" "$ENV_FILE"
fi
mkdir -p -- "$DATA_ROOT"
set_env_value "$ENV_FILE" TELEOP_DATA_ROOT "$DATA_ROOT"
set_env_value "$ENV_FILE" GELLO_SOFTWARE_ROOT "$GELLO_REPO"
set_env_value "$ENV_FILE" LITCHI_REPO "$LITCHI_REPO"
set_env_value "$ENV_FILE" ORBBEC_SDK_LICENSE_ACCEPTED YES
for pair in TELEOP_CONTROL_PORT:5590 PRESET_IK_PORT:5591 COLLECTION_CONTROL_PORT:5592; do
  key=${pair%%:*}; fallback=${pair##*:}
  current="$(awk -F= -v key="$key" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "$ENV_FILE")"
  set_env_value "$ENV_FILE" "$key" "${current:-$fallback}"
done

echo "创建主机 Conda 环境 ..."
"$REPO_ROOT/ops/setup/setup_teleop_env.sh"
GELLO_SOFTWARE_ROOT="$GELLO_REPO" "$REPO_ROOT/ops/setup/setup_gello_driver.sh"

echo "创建 Sharpa/MANUS Pixi 和 ROS 环境 ..."
for generated_dir in build install log; do
  generated_path="$LITCHI_REPO/ros2/$generated_dir"
  [[ ! -e "$generated_path" ]] || rm -rf -- "$generated_path"
done
(
  cd "$LITCHI_REPO"
  pixi install -e ros2
  pixi install -e retarget-v4
  if [[ -f reference/sharpa-manus-sdk/retargeting_alg_release_V4.0/include/hand_kinematic_casadi.so ]]; then
    pixi run -e ros2 stage-retarget-v4
  else
    # Portable deployments already contain the licensed staged binaries.
    # Do not run --clean when their original vendor reference was omitted.
    for asset in \
      manus_sharpa_teleop/manus_sharpa_teleop/vendor/retarget_v4/include/hand_kinematic_casadi.so \
      manus_sharpa_teleop/manus_sharpa_teleop/vendor/retarget_v4/include/hand_retargeting_optimizer.so \
      manus_sharpa_teleop/sharpa_wave_description/urdf/left_sharpa_wave/left_sharpa_wave.urdf \
      manus_sharpa_teleop/sharpa_wave_description/urdf/right_sharpa_wave/right_sharpa_wave.urdf; do
      [[ -f "ros2/src/$asset" ]] || { echo "Missing staged V4 asset: $asset" >&2; exit 1; }
    done
  fi
  pixi run -e ros2 ros-build
)

echo "构建 Docker 镜像和项目 ROS workspace ..."
for generated_dir in build install log; do
  generated_path="$REPO_ROOT/ros_ws/$generated_dir"
  [[ ! -e "$generated_path" ]] || rm -rf -- "$generated_path"
done
"$REPO_ROOT/ops/setup/build.sh" --accept-orbbec-eula --rebuild-image

echo "执行软件检查 ..."
conda run --no-capture-output -n "$CONDA_ENV" python -c \
  'import gello.dynamixel.driver, numpy, scipy, pinocchio; print("[PASS] host Python environment")'
(
  cd "$LITCHI_REPO"
  pixi run -e ros2 python -c \
    'from litchi_hardware import manus, sharpa; print("[PASS] Sharpa/MANUS runtime")'
)
(cd "$REPO_ROOT/docker" && docker compose config --quiet)
docker run --rm --network none franka-upper-body-teleop:latest \
  python3 -c 'import cv2, pyarrow; print("[PASS] Docker collection runtime")'

if ! "$REPO_ROOT/ops/diagnostics/check_franka_cpu_layout.sh"; then
  cat >&2 <<'EOF'

软件环境已经安装完成，但本机 CPU/IRQ 实时布局未通过检查。
不要启动机械臂。请按《工程迁移与一键安装说明.md》修改目标电脑的硬件配置，
然后重新执行 ops/diagnostics/check_franka_cpu_layout.sh。
EOF
  exit 4
fi

cat <<EOF

环境安装完成。
数据目录：$DATA_ROOT

首次运动前仍须完成《工程迁移与一键安装说明.md》中的硬件检查。
确认无误后，继续使用原来的三个启动命令。
EOF
