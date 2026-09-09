#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "该项目的原生基线是 Ubuntu 22.04，当前系统是 ${PRETTY_NAME:-未知}。" >&2
    echo "ROS 2 Humble 不以 Ubuntu 24.04 为官方二进制目标，停止安装以避免混装。" >&2
    exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "请先按 ROS 2 官方文档安装 ROS 2 Humble Desktop。" >&2
    exit 1
fi

jaka_ws="${JAKA_VENDOR_WORKSPACE:-$(dirname "${repo_dir}")/jaka_ros2}"
if [[ ! -f "${jaka_ws}/install/setup.bash" ]]; then
    echo "请先按 README 构建供应商工作区，或设置 JAKA_VENDOR_WORKSPACE。" >&2
    exit 1
fi
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential \
    git \
    libglfw3 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-venv \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-visual-tools \
    ros-humble-rqt-plot

source /opt/ros/humble/setup.bash
source "${jaka_ws}/install/setup.bash"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update
rosdep install \
    --from-paths "${jaka_ws}/src" "${repo_dir}/src" \
    --ignore-src \
    --rosdistro humble \
    --as-root pip:false \
    -r -y

python3 -m venv --system-site-packages "${repo_dir}/.venv"
source "${repo_dir}/.venv/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "${repo_dir}/requirements-mujoco.txt"

"${repo_dir}/scripts/native-build-moveit-overlay.sh"
echo "原生依赖安装完成。下一步运行：./scripts/native-colcon-build.sh"
