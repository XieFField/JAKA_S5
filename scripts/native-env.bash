#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "请使用 source scripts/native-env.bash" >&2
    exit 1
fi

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "未找到 ROS 2 Humble。" >&2
    return 1
fi
if [[ ! -f "${repo_dir}/.venv/bin/activate" ]]; then
    echo "未找到 MuJoCo Python 环境，请先运行 scripts/native-install-deps.sh。" >&2
    return 1
fi
if [[ ! -f "${repo_dir}/.native/moveit_shutdown_overlay/setup.bash" ]]; then
    echo "未找到 MoveIt 退出修复，请先运行 scripts/native-install-deps.sh。" >&2
    return 1
fi

jaka_ws="${JAKA_VENDOR_WORKSPACE:-$(dirname "${repo_dir}")/jaka_ros2}"
if [[ ! -f "${jaka_ws}/install/setup.bash" ]]; then
    echo "请先构建供应商工作区，或设置 JAKA_VENDOR_WORKSPACE。" >&2
    return 1
fi
source /opt/ros/humble/setup.bash
source "${jaka_ws}/install/setup.bash"
source "${repo_dir}/.venv/bin/activate"
source "${repo_dir}/.native/moveit_shutdown_overlay/setup.bash"
if [[ -f "${repo_dir}/install/setup.bash" ]]; then
    source "${repo_dir}/install/setup.bash"
fi

export JAKA_ARM_WORKSPACE="${repo_dir}"
