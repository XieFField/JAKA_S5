#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
overlay_dir="${repo_dir}/.native/moveit_shutdown_overlay"
baseline_file="${overlay_dir}/JAKA_BUILD_BASELINE"
moveit_commit="85dd2cf47bc97613b3e7196eb618ba6a243e8d72"
patch_file="${repo_dir}/patches/moveit2-2.5.9-clean-shutdown.patch"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "未找到 ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
    exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
    echo "未找到 MoveIt 退出修复：${patch_file}" >&2
    exit 1
fi

installed_version="$(dpkg-query --show --showformat='${Version}' ros-humble-moveit-ros-planning)"
case "${installed_version}" in
    2.5.9-*) ;;
    *)
        echo "当前脚本只支持 MoveIt 2.5.9，检测到：${installed_version}" >&2
        echo "不能把固定源码补丁应用到其他版本，请先核对版本和退出问题。" >&2
        exit 1
        ;;
esac

expected_commit="moveit2_commit=${moveit_commit}"
expected_version="debian_version=${installed_version}"
if [[ -f "${baseline_file}" ]] \
    && grep -Fxq "${expected_commit}" "${baseline_file}" \
    && grep -Fxq "${expected_version}" "${baseline_file}"; then
    echo "MoveIt 原生退出修复已安装：${overlay_dir}"
    exit 0
fi

mkdir -p "${repo_dir}/.native"
build_dir="$(mktemp -d /tmp/jaka-moveit-overlay.XXXXXX)"
trap 'rm -rf "${build_dir}"' EXIT

git -C "${build_dir}" init moveit2
git -C "${build_dir}/moveit2" remote add origin https://github.com/moveit/moveit2.git
git -C "${build_dir}/moveit2" sparse-checkout init --cone
git -C "${build_dir}/moveit2" sparse-checkout set \
    moveit_ros/planning moveit_ros/move_group
git -C "${build_dir}/moveit2" fetch --depth=1 origin "${moveit_commit}"
git -C "${build_dir}/moveit2" checkout --detach FETCH_HEAD
git -C "${build_dir}/moveit2" apply "${patch_file}"
mkdir -p "${build_dir}/workspace/src"
ln -s "${build_dir}/moveit2" "${build_dir}/workspace/src/moveit2"

source /opt/ros/humble/setup.bash
rm -rf "${overlay_dir}"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
    --base-paths "${build_dir}/workspace/src" \
    --build-base "${build_dir}/workspace/build" \
    --log-base "${build_dir}/workspace/log" \
    --install-base "${overlay_dir}" \
    --merge-install \
    --executor sequential \
    --parallel-workers 1 \
    --packages-select moveit_ros_planning moveit_ros_move_group \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

printf '%s\n%s\n' "${expected_commit}" "${expected_version}" > "${baseline_file}"
echo "MoveIt 原生退出修复已安装：${overlay_dir}"
