#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_dir}/scripts/native-env.bash"
cd "${repo_dir}"

CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
    --symlink-install \
    --executor sequential \
    --parallel-workers 1 \
    --packages-up-to massage_bringup massage_mujoco \
    --event-handlers console_cohesion+
