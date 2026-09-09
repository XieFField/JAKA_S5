#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_dir}/scripts/native-env.bash"
cd "${repo_dir}"

test_status=0
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon test \
    --executor sequential \
    --parallel-workers 1 \
    --packages-select massage_mujoco massage_bringup \
    --event-handlers console_cohesion+ \
    --return-code-on-test-failure || test_status=$?
colcon test-result --verbose || test_status=$?
exit "${test_status}"
