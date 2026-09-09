"""Command-line smoke test for the headless MuJoCo runtime."""

import argparse
import json

import numpy as np

from massage_mujoco.model import JOINT_NAMES
from massage_mujoco.runtime import MujocoRuntime


DEFAULT_TARGET = [0.15, 1.45, -1.45, 1.45, 1.45, -0.15]


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--settle-duration", type=float, default=0.5)
    parser.add_argument("--target", nargs=6, type=float, default=DEFAULT_TARGET)
    return parser.parse_args()


def main() -> None:
    """Run one deterministic position-control movement and print JSON."""
    arguments = _arguments()
    runtime = MujocoRuntime()
    result = runtime.move_to(
        arguments.target,
        duration=arguments.duration,
        settle_duration=arguments.settle_duration,
    )
    target = np.asarray(arguments.target)
    error = np.abs(result.position - target)
    print(json.dumps({
        "joint_names": JOINT_NAMES,
        "simulation_time": result.time,
        "target": target.tolist(),
        "position": result.position.tolist(),
        "maximum_joint_error": float(np.max(error)),
        "tool_position": result.tool_position.tolist(),
    }, sort_keys=True))


if __name__ == "__main__":
    main()
