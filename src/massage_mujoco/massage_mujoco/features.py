"""Pure helpers for MuJoCo teaching, diagnostics, and surface following."""

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml

from massage_mujoco.runtime import TrajectoryPoint


class TeachingTrajectory:
    """Store validated joint keyframes and turn them into timed segments."""

    def __init__(self, joint_count: int, duplicate_tolerance: float = 1e-5):
        self.joint_count = int(joint_count)
        self.duplicate_tolerance = float(duplicate_tolerance)
        self.waypoints = []

    def _validate(self, positions: Sequence[float]) -> np.ndarray:
        values = np.asarray(positions, dtype=float)
        if values.shape != (self.joint_count,):
            raise ValueError(f"expected {self.joint_count} joint positions")
        if not np.all(np.isfinite(values)):
            raise ValueError("joint positions must be finite")
        return values

    def record(self, positions: Sequence[float]) -> int:
        values = self._validate(positions)
        if self.waypoints and np.max(np.abs(values - self.waypoints[-1])) <= (
            self.duplicate_tolerance
        ):
            raise ValueError("new teaching point is the same as the previous point")
        self.waypoints.append(values.copy())
        return len(self.waypoints)

    def clear(self) -> None:
        self.waypoints.clear()

    def timed_points(
        self,
        start_positions: Sequence[float],
        maximum_joint_speed: float,
        minimum_segment_duration: float,
    ) -> list:
        start = self._validate(start_positions)
        if not self.waypoints:
            raise ValueError("teaching trajectory contains no points")
        if not np.isfinite(maximum_joint_speed) or maximum_joint_speed <= 0.0:
            raise ValueError("maximum joint speed must be finite and positive")
        if (
            not np.isfinite(minimum_segment_duration)
            or minimum_segment_duration <= 0.0
        ):
            raise ValueError("minimum segment duration must be finite and positive")
        elapsed = 0.0
        previous = start
        points = []
        for waypoint in self.waypoints:
            duration = max(
                float(np.max(np.abs(waypoint - previous))) / maximum_joint_speed,
                minimum_segment_duration,
            )
            elapsed += duration
            points.append(TrajectoryPoint(elapsed, waypoint.copy()))
            previous = waypoint
        return points

    def save(self, path: Path, joint_names: Sequence[str]) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "joint_names": list(joint_names),
            "waypoints": [point.tolist() for point in self.waypoints],
        }
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, sort_keys=False)
        temporary.replace(destination)

    def load(self, path: Path, joint_names: Sequence[str]) -> int:
        source = Path(path)
        with source.open("r", encoding="utf-8") as stream:
            payload = yaml.safe_load(stream)
        if not isinstance(payload, dict) or payload.get("version") != 1:
            raise ValueError("unsupported teaching trajectory format")
        if payload.get("joint_names") != list(joint_names):
            raise ValueError("teaching trajectory joint order does not match")
        raw_waypoints = payload.get("waypoints")
        if not isinstance(raw_waypoints, list):
            raise ValueError("teaching trajectory waypoints must be a list")
        loaded = [self._validate(point).copy() for point in raw_waypoints]
        self.waypoints = loaded
        return len(self.waypoints)


@dataclass(frozen=True)
class TcpDiagnosticSample:
    """One simulation-time TCP tracking sample."""

    actual_velocity: np.ndarray
    actual_speed: float
    target_error: float


class TcpDiagnosticTracker:
    """Calculate TCP velocity from simulation time without wall-clock drift."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.previous_time = None
        self.previous_position = None

    def update(self, timestamp, actual_position, target_position):
        actual = np.asarray(actual_position, dtype=float)
        target = np.asarray(target_position, dtype=float)
        if actual.shape != (3,) or target.shape != (3,):
            raise ValueError("TCP positions must contain three values")
        if not np.all(np.isfinite(actual)) or not np.all(np.isfinite(target)):
            raise ValueError("TCP diagnostic inputs must be finite")
        velocity = np.zeros(3)
        if self.previous_time is not None:
            dt = float(timestamp) - self.previous_time
            if dt > 0.0:
                velocity = (actual - self.previous_position) / dt
            elif dt < 0.0:
                self.reset()
        self.previous_time = float(timestamp)
        self.previous_position = actual.copy()
        return TcpDiagnosticSample(
            velocity,
            float(np.linalg.norm(velocity)),
            float(np.linalg.norm(target - actual)),
        )


@dataclass(frozen=True)
class SurfaceFollowCommand:
    """Output of one surface-following controller update."""

    velocity: np.ndarray
    state: str
    finished: bool
    fault: str = ""


class SurfaceFollower:
    """Approach contact, regulate normal force, and move tangentially."""

    def __init__(
        self,
        approach_direction,
        tangent_velocity,
        target_force,
        force_gain,
        approach_speed,
        maximum_normal_speed,
        contact_force,
        maximum_force,
        duration,
        approach_timeout,
        force_tolerance,
    ):
        direction = np.asarray(approach_direction, dtype=float)
        tangent = np.asarray(tangent_velocity, dtype=float)
        scalars = np.asarray([
            target_force,
            force_gain,
            approach_speed,
            maximum_normal_speed,
            contact_force,
            maximum_force,
            duration,
            approach_timeout,
            force_tolerance,
        ], dtype=float)
        if direction.shape != (3,) or tangent.shape != (3,):
            raise ValueError("surface-follow vectors must contain three values")
        if (
            not np.all(np.isfinite(direction))
            or not np.all(np.isfinite(tangent))
            or not np.all(np.isfinite(scalars))
            or np.any(scalars <= 0.0)
        ):
            raise ValueError("surface-follow parameters must be finite and positive")
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            raise ValueError("surface approach direction cannot be zero")
        self.approach_direction = direction / norm
        # Tangential motion must not secretly change the requested normal force.
        self.tangent_velocity = tangent - (
            np.dot(tangent, self.approach_direction) * self.approach_direction
        )
        self.target_force = float(target_force)
        self.force_gain = float(force_gain)
        self.approach_speed = float(approach_speed)
        self.maximum_normal_speed = float(maximum_normal_speed)
        self.contact_force = float(contact_force)
        self.maximum_force = float(maximum_force)
        self.duration = float(duration)
        self.approach_timeout = float(approach_timeout)
        self.force_tolerance = float(force_tolerance)
        if self.target_force <= self.contact_force:
            raise ValueError("target force must exceed the contact threshold")
        if self.target_force >= self.maximum_force:
            raise ValueError("target force must be below the maximum force")
        self.reset(0.0)

    def reset(self, timestamp) -> None:
        self.started_at = float(timestamp)
        self.follow_started_at = None
        self.follow_elapsed = 0.0
        self.previous_time = float(timestamp)
        self.state = "APPROACH"

    def update(self, timestamp, normal_force, contact_direction=None):
        now = float(timestamp)
        force = float(normal_force)
        if not np.isfinite(now) or not np.isfinite(force) or force < 0.0:
            raise ValueError("surface-follow sample must be finite and non-negative")
        dt = max(0.0, now - self.previous_time)
        self.previous_time = now
        if force >= self.maximum_force:
            self.state = "FAULT"
            return SurfaceFollowCommand(
                np.zeros(3), self.state, True, "maximum contact force exceeded"
            )
        if self.state == "APPROACH":
            if force >= self.contact_force:
                self.state = "ESTABLISH_FORCE"
            elif now - self.started_at >= self.approach_timeout:
                self.state = "FAULT"
                return SurfaceFollowCommand(
                    np.zeros(3), self.state, True, "contact approach timed out"
                )
            else:
                return SurfaceFollowCommand(
                    self.approach_direction * self.approach_speed,
                    self.state,
                    False,
                )
        direction = self.approach_direction
        if contact_direction is not None:
            measured = np.asarray(contact_direction, dtype=float)
            measured_norm = float(np.linalg.norm(measured))
            if measured.shape == (3,) and measured_norm > 1e-12:
                direction = measured / measured_norm
                if np.dot(direction, self.approach_direction) < 0.0:
                    direction = -direction
        normal_speed = float(np.clip(
            self.force_gain * (self.target_force - force),
            -self.maximum_normal_speed,
            self.maximum_normal_speed,
        ))
        if self.state == "FOLLOW" and force < self.contact_force:
            self.state = "ESTABLISH_FORCE"
        if self.state == "ESTABLISH_FORCE":
            if abs(force - self.target_force) <= self.force_tolerance:
                self.state = "FOLLOW"
                self.follow_started_at = now
        elif self.state == "FOLLOW":
            self.follow_elapsed += dt
            if self.follow_elapsed >= self.duration:
                self.state = "COMPLETE"
                return SurfaceFollowCommand(np.zeros(3), self.state, True)
        tangent = np.zeros(3)
        if self.state == "FOLLOW":
            tangent = self.tangent_velocity - (
                np.dot(self.tangent_velocity, direction) * direction
            )
        return SurfaceFollowCommand(
            direction * normal_speed + tangent,
            self.state,
            False,
        )
