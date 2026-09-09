"""Deterministic headless runtime for the JAKA S5 MuJoCo model."""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Iterable, Optional, Sequence

import mujoco
import numpy as np

from massage_mujoco.contact import read_contacts
from massage_mujoco.model import (
    JOINT_NAMES,
    PRONE_BACK_FLEX_NAME,
    build_model,
    load_configuration,
)


class ControlMode(str, Enum):
    """Available owners of the MuJoCo joint actuators."""

    POSITION = "POSITION"
    HOLD = "HOLD"
    GRAVITY = "GRAVITY"


@dataclass(frozen=True)
class SimulationState:
    """Copy of the observable simulation state at one instant."""

    time: float
    position: np.ndarray
    velocity: np.ndarray
    target_position: np.ndarray
    target_velocity: np.ndarray
    commanded_torque: np.ndarray
    torque_controlled: np.ndarray
    actuator_effort: np.ndarray
    tool_position: np.ndarray
    tool_rotation: np.ndarray
    tool_wrench: np.ndarray
    contact_pad_position: float


@dataclass(frozen=True)
class TrajectoryPoint:
    """One absolute-time joint target in a simulation trajectory."""

    time_from_start: float
    positions: Sequence[float]


class MujocoRuntime:
    """Own a model/data pair and expose explicit reset and stepping calls."""

    def __init__(
        self,
        config_path: Optional[Path] = None,
        use_prone_mannequin: bool = False,
        initial_positions_path: Optional[Path] = None,
    ) -> None:
        self.config = load_configuration(config_path)
        self.model, self.contract = build_model(
            config_path,
            use_prone_mannequin=use_prone_mannequin,
            initial_positions_path=initial_positions_path,
        )
        self.data = mujoco.MjData(self.model)
        self._kinematics_data = mujoco.MjData(self.model)
        self._joint_qpos_addresses = np.array(
            [self.model.joint(name).qposadr[0] for name in JOINT_NAMES],
            dtype=int,
        )
        self._joint_dof_addresses = np.array(
            [self.model.joint(name).dofadr[0] for name in JOINT_NAMES],
            dtype=int,
        )
        self._position_actuator_ids = np.array(
            [self.model.actuator(f"{name}_position").id for name in JOINT_NAMES],
            dtype=int,
        )
        self._torque_actuator_ids = np.array(
            [self.model.actuator(f"{name}_torque_Nm").id for name in JOINT_NAMES],
            dtype=int,
        )
        self._actuator_ids = self._position_actuator_ids
        self._nominal_actuator_gainprm = self.model.actuator_gainprm[
            self._position_actuator_ids
        ].copy()
        self._nominal_actuator_biasprm = self.model.actuator_biasprm[
            self._position_actuator_ids
        ].copy()
        self._joint_body_ids = np.array([
            int(self.model.joint(name).bodyid[0]) for name in JOINT_NAMES
        ])
        self._tool_site_id = self.model.site("massage_tool_tip").id
        self._ft_site_id = self.model.site("massage_ft_site").id
        self._tool_body_id = self.model.body("massage_head_link").id
        self._tcp_control_body_id = self.model.body("tcp_control_target").id
        self._tcp_control_mocap_id = int(
            self.model.body_mocapid[self._tcp_control_body_id]
        )
        self._force_sensor_address = self.model.sensor(
            "massage_ft_force"
        ).adr[0]
        self._torque_sensor_address = self.model.sensor(
            "massage_ft_torque"
        ).adr[0]
        self._use_prone_mannequin = bool(use_prone_mannequin)
        if self._use_prone_mannequin:
            self._pad_qpos_address = None
            self._pad_geom_id = None
            self._back_flex_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_FLEX,
                PRONE_BACK_FLEX_NAME,
            )
            self.contact_target_name = PRONE_BACK_FLEX_NAME
        else:
            self._pad_qpos_address = self.model.joint(
                "contact_pad_slide"
            ).qposadr[0]
            self._pad_geom_id = self.model.geom("contact_pad_geom").id
            self._back_flex_id = None
            self.contact_target_name = self.config.contact_monitor["target_body"]
        self._last_step_time = 0.0
        self._time_zero_position = np.zeros(len(JOINT_NAMES))
        self._control_mode = ControlMode.POSITION
        self._servo_velocity = np.zeros(len(JOINT_NAMES))
        self._servo_deadline = None
        self._torque_controlled = np.zeros(len(JOINT_NAMES), dtype=bool)
        self._mouse_joint_index = None
        self._mouse_joint_torque = 0.0
        self.reset()

    @property
    def control_mode(self) -> ControlMode:
        """Return the active joint control mode."""
        return self._control_mode

    def _enable_position_actuators(self) -> None:
        self.model.actuator_gainprm[self._actuator_ids] = (
            self._nominal_actuator_gainprm
        )
        self.model.actuator_biasprm[self._actuator_ids] = (
            self._nominal_actuator_biasprm
        )

    def _disable_position_actuators(self) -> None:
        self.model.actuator_gainprm[self._actuator_ids] = 0.0
        self.model.actuator_biasprm[self._actuator_ids] = 0.0

    def _update_position_actuator_ownership(self) -> None:
        enabled = ~self._torque_controlled
        self._disable_position_actuators()
        self.model.actuator_gainprm[
            self._position_actuator_ids[enabled]
        ] = self._nominal_actuator_gainprm[enabled]
        self.model.actuator_biasprm[
            self._position_actuator_ids[enabled]
        ] = self._nominal_actuator_biasprm[enabled]

    def _physics_step(self) -> None:
        if self._servo_deadline is not None:
            if self.data.time >= self._servo_deadline - 1e-12:
                self.stop_servo()
            else:
                target = self.target_positions() + (
                    self._servo_velocity * self.model.opt.timestep
                )
                bounds = np.array([
                    self.contract.joint_ranges[name] for name in JOINT_NAMES
                ])
                limited = np.clip(target, bounds[:, 0], bounds[:, 1])
                self._servo_velocity[limited != target] = 0.0
                self.data.ctrl[self._actuator_ids] = limited
                # Position actuators implement kp*(q_ref-q)-kd*qvel. Add
                # kd*v_ref inside the actuator so its force limit still applies.
                self.model.actuator_biasprm[self._actuator_ids, 0] = (
                    -self._nominal_actuator_biasprm[:, 2] * self._servo_velocity
                )
        if self.config.gravity_compensation:
            self.data.qfrc_applied[self._joint_dof_addresses] = (
                self.data.qfrc_bias[self._joint_dof_addresses]
            )
        else:
            self.data.qfrc_applied[self._joint_dof_addresses] = 0.0
        if self._mouse_joint_index is not None:
            dof = self._joint_dof_addresses[self._mouse_joint_index]
            self.data.qfrc_applied[dof] += self._mouse_joint_torque
        mujoco.mj_step(self.model, self.data)
        self._last_step_time = float(self.data.time)

    def validate_positions(self, values: Sequence[float]) -> np.ndarray:
        """Return a validated six-joint target in canonical joint order."""
        positions = np.asarray(values, dtype=float)
        if positions.shape != (len(JOINT_NAMES),):
            raise ValueError("expected six joint positions")
        if not np.all(np.isfinite(positions)):
            raise ValueError("joint positions must be finite")
        for index, name in enumerate(JOINT_NAMES):
            lower, upper = self.contract.joint_ranges[name]
            if positions[index] < lower or positions[index] > upper:
                raise ValueError(f"{name} target is outside its URDF limits")
        return positions

    def reset(self, positions: Optional[Sequence[float]] = None) -> SimulationState:
        """Reset all dynamic state and optionally set an initial joint pose."""
        initial = np.array([
            self.contract.initial_positions[name] for name in JOINT_NAMES
        ])
        if positions is not None:
            initial = self.validate_positions(positions)
        self._enable_position_actuators()
        self._servo_velocity[:] = 0.0
        self._servo_deadline = None
        self._torque_controlled[:] = False
        self._mouse_joint_index = None
        self._mouse_joint_torque = 0.0
        self._control_mode = ControlMode.POSITION
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self._joint_qpos_addresses] = initial
        self.data.ctrl[self._actuator_ids] = initial
        self.data.ctrl[self._torque_actuator_ids] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.sync_tcp_control_target()
        mujoco.mj_forward(self.model, self.data)
        self._last_step_time = float(self.data.time)
        self._time_zero_position = initial.copy()
        return self.state()

    @property
    def tcp_control_body_id(self) -> int:
        """Return the selectable body used by the native Viewer."""
        return self._tcp_control_body_id

    def tcp_control_target(self) -> np.ndarray:
        """Return the world-frame XYZ target edited by the native Viewer."""
        return self.data.mocap_pos[self._tcp_control_mocap_id].copy()

    def set_tcp_control_target(self, position: Sequence[float]) -> None:
        """Move the Viewer target without changing the robot state."""
        target = np.asarray(position, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("TCP control target must contain three finite values")
        self.data.mocap_pos[self._tcp_control_mocap_id] = target

    def sync_tcp_control_target(self) -> None:
        """Place the Viewer target at the current tool tip in world axes."""
        self.data.mocap_pos[self._tcp_control_mocap_id] = (
            self.data.site_xpos[self._tool_site_id]
        )
        self.data.mocap_quat[self._tcp_control_mocap_id] = [1.0, 0.0, 0.0, 0.0]

    def tool_pose_for_positions(self, positions: Sequence[float]):
        """Return world TCP pose for joint targets without changing live state."""
        target = self.validate_positions(positions)
        scratch = self._kinematics_data
        scratch.qpos[:] = self.data.qpos
        scratch.qpos[self._joint_qpos_addresses] = target
        mujoco.mj_forward(self.model, scratch)
        return (
            scratch.site_xpos[self._tool_site_id].copy(),
            scratch.site_xmat[self._tool_site_id].reshape(3, 3).copy(),
        )

    def distance_to_contact_pad(self, position: Sequence[float]) -> float:
        """Return shortest outside distance to the active contact target."""
        point = np.asarray(position, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("contact distance point must contain three values")
        if self._back_flex_id is not None:
            flex_id = self._back_flex_id
            start = self.model.flex_vertadr[flex_id]
            count = self.model.flex_vertnum[flex_id]
            vertices = self.data.flexvert_xpos[start:start + count]
            radius = self.model.flex_radius[flex_id]
            low = vertices.min(axis=0) - radius
            high = vertices.max(axis=0) + radius
            outside = np.maximum(np.maximum(low - point, point - high), 0.0)
            return float(np.linalg.norm(outside))
        geom_id = self._pad_geom_id
        rotation = self.data.geom_xmat[geom_id].reshape(3, 3)
        local = rotation.T @ (point - self.data.geom_xpos[geom_id])
        size = self.model.geom_size[geom_id]
        outside = np.maximum(np.abs(local) - size, 0.0)
        return float(np.linalg.norm(outside))

    def target_positions(self) -> np.ndarray:
        """Return detached actuator targets, including native UI edits."""
        return self.data.ctrl[self._actuator_ids].copy()

    def torque_controls(self) -> np.ndarray:
        """Return detached native Viewer joint torque commands."""
        return self.data.ctrl[self._torque_actuator_ids].copy()

    def set_native_torque_control_limit(self, maximum_torque: float) -> None:
        """Set a practical symmetric limit for Viewer torque inputs."""
        self.set_native_torque_control_limits(
            np.full(len(JOINT_NAMES), maximum_torque, dtype=float)
        )

    def set_native_torque_control_limits(
        self, maximum_torques: Sequence[float],
    ) -> None:
        """Set independent symmetric Viewer torque limits for all joints."""
        requested = np.asarray(maximum_torques, dtype=float)
        if (
            requested.shape != (len(JOINT_NAMES),)
            or not np.all(np.isfinite(requested))
            or np.any(requested <= 0.0)
        ):
            raise ValueError(
                "native joint maximum torques must contain six positive values"
            )
        effort_limits = np.array([
            self.contract.effort_limits[name] for name in JOINT_NAMES
        ])
        limits = np.minimum(effort_limits, requested)
        self.model.actuator_ctrlrange[self._torque_actuator_ids, 0] = -limits
        self.model.actuator_ctrlrange[self._torque_actuator_ids, 1] = limits

    @property
    def torque_controlled(self) -> np.ndarray:
        """Return which joints currently accept direct torque input."""
        return self._torque_controlled.copy()

    def joint_index_for_body(self, body_id: int) -> Optional[int]:
        """Map a selected link or tool body to its nearest upstream joint."""
        selected = int(body_id)
        while selected > 0:
            matches = np.flatnonzero(self._joint_body_ids == selected)
            if matches.size:
                return int(matches[0])
            selected = int(self.model.body_parentid[selected])
        return None

    def recover_native_reset(self) -> bool:
        """Restore actuator targets after the native viewer resets MjData."""
        position = self.data.qpos[self._joint_qpos_addresses]
        time_went_back = self.data.time < self._last_step_time
        state_changed_at_time_zero = (
            self.data.time == 0.0
            and not np.allclose(position, self._time_zero_position)
        )
        if not time_went_back and not state_changed_at_time_zero:
            return False
        self.reset()
        return True

    def set_target(self, positions: Sequence[float]) -> None:
        """Set a position target for all six simulation servos."""
        if self._control_mode is not ControlMode.POSITION:
            raise RuntimeError("joint targets require POSITION control mode")
        target = self.validate_positions(positions)
        self.stop_servo()
        self.stop_native_joint_control()
        self.data.ctrl[self._actuator_ids] = target

    def accept_native_position_targets(
        self,
        positions: Sequence[float],
        changed: Sequence[bool],
    ) -> None:
        """Accept Viewer position sliders and release matching torque joints."""
        target = self.validate_positions(positions)
        changed_mask = np.asarray(changed, dtype=bool)
        if changed_mask.shape != (len(JOINT_NAMES),):
            raise ValueError("changed mask must contain six values")
        self.stop_servo()
        self._torque_controlled[changed_mask] = False
        self.data.ctrl[self._torque_actuator_ids[changed_mask]] = 0.0
        self.data.ctrl[self._position_actuator_ids] = target
        self._update_position_actuator_ownership()

    def accept_native_torque_controls(
        self,
        torques: Sequence[float],
        changed: Sequence[bool],
    ) -> None:
        """Accept Viewer torque sliders and release matching position servos."""
        values = np.asarray(torques, dtype=float)
        changed_mask = np.asarray(changed, dtype=bool)
        if values.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(values)):
            raise ValueError("joint torques must contain six finite values")
        if changed_mask.shape != (len(JOINT_NAMES),):
            raise ValueError("changed mask must contain six values")
        limits = self.model.actuator_ctrlrange[self._torque_actuator_ids, 1]
        if np.any(np.abs(values) > limits + 1e-9):
            raise ValueError("joint torque is outside its native control limit")
        self.stop_servo()
        actual = self.data.qpos[self._joint_qpos_addresses]
        self.data.ctrl[self._position_actuator_ids[changed_mask]] = (
            actual[changed_mask]
        )
        self._torque_controlled[changed_mask] = True
        self.data.ctrl[self._torque_actuator_ids] = values
        self._update_position_actuator_ownership()

    def begin_mouse_joint_control(self, joint_index: int) -> None:
        """Release one joint so Viewer rotation acts through dynamics."""
        if joint_index < 0 or joint_index >= len(JOINT_NAMES):
            raise ValueError("joint index is outside the JAKA arm")
        self.stop_servo()
        actual = self.data.qpos[self._joint_qpos_addresses]
        self.data.ctrl[self._position_actuator_ids[joint_index]] = (
            actual[joint_index]
        )
        self.data.ctrl[self._torque_actuator_ids[joint_index]] = 0.0
        self._torque_controlled[joint_index] = True
        self._mouse_joint_index = joint_index
        self._mouse_joint_torque = 0.0
        self._update_position_actuator_ownership()

    def update_mouse_joint_torque(
        self,
        body_id: int,
        maximum_torque: float,
    ) -> float:
        """Project a Viewer body perturbation onto one joint and clamp it."""
        if self._mouse_joint_index is None:
            return 0.0
        if not np.isfinite(maximum_torque) or maximum_torque <= 0.0:
            raise ValueError("maximum mouse joint torque must be positive")
        body = int(body_id)
        wrench = self.data.xfrc_applied[body].copy()
        generalized = np.zeros(self.model.nv)
        mujoco.mj_applyFT(
            self.model,
            self.data,
            wrench[:3],
            wrench[3:],
            self.data.xipos[body],
            body,
            generalized,
        )
        self.data.xfrc_applied[body] = 0.0
        index = self._mouse_joint_index
        limit = min(
            self.contract.effort_limits[JOINT_NAMES[index]],
            float(maximum_torque),
        )
        torque = float(generalized[self._joint_dof_addresses[index]])
        self._mouse_joint_torque = float(np.clip(torque, -limit, limit))
        return self._mouse_joint_torque

    def end_mouse_joint_control(self) -> None:
        """Hold the mouse-controlled joint at release position."""
        if self._mouse_joint_index is None:
            return
        index = self._mouse_joint_index
        actual = self.data.qpos[self._joint_qpos_addresses]
        self.data.ctrl[self._position_actuator_ids[index]] = actual[index]
        self.data.ctrl[self._torque_actuator_ids[index]] = 0.0
        self._torque_controlled[index] = False
        self._mouse_joint_index = None
        self._mouse_joint_torque = 0.0
        self._update_position_actuator_ownership()

    def stop_native_joint_control(self) -> None:
        """Restore all position servos and clear native joint torques."""
        self.end_mouse_joint_control()
        self._torque_controlled[:] = False
        self.data.ctrl[self._torque_actuator_ids] = 0.0
        self._mouse_joint_index = None
        self._mouse_joint_torque = 0.0
        self._update_position_actuator_ownership()

    def set_servo_target(
        self, positions: Sequence[float], velocities: Sequence[float], timeout: float,
    ) -> None:
        """Execute a protected Servo velocity stream in simulation time."""
        if self._control_mode is not ControlMode.POSITION:
            raise RuntimeError("Servo targets require POSITION control mode")
        self.validate_positions(positions)
        velocity = np.asarray(velocities, dtype=float)
        if velocity.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(velocity)):
            raise ValueError("Servo velocities must contain six finite values")
        limits = np.array([self.contract.velocity_limits[name] for name in JOINT_NAMES])
        if np.any(np.abs(velocity) > limits + 1e-9):
            raise ValueError("Servo velocity is outside its URDF limits")
        if not np.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("Servo timeout must be finite and positive")
        target = self.target_positions()
        actual = self.data.qpos[self._joint_qpos_addresses]
        if self._servo_deadline is None:
            target = actual.copy()
        # A Servo halt must discard accumulated following error, also for a
        # single halted joint, rather than finish a previously integrated move.
        target[velocity == 0.0] = actual[velocity == 0.0]
        self.data.ctrl[self._actuator_ids] = target
        self._servo_velocity = velocity.copy()
        self._servo_deadline = float(self.data.time) + timeout

    def stop_servo(self) -> None:
        """Clear velocity feedforward and capture actual position on stream exit."""
        if self._servo_deadline is not None:
            self.data.ctrl[self._actuator_ids] = (
                self.data.qpos[self._joint_qpos_addresses]
            )
        self._servo_deadline = None
        self._servo_velocity[:] = 0.0
        self.model.actuator_biasprm[self._actuator_ids, 0] = 0.0
        if self._control_mode is ControlMode.POSITION:
            self._update_position_actuator_ownership()

    def set_control_mode(self, mode: ControlMode) -> SimulationState:
        """Switch actuator ownership without introducing a target jump."""
        requested = ControlMode(mode)
        self.stop_servo()
        self.stop_native_joint_control()
        position = self.data.qpos[self._joint_qpos_addresses].copy()
        self.data.ctrl[self._actuator_ids] = position
        if requested is ControlMode.GRAVITY:
            self._disable_position_actuators()
        else:
            self._enable_position_actuators()
        self._control_mode = requested
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def set_tool_external_wrench(
        self,
        force: Sequence[float],
        torque: Sequence[float],
    ) -> None:
        """Apply a wrench at the FT origin, expressed in the sensor frame."""
        local_force = self._wrench_vector(force, "force")
        local_torque = self._wrench_vector(torque, "torque")
        rotation = self.data.site_xmat[self._ft_site_id].reshape(3, 3)
        world_force = rotation @ local_force
        world_torque_at_site = rotation @ local_torque
        site_to_com = (
            self.data.xipos[self._tool_body_id]
            - self.data.site_xpos[self._ft_site_id]
        )
        world_torque_at_com = (
            world_torque_at_site - np.cross(site_to_com, world_force)
        )
        self.data.xfrc_applied[self._tool_body_id, :3] = world_force
        self.data.xfrc_applied[self._tool_body_id, 3:] = world_torque_at_com

    def clear_tool_external_wrench(self) -> None:
        """Remove the deterministic external wrench applied to the tool."""
        self.data.xfrc_applied[self._tool_body_id] = 0.0

    @staticmethod
    def _wrench_vector(values: Sequence[float], label: str) -> np.ndarray:
        vector = np.asarray(values, dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{label} must contain three finite values")
        return vector

    def step(self, steps: int = 1) -> SimulationState:
        """Advance the simulation by an exact number of physics steps."""
        if steps < 1:
            raise ValueError("steps must be at least one")
        self.recover_native_reset()
        for _ in range(steps):
            self._physics_step()
        return self.state()

    def move_to(
        self,
        positions: Sequence[float],
        duration: float,
        settle_duration: float = 0.5,
    ) -> SimulationState:
        """Linearly ramp the servo target, then allow it to settle."""
        if duration <= 0.0 or settle_duration < 0.0:
            raise ValueError("duration must be positive and settle duration nonnegative")
        return self.execute_trajectory(
            [TrajectoryPoint(time_from_start=duration, positions=positions)],
            settle_duration=settle_duration,
        )

    def execute_trajectory(
        self,
        points: Iterable[TrajectoryPoint],
        settle_duration: float = 0.0,
    ) -> SimulationState:
        """Execute strictly time-ordered points with linear target interpolation."""
        trajectory = list(points)
        if not trajectory:
            raise ValueError("trajectory must contain at least one point")
        if settle_duration < 0.0:
            raise ValueError("settle duration must be nonnegative")

        previous_time = 0.0
        previous_target = self.state().position
        for point in trajectory:
            if not np.isfinite(point.time_from_start):
                raise ValueError("trajectory times must be finite")
            if point.time_from_start <= previous_time:
                raise ValueError("trajectory times must be strictly increasing")
            target = self.validate_positions(point.positions)
            segment_duration = point.time_from_start - previous_time
            step_count = max(1, round(segment_duration / self.model.opt.timestep))
            for step_index in range(1, step_count + 1):
                alpha = step_index / step_count
                self.set_target(previous_target + alpha * (target - previous_target))
                self._physics_step()
            previous_time = point.time_from_start
            previous_target = target

        settle_steps = round(settle_duration / self.model.opt.timestep)
        for _ in range(settle_steps):
            self._physics_step()
        return self.state()

    def contacts(self):
        """Read contact points independently of state to keep core stepping cheap."""
        return read_contacts(
            self.model, self.data, self.config.contact_monitor['tool_body'],
            self.contact_target_name)

    def state(self) -> SimulationState:
        """Return detached arrays so callers cannot mutate MuJoCo state."""
        raw_force = self.data.sensordata[
            self._force_sensor_address:self._force_sensor_address + 3
        ]
        raw_torque = self.data.sensordata[
            self._torque_sensor_address:self._torque_sensor_address + 3
        ]
        return SimulationState(
            time=float(self.data.time),
            position=self.data.qpos[self._joint_qpos_addresses].copy(),
            velocity=self.data.qvel[self._joint_dof_addresses].copy(),
            target_position=self.data.ctrl[self._actuator_ids].copy(),
            target_velocity=self._servo_velocity.copy(),
            commanded_torque=self.data.ctrl[self._torque_actuator_ids].copy(),
            torque_controlled=self._torque_controlled.copy(),
            actuator_effort=self.data.qfrc_actuator[
                self._joint_dof_addresses
            ].copy(),
            tool_position=self.data.site_xpos[self._tool_site_id].copy(),
            tool_rotation=self.data.site_xmat[self._tool_site_id].reshape(3, 3).copy(),
            # MuJoCo reports parent-on-child; Gazebo is configured child-to-parent.
            tool_wrench=-np.concatenate((raw_force, raw_torque)),
            contact_pad_position=(
                float(self.data.qpos[self._pad_qpos_address])
                if self._pad_qpos_address is not None
                else 0.0
            ),
        )
