"""ROS 2 adapter for the MuJoCo simulation core and optional viewer."""

from contextlib import nullcontext
from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import threading
import time
from typing import Optional, Sequence

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, WrenchStamped
import mujoco
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from massage_msgs.msg import ContactPoint, ContactState
from massage_mujoco.contact import ContactMonitor
from massage_mujoco.features import (
    SurfaceFollower,
    TeachingTrajectory,
    TcpDiagnosticTracker,
)
from massage_mujoco.model import JOINT_NAMES
from massage_mujoco.native_control import limited_linear_velocity
from massage_mujoco.runtime import ControlMode, MujocoRuntime, TrajectoryPoint
from massage_mujoco.trajectory import prepare_trajectory, sample_trajectory


@dataclass
class _ActiveTrajectory:
    goal_handle: object
    points: Sequence[TrajectoryPoint]
    start_positions: np.ndarray
    start_time: float
    done: threading.Event
    error_code: int = FollowJointTrajectory.Result.SUCCESSFUL
    error_string: str = ""


@dataclass
class _TeachReplay:
    points: Sequence[TrajectoryPoint]
    start_positions: np.ndarray
    start_time: float


class _ControlOwner(str, Enum):
    IDLE = "IDLE"
    ACTION = "FOLLOW_JOINT_TRAJECTORY"
    SERVO = "MOVEIT_SERVO"
    NATIVE_JOINT = "MUJOCO_JOINT_CONTROL"
    NATIVE_TCP = "MUJOCO_TCP_CONTROL"
    TEACH_REPLAY = "TEACH_REPLAY"
    SURFACE_FOLLOW = "SURFACE_FOLLOW"


def _time_message(seconds: float) -> Time:
    whole_seconds = math.floor(seconds)
    nanoseconds = round((seconds - whole_seconds) * 1e9)
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Time(sec=whole_seconds, nanosec=nanoseconds)


def _joint_mouse_perturbation_active(active: int) -> bool:
    """Return whether the Viewer is rotating or pulling a selected body."""
    joint_perturbations = (
        int(mujoco.mjtPertBit.mjPERT_ROTATE)
        | int(mujoco.mjtPertBit.mjPERT_TRANSLATE)
    )
    return bool(int(active) & joint_perturbations)


class MujocoNode(Node):
    """Expose MuJoCo state and position trajectories through ROS 2."""

    def __init__(self, **kwargs) -> None:
        super().__init__("massage_mujoco", **kwargs)
        self.declare_parameter("config_path", "")
        self.declare_parameter("initial_positions_file", "")
        self.declare_parameter("use_prone_mannequin", False)
        self.declare_parameter("physics_steps_per_update", 10)
        self.declare_parameter("realtime_factor", 1.0)
        self.declare_parameter("goal_tolerance", 0.01)
        self.declare_parameter("goal_time_tolerance", 2.0)
        self.declare_parameter("servo_command_timeout", 0.2)
        self.declare_parameter("wrench_frame_id", "massage_head_link")
        self.declare_parameter("use_mujoco_viewer", False)
        self.declare_parameter("show_mujoco_left_ui", True)
        self.declare_parameter("show_mujoco_right_ui", True)
        self.declare_parameter("use_tcp_control_ball", True)
        self.declare_parameter("tcp_control_linear_gain", 6.0)
        self.declare_parameter("tcp_control_maximum_speed", 0.50)
        self.declare_parameter("tcp_control_position_tolerance", 0.001)
        self.declare_parameter("tcp_control_maximum_distance", 0.30)
        # Viewer torque commands are added on top of bias-force compensation.
        # Keep the default range narrow enough for precise slider adjustment;
        # larger experiments can explicitly override the launch parameter.
        self.declare_parameter("joint_control_maximum_torque", 5.0)
        self.declare_parameter("joint_2_control_maximum_torque", 13.37)
        self.declare_parameter("joint_3_control_maximum_torque", 3.510)
        self.declare_parameter("joint_4_control_maximum_torque", 0.1158)
        self.declare_parameter("joint_5_control_maximum_torque", 0.02179)
        self.declare_parameter("joint_6_control_maximum_torque", 0.0003014)
        self.declare_parameter("joint_mouse_control_maximum_torque", 50.0)
        self.declare_parameter(
            "teach_trajectory_file", "log/mujoco/teach_trajectory.yaml"
        )
        self.declare_parameter("teach_replay_maximum_joint_speed", 0.4)
        self.declare_parameter("teach_replay_minimum_segment_duration", 0.2)
        self.declare_parameter("tcp_diagnostic_trace_length", 500)
        self.declare_parameter("tcp_diagnostic_publish_period", 0.05)
        self.declare_parameter("surface_follow_approach_x", 0.0)
        self.declare_parameter("surface_follow_approach_y", -1.0)
        self.declare_parameter("surface_follow_approach_z", 0.0)
        self.declare_parameter("surface_follow_tangent_x", 0.02)
        self.declare_parameter("surface_follow_tangent_y", 0.0)
        self.declare_parameter("surface_follow_tangent_z", 0.0)
        self.declare_parameter("surface_follow_target_force", 2.0)
        self.declare_parameter("surface_follow_force_gain", 0.01)
        self.declare_parameter("surface_follow_approach_speed", 0.01)
        self.declare_parameter("surface_follow_maximum_normal_speed", 0.02)
        self.declare_parameter("surface_follow_duration", 3.0)
        self.declare_parameter("surface_follow_approach_timeout", 10.0)
        self.declare_parameter("surface_follow_maximum_approach_distance", 0.05)
        self.declare_parameter("surface_follow_force_tolerance", 0.2)
        for name in ("show_contact_points", "show_contact_forces",
                     "show_collision_proxies", "show_site_frames"):
            self.declare_parameter(name, False)

        raw_config_path = self.get_parameter("config_path").value
        config_path = Path(raw_config_path) if raw_config_path else None
        raw_initial_positions_path = self.get_parameter(
            "initial_positions_file"
        ).value
        initial_positions_path = (
            Path(raw_initial_positions_path) if raw_initial_positions_path else None
        )
        self._runtime = MujocoRuntime(
            config_path,
            use_prone_mannequin=bool(
                self.get_parameter("use_prone_mannequin").value
            ),
            initial_positions_path=initial_positions_path,
        )
        self._contact_monitor = ContactMonitor(self._runtime.config.contact_monitor)
        self._steps_per_update = int(
            self.get_parameter("physics_steps_per_update").value
        )
        realtime_factor = float(self.get_parameter("realtime_factor").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self._goal_time_tolerance = float(
            self.get_parameter("goal_time_tolerance").value
        )
        self._servo_command_timeout = float(
            self.get_parameter("servo_command_timeout").value
        )
        self._wrench_frame_id = str(
            self.get_parameter("wrench_frame_id").value
        )
        self._tcp_ball_enabled = bool(
            self.get_parameter("use_tcp_control_ball").value
        )
        self._tcp_ball_gain = float(
            self.get_parameter("tcp_control_linear_gain").value
        )
        self._tcp_ball_maximum_speed = float(
            self.get_parameter("tcp_control_maximum_speed").value
        )
        self._tcp_ball_tolerance = float(
            self.get_parameter("tcp_control_position_tolerance").value
        )
        self._tcp_ball_maximum_distance = float(
            self.get_parameter("tcp_control_maximum_distance").value
        )
        self._joint_control_maximum_torque = float(
            self.get_parameter("joint_control_maximum_torque").value
        )
        self._joint_control_maximum_torques = np.array(
            [self._joint_control_maximum_torque] + [
                float(self.get_parameter(
                    f"joint_{index}_control_maximum_torque"
                ).value)
                for index in range(2, len(JOINT_NAMES) + 1)
            ]
        )
        self._joint_mouse_control_maximum_torque = float(
            self.get_parameter("joint_mouse_control_maximum_torque").value
        )
        self._teach_file = Path(
            str(self.get_parameter("teach_trajectory_file").value)
        )
        self._teach_maximum_speed = float(
            self.get_parameter("teach_replay_maximum_joint_speed").value
        )
        self._teach_minimum_duration = float(
            self.get_parameter("teach_replay_minimum_segment_duration").value
        )
        self._tcp_trace_length = int(
            self.get_parameter("tcp_diagnostic_trace_length").value
        )
        self._tcp_diagnostic_period = float(
            self.get_parameter("tcp_diagnostic_publish_period").value
        )
        self._surface_parameters = {
            "approach_direction": [
                self.get_parameter(f"surface_follow_approach_{axis}").value
                for axis in "xyz"
            ],
            "tangent_velocity": [
                self.get_parameter(f"surface_follow_tangent_{axis}").value
                for axis in "xyz"
            ],
            "target_force": self.get_parameter("surface_follow_target_force").value,
            "force_gain": self.get_parameter("surface_follow_force_gain").value,
            "approach_speed": self.get_parameter(
                "surface_follow_approach_speed"
            ).value,
            "maximum_normal_speed": self.get_parameter(
                "surface_follow_maximum_normal_speed"
            ).value,
            "contact_force": self._runtime.config.contact_monitor["contact_force"],
            "maximum_force": self._runtime.config.contact_monitor["over_force"],
            "duration": self.get_parameter("surface_follow_duration").value,
            "approach_timeout": self.get_parameter(
                "surface_follow_approach_timeout"
            ).value,
            "force_tolerance": self.get_parameter(
                "surface_follow_force_tolerance"
            ).value,
        }
        self._surface_maximum_approach_distance = float(
            self.get_parameter(
                "surface_follow_maximum_approach_distance"
            ).value
        )
        use_mujoco_viewer = bool(
            self.get_parameter("use_mujoco_viewer").value
        )
        show_mujoco_left_ui = bool(
            self.get_parameter("show_mujoco_left_ui").value
        )
        show_mujoco_right_ui = bool(
            self.get_parameter("show_mujoco_right_ui").value
        )
        if self._steps_per_update < 1:
            raise ValueError("physics_steps_per_update must be positive")
        if realtime_factor <= 0.0:
            raise ValueError("realtime_factor must be positive")
        if self._goal_tolerance <= 0.0 or self._goal_time_tolerance < 0.0:
            raise ValueError("trajectory tolerances are invalid")
        if self._servo_command_timeout <= 0.0:
            raise ValueError("servo_command_timeout must be positive")
        tcp_parameters = np.asarray([
            self._tcp_ball_gain,
            self._tcp_ball_maximum_speed,
            self._tcp_ball_tolerance,
            self._tcp_ball_maximum_distance,
        ])
        if not np.all(np.isfinite(tcp_parameters)) or np.any(tcp_parameters <= 0.0):
            raise ValueError("TCP control ball parameters must be finite and positive")
        if (
            not np.all(np.isfinite(self._joint_control_maximum_torques))
            or np.any(self._joint_control_maximum_torques <= 0.0)
        ):
            raise ValueError("joint control maximum torques must be positive")
        if (
            not np.isfinite(self._joint_mouse_control_maximum_torque)
            or self._joint_mouse_control_maximum_torque <= 0.0
        ):
            raise ValueError("joint mouse control maximum torque must be positive")
        if (
            not np.isfinite(self._teach_maximum_speed)
            or self._teach_maximum_speed <= 0.0
            or not np.isfinite(self._teach_minimum_duration)
            or self._teach_minimum_duration <= 0.0
        ):
            raise ValueError("teaching replay timing parameters must be positive")
        if self._tcp_trace_length < 2:
            raise ValueError("tcp_diagnostic_trace_length must be at least two")
        if (
            not np.isfinite(self._tcp_diagnostic_period)
            or self._tcp_diagnostic_period <= 0.0
        ):
            raise ValueError("tcp_diagnostic_publish_period must be positive")
        # Construct once here so bad force-control parameters fail at startup.
        SurfaceFollower(**self._surface_parameters)
        if (
            not np.isfinite(self._surface_maximum_approach_distance)
            or self._surface_maximum_approach_distance <= 0.0
        ):
            raise ValueError("surface maximum approach distance must be positive")
        self._runtime.set_native_torque_control_limits(
            self._joint_control_maximum_torques
        )

        self._lock = threading.RLock()
        self._active: Optional[_ActiveTrajectory] = None
        self._goal_reserved = False
        self._reserved_goal_interrupted = False
        self._reserved_goal_error = ""
        self._last_servo_command_time: Optional[float] = None
        self._control_owner = _ControlOwner.IDLE
        self._expected_joint_target = self._runtime.target_positions()
        self._expected_joint_torque = self._runtime.torque_controls()
        self._mouse_joint_index = None
        self._mouse_joint_body_id = None
        self._tcp_ball_active = False
        self._tcp_ball_last_target = self._runtime.tcp_control_target()
        self._ignore_servo_commands_until = 0.0
        self._clock_offset = 0.0
        self._last_simulation_time = float(self._runtime.data.time)
        self._last_clock_time = self._last_simulation_time
        self._teaching = TeachingTrajectory(len(JOINT_NAMES))
        self._teach_replay: Optional[_TeachReplay] = None
        self._surface_follower = None
        self._surface_stop_pending = False
        self._latest_contacts = []
        self._tcp_diagnostics = TcpDiagnosticTracker()
        self._tcp_actual_trace = []
        self._tcp_target_trace = []
        self._last_tcp_command = np.zeros(3)
        self._last_tcp_command_time = None
        self._last_tcp_diagnostic_time = -math.inf
        self._last_surface_status = None
        self._last_surface_status_time = -math.inf
        self._viewer = None
        if use_mujoco_viewer:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(
                self._runtime.model,
                self._runtime.data,
                show_left_ui=show_mujoco_left_ui,
                show_right_ui=show_mujoco_right_ui,
            )
            with self._viewer.lock():
                self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                self._viewer.cam.lookat[:] = [0.30, 0.0, 0.55]
                self._viewer.cam.distance = 1.75
                self._viewer.cam.azimuth = 135.0
                self._viewer.cam.elevation = -18.0
                self._viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = bool(
                    self.get_parameter("show_contact_points").value)
                self._viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = bool(
                    self.get_parameter("show_contact_forces").value)
                self._viewer.opt.geomgroup[3] = bool(
                    self.get_parameter("show_collision_proxies").value)
                self._viewer.opt.geomgroup[4] = self._tcp_ball_enabled
                if self.get_parameter("show_site_frames").value:
                    self._viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
                if self._tcp_ball_enabled:
                    self._select_tcp_control_ball()
            self._viewer.sync()
            self._viewer.set_texts((
                mujoco.mjtFontScale.mjFONTSCALE_100,
                mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                "Joint mouse control",
                "Double-click link; Ctrl + left rotates, right pulls",
            ))
            self.get_logger().info("MuJoCo viewer started")
        callback_group = ReentrantCallbackGroup()
        self._contact_publisher = self.create_publisher(
            ContactState, "/massage_mujoco/contact_state", 10)
        self._contact_event_publisher = self.create_publisher(
            ContactState, "/massage_mujoco/contact_events", 100)
        self._clock_publisher = self.create_publisher(Clock, "/clock", 1)
        self._joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self._controller_state_publisher = self.create_publisher(
            JointTrajectoryControllerState,
            "/jaka_s5_controller/controller_state",
            10,
        )
        mode_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._control_mode_publisher = self.create_publisher(
            String,
            "/massage_mujoco/control_mode",
            mode_qos,
        )
        self._control_owner_publisher = self.create_publisher(
            String,
            "/massage_mujoco/control_owner",
            mode_qos,
        )
        self._wrench_publisher = self.create_publisher(
            WrenchStamped, "/massage_ft_broadcaster/wrench", 10
        )
        self._cartesian_jog_publisher = self.create_publisher(
            TwistStamped, "/massage/cartesian_jog/command", 10
        )
        self._teach_status_publisher = self.create_publisher(
            String, "/massage_mujoco/teach/status", mode_qos
        )
        self._surface_status_publisher = self.create_publisher(
            String, "/massage_mujoco/surface_follow/status", mode_qos
        )
        self._tcp_actual_pose_publisher = self.create_publisher(
            PoseStamped, "/massage_mujoco/tcp/actual_pose", 10
        )
        self._tcp_target_pose_publisher = self.create_publisher(
            PoseStamped, "/massage_mujoco/tcp/target_pose", 10
        )
        self._tcp_actual_velocity_publisher = self.create_publisher(
            TwistStamped, "/massage_mujoco/tcp/actual_velocity", 10
        )
        self._tcp_diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/massage_mujoco/tcp/diagnostics", 10
        )
        self._tcp_trace_publisher = self.create_publisher(
            MarkerArray, "/massage_mujoco/tcp/traces", 10
        )
        self._mode_services = [
            self.create_service(
                Trigger,
                "/massage_mujoco/set_position_mode",
                lambda request, response: self._set_control_mode(
                    ControlMode.POSITION, request, response
                ),
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/set_hold_mode",
                lambda request, response: self._set_control_mode(
                    ControlMode.HOLD, request, response
                ),
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/set_gravity_mode",
                lambda request, response: self._set_control_mode(
                    ControlMode.GRAVITY, request, response
                ),
                callback_group=callback_group,
            ),
        ]
        self._teach_services = [
            self.create_service(
                Trigger,
                "/massage_mujoco/teach/record_point",
                self._record_teaching_point,
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/teach/clear",
                self._clear_teaching_points,
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/teach/save",
                self._save_teaching_points,
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/teach/load",
                self._load_teaching_points,
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/teach/replay",
                self._start_teaching_replay,
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/teach/stop",
                self._stop_teaching_replay_service,
                callback_group=callback_group,
            ),
        ]
        self._surface_services = [
            self.create_service(
                Trigger,
                "/massage_mujoco/surface_follow/start",
                self._start_surface_follow,
                callback_group=callback_group,
            ),
            self.create_service(
                Trigger,
                "/massage_mujoco/surface_follow/stop",
                self._stop_surface_follow_service,
                callback_group=callback_group,
            ),
        ]
        self._cartesian_jog_subscription = self.create_subscription(
            TwistStamped,
            "/massage/cartesian_jog/command",
            self._track_tcp_command,
            10,
            callback_group=callback_group,
        )
        self._servo_command_subscription = self.create_subscription(
            JointTrajectory,
            "/jaka_s5_controller/joint_trajectory",
            self._servo_command_callback,
            10,
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/jaka_s5_controller/follow_joint_trajectory",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        simulation_period = self._runtime.model.opt.timestep * self._steps_per_update
        self._timer = self.create_timer(
            simulation_period / realtime_factor,
            self._advance,
            callback_group=callback_group,
        )
        self._publish_state(
            stamp_seconds=self._update_clock_time(float(self._runtime.data.time))
        )
        self._publish_control_mode()
        self._publish_control_owner()
        self._publish_teaching_status("IDLE")
        self._publish_surface_status("IDLE")
        self.get_logger().info(
            "MuJoCo ready: dt=%.6f s, %d steps/update, "
            "realtime_factor=%.3f, viewer=%s"
            % (
                self._runtime.model.opt.timestep,
                self._steps_per_update,
                realtime_factor,
                "on" if self._viewer is not None else "off",
            )
        )

    def _viewer_lock(self):
        if self._viewer is None:
            return nullcontext()
        return self._viewer.lock()

    def _select_tcp_control_ball(self) -> None:
        """Preselect the mocap target for native Viewer perturb controls."""
        if self._viewer is None:
            return
        perturb = self._viewer.perturb
        target = self._runtime.tcp_control_target()
        perturb.select = self._runtime.tcp_control_body_id
        perturb.flexselect = -1
        perturb.skinselect = -1
        perturb.localpos[:] = 0.0
        perturb.refpos[:] = target
        perturb.refselpos[:] = target
        perturb.refquat[:] = [1.0, 0.0, 0.0, 0.0]

    def _sync_tcp_control_ball(self) -> None:
        self._runtime.sync_tcp_control_target()
        self._tcp_ball_last_target = self._runtime.tcp_control_target()

    def _stop_tcp_control_ball(self) -> None:
        self._tcp_ball_active = False
        self._sync_tcp_control_ball()

    def _interrupt_trajectory(self, reason: str) -> None:
        active = self._active
        if active is not None and not active.done.is_set():
            active.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            active.error_string = reason
            active.done.set()
        if self._goal_reserved:
            self._reserved_goal_interrupted = True
            self._reserved_goal_error = reason
            self._goal_reserved = False

    def _update_clock_time(self, simulation_time: float) -> float:
        """Keep ROS time monotonic when native Reset rewinds MjData time."""
        if simulation_time < self._last_simulation_time:
            update_period = (
                self._runtime.model.opt.timestep * self._steps_per_update
            )
            self._clock_offset = (
                self._last_clock_time + update_period - simulation_time
            )
        clock_time = self._clock_offset + simulation_time
        self._last_simulation_time = simulation_time
        self._last_clock_time = clock_time
        return clock_time

    def _sync_viewer(self) -> None:
        if self._viewer is None:
            return
        if not self._viewer.is_running():
            self.get_logger().info(
                "MuJoCo viewer closed; simulation continues headless"
            )
            self._viewer = None
            self._tcp_ball_active = False
            with self._lock:
                self._runtime.stop_native_joint_control()
                self._expected_joint_target = self._runtime.target_positions()
                self._expected_joint_torque = self._runtime.torque_controls()
                self._mouse_joint_index = None
                self._mouse_joint_body_id = None
            if self._control_owner in (
                _ControlOwner.NATIVE_JOINT,
                _ControlOwner.NATIVE_TCP,
            ):
                self._set_control_owner(_ControlOwner.IDLE)
            return
        self._viewer.sync()

    def _close_viewer(self) -> None:
        if self._viewer is None:
            return
        viewer = self._viewer
        self._viewer = None
        viewer.close()
        deadline = time.monotonic() + 2.0
        while viewer.m is not None and time.monotonic() < deadline:
            time.sleep(0.01)

    def _publish_teaching_status(self, state: str, detail: str = "") -> None:
        message = f"state={state}; points={len(self._teaching.waypoints)}"
        if detail:
            message += f"; detail={detail}"
        self._teach_status_publisher.publish(String(data=message))

    def _record_teaching_point(self, _request, response):
        with self._lock:
            if self._teach_replay is not None or self._surface_follower is not None:
                response.success = False
                response.message = "cannot record while an automatic motion is active"
                return response
            with self._viewer_lock():
                positions = self._runtime.state().position
            try:
                count = self._teaching.record(positions)
            except ValueError as error:
                response.success = False
                response.message = str(error)
                return response
            self._publish_teaching_status("READY", f"recorded point {count}")
        response.success = True
        response.message = f"recorded teaching point {count}"
        return response

    def _clear_teaching_points(self, _request, response):
        with self._lock:
            if self._teach_replay is not None:
                response.success = False
                response.message = "cannot clear while teaching replay is active"
                return response
            self._teaching.clear()
            self._publish_teaching_status("IDLE", "trajectory cleared")
        response.success = True
        response.message = "teaching trajectory cleared"
        return response

    def _save_teaching_points(self, _request, response):
        with self._lock:
            if not self._teaching.waypoints:
                response.success = False
                response.message = "teaching trajectory contains no points"
                return response
            try:
                self._teaching.save(self._teach_file, JOINT_NAMES)
            except (OSError, ValueError) as error:
                response.success = False
                response.message = f"failed to save teaching trajectory: {error}"
                return response
        response.success = True
        response.message = f"saved teaching trajectory to {self._teach_file}"
        return response

    def _load_teaching_points(self, _request, response):
        with self._lock:
            if self._teach_replay is not None:
                response.success = False
                response.message = "cannot load while teaching replay is active"
                return response
            try:
                count = self._teaching.load(self._teach_file, JOINT_NAMES)
                for point in self._teaching.waypoints:
                    self._runtime.validate_positions(point)
            except (OSError, ValueError) as error:
                response.success = False
                response.message = f"failed to load teaching trajectory: {error}"
                return response
            self._publish_teaching_status("READY", "trajectory loaded")
        response.success = True
        response.message = f"loaded {count} teaching points"
        return response

    def _start_teaching_replay(self, _request, response):
        with self._lock:
            if self._runtime.control_mode is not ControlMode.POSITION:
                response.success = False
                response.message = "teaching replay requires POSITION mode"
                return response
            if (
                self._active is not None
                or self._goal_reserved
                or self._teach_replay is not None
                or self._surface_follower is not None
                or self._servo_owner_is_active()
                or self._control_owner is not _ControlOwner.IDLE
            ):
                response.success = False
                response.message = "controller is busy"
                return response
            with self._viewer_lock():
                state = self._runtime.state()
                try:
                    points = self._teaching.timed_points(
                        state.position,
                        self._teach_maximum_speed,
                        self._teach_minimum_duration,
                    )
                    for point in points:
                        self._runtime.validate_positions(point.positions)
                except ValueError as error:
                    response.success = False
                    response.message = str(error)
                    return response
                self._runtime.set_target(state.position)
                self._expected_joint_target = state.position.copy()
            self._teach_replay = _TeachReplay(
                points=points,
                start_positions=state.position.copy(),
                start_time=state.time,
            )
            self._stop_tcp_control_ball()
            self._set_control_owner(_ControlOwner.TEACH_REPLAY)
            self._publish_teaching_status("REPLAYING")
        response.success = True
        response.message = f"replaying {len(points)} teaching points"
        return response

    def _stop_teaching_replay(
        self, detail: str, viewer_locked: bool = False
    ) -> bool:
        if self._teach_replay is None:
            return False
        context = nullcontext() if viewer_locked else self._viewer_lock()
        with context:
            actual = self._runtime.state().position
            self._runtime.set_target(actual)
            self._expected_joint_target = actual.copy()
        self._teach_replay = None
        self._set_control_owner(_ControlOwner.IDLE)
        self._publish_teaching_status("IDLE", detail)
        return True

    def _stop_teaching_replay_service(self, _request, response):
        with self._lock:
            stopped = self._stop_teaching_replay("stopped by service")
        response.success = stopped
        response.message = (
            "teaching replay stopped" if stopped else "teaching replay is not active"
        )
        return response

    def _publish_surface_status(
        self, state: str, detail: str = "", force: bool = False
    ) -> None:
        now = time.monotonic()
        if (
            not force
            and state == self._last_surface_status
            and now - self._last_surface_status_time < 0.1
        ):
            return
        message = f"state={state}"
        if detail:
            message += f"; detail={detail}"
        self._surface_status_publisher.publish(String(data=message))
        self._last_surface_status = state
        self._last_surface_status_time = now

    def _start_surface_follow(self, _request, response):
        with self._lock:
            if self._runtime.control_mode is not ControlMode.POSITION:
                response.success = False
                response.message = "surface following requires POSITION mode"
                return response
            if self._cartesian_jog_publisher.get_subscription_count() <= 1:
                response.success = False
                response.message = "Cartesian jog bridge is not connected"
                return response
            if (
                self._active is not None
                or self._goal_reserved
                or self._teach_replay is not None
                or self._surface_follower is not None
                or self._servo_owner_is_active()
                or self._control_owner is not _ControlOwner.IDLE
            ):
                response.success = False
                response.message = "controller is busy"
                return response
            with self._viewer_lock():
                state = self._runtime.state()
                distance = self._runtime.distance_to_contact_pad(
                    state.tool_position
                )
                if distance > self._surface_maximum_approach_distance:
                    response.success = False
                    response.message = (
                        "TCP is %.3f m from contact pad; move to a pre-contact "
                        "pose within %.3f m first"
                        % (distance, self._surface_maximum_approach_distance)
                    )
                    return response
                self._runtime.set_target(state.position)
                self._expected_joint_target = state.position.copy()
            self._surface_follower = SurfaceFollower(**self._surface_parameters)
            self._surface_follower.reset(state.time)
            self._surface_stop_pending = False
            self._stop_tcp_control_ball()
            self._set_control_owner(_ControlOwner.SURFACE_FOLLOW)
            self._publish_surface_status("APPROACH")
        response.success = True
        response.message = "surface following started"
        return response

    def _stop_surface_follow(self, detail: str) -> bool:
        if self._surface_follower is None:
            return False
        self._surface_follower = None
        self._surface_stop_pending = True
        self._publish_tcp_velocity(np.zeros(3))
        self._publish_surface_status("IDLE", detail)
        return True

    def _stop_surface_follow_service(self, _request, response):
        with self._lock:
            stopped = self._stop_surface_follow("stopped by service")
        response.success = stopped
        response.message = (
            "surface following stopped" if stopped else "surface following is not active"
        )
        return response

    def _track_tcp_command(self, command: TwistStamped) -> None:
        self._last_tcp_command = np.array([
            command.twist.linear.x,
            command.twist.linear.y,
            command.twist.linear.z,
        ])
        self._last_tcp_command_time = time.monotonic()

    def _surface_contact_measurement(self):
        tool = self._runtime.config.contact_monitor["tool_body"]
        target = self._runtime.contact_target_name
        total_force = 0.0
        weighted_direction = np.zeros(3)
        for contact in self._latest_contacts:
            if not contact.massage_pair:
                continue
            direction = np.asarray(contact.normal, dtype=float)
            if contact.body_b == tool and contact.body_a == target:
                direction = -direction
            total_force += contact.normal_force
            weighted_direction += contact.normal_force * direction
        direction = None
        if np.linalg.norm(weighted_direction) > 1e-12:
            direction = weighted_direction / np.linalg.norm(weighted_direction)
        return total_force, direction

    def _advance_surface_follow(self, state) -> None:
        if self._surface_follower is None:
            return
        if any(
            not contact.massage_pair and contact.normal_force > 0.0
            for contact in self._latest_contacts
        ):
            self._publish_surface_status(
                "FAULT", "unexpected collision detected"
            )
            self._stop_surface_follow("unexpected collision detected")
            return
        force, direction = self._surface_contact_measurement()
        command = self._surface_follower.update(state.time, force, direction)
        self._publish_tcp_velocity(command.velocity)
        self._publish_surface_status(
            command.state,
            f"force={force:.3f} N; speed={np.linalg.norm(command.velocity):.4f} m/s",
        )
        if command.finished:
            detail = command.fault or "surface path completed"
            self._stop_surface_follow(detail)

    def _advance_teaching_replay(self, state):
        replay = self._teach_replay
        if replay is None:
            return None
        elapsed = state.time - replay.start_time
        next_time = elapsed + (
            self._runtime.model.opt.timestep * self._steps_per_update
        )
        desired = sample_trajectory(
            replay.start_positions, replay.points, next_time
        )
        self._runtime.set_target(desired)
        self._expected_joint_target = desired.copy()
        target = np.asarray(replay.points[-1].positions)
        final_time = replay.points[-1].time_from_start
        if elapsed >= final_time and np.max(np.abs(target - state.position)) <= (
            self._goal_tolerance
        ):
            self._stop_teaching_replay("trajectory completed", viewer_locked=True)
        elif elapsed > final_time + self._goal_time_tolerance:
            self._stop_teaching_replay(
                "goal tolerance was not reached", viewer_locked=True
            )
        return desired

    def _goal_callback(self, goal_request):
        try:
            points = prepare_trajectory(goal_request.trajectory)
            for point in points:
                self._runtime.validate_positions(point.positions)
        except ValueError as error:
            self.get_logger().warning(f"Rejecting trajectory: {error}")
            return GoalResponse.REJECT
        with self._lock:
            if self._runtime.control_mode is not ControlMode.POSITION:
                self.get_logger().warning(
                    "Rejecting trajectory: controller is in %s mode"
                    % self._runtime.control_mode.value
                )
                return GoalResponse.REJECT
            if (
                self._active is not None
                or self._goal_reserved
                or self._reserved_goal_interrupted
                or self._teach_replay is not None
                or self._surface_follower is not None
            ):
                self.get_logger().warning("Rejecting trajectory: controller is busy")
                return GoalResponse.REJECT
            if self._servo_owner_is_active():
                self.get_logger().warning(
                    "Rejecting trajectory: MoveIt Servo owns the controller"
                )
                return GoalResponse.REJECT
            if self._control_owner in (
                _ControlOwner.NATIVE_JOINT,
                _ControlOwner.NATIVE_TCP,
            ):
                self.get_logger().warning(
                    "Rejecting trajectory: MuJoCo Viewer owns the controller"
                )
                return GoalResponse.REJECT
            self._goal_reserved = True
            self._reserved_goal_error = ""
            self._set_control_owner(_ControlOwner.ACTION)
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _set_control_mode(self, mode, _request, response):
        with self._lock:
            self._stop_teaching_replay(
                "interrupted by control mode switch"
            )
            self._stop_surface_follow(
                "interrupted by control mode switch"
            )
            self._surface_stop_pending = False
            with self._viewer_lock():
                self._interrupt_trajectory(
                    "trajectory interrupted by control mode switch to %s"
                    % mode.value
                )
                self._last_servo_command_time = None
                self._set_control_owner(_ControlOwner.IDLE)
                self._runtime.set_control_mode(mode)
                self._expected_joint_target = self._runtime.target_positions()
                self._expected_joint_torque = self._runtime.torque_controls()
                self._mouse_joint_index = None
                self._mouse_joint_body_id = None
                self._stop_tcp_control_ball()
            self._publish_control_mode()
        response.success = True
        response.message = "control mode set to %s" % mode.value
        self.get_logger().info(response.message)
        return response

    def _publish_control_mode(self) -> None:
        self._control_mode_publisher.publish(
            String(data=self._runtime.control_mode.value)
        )

    def _publish_control_owner(self) -> None:
        self._control_owner_publisher.publish(
            String(data=self._control_owner.value)
        )

    def _set_control_owner(self, owner: _ControlOwner) -> None:
        if self._control_owner is owner:
            return
        self._control_owner = owner
        self._publish_control_owner()

    def _servo_owner_is_active(self) -> bool:
        if self._last_servo_command_time is None:
            return False
        if (
            time.monotonic() - self._last_servo_command_time
            <= self._servo_command_timeout
        ):
            return True
        self._last_servo_command_time = None
        with self._viewer_lock():
            self._runtime.stop_servo()
            self._expected_joint_target = self._runtime.target_positions()
        if self._control_owner in (
            _ControlOwner.SERVO,
            _ControlOwner.SURFACE_FOLLOW,
        ):
            self._set_control_owner(_ControlOwner.IDLE)
        return False

    def _servo_command_callback(self, trajectory) -> None:
        try:
            points = prepare_trajectory(trajectory)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring Servo command: {error}")
            return
        with self._lock:
            if self._runtime.control_mode is not ControlMode.POSITION:
                return
            if time.monotonic() < self._ignore_servo_commands_until:
                return
            if (
                self._active is not None
                or self._goal_reserved
                or self._reserved_goal_interrupted
                or self._teach_replay is not None
            ):
                return
            if self._control_owner in (
                _ControlOwner.NATIVE_JOINT,
                _ControlOwner.TEACH_REPLAY,
            ):
                return
            try:
                target = self._runtime.validate_positions(points[-1].positions)
            except ValueError as error:
                self.get_logger().warning(f"Ignoring Servo command: {error}")
                return
            with self._viewer_lock():
                try:
                    point = trajectory.points[-1]
                    if point.velocities:
                        if len(point.velocities) != len(JOINT_NAMES):
                            raise ValueError("Servo command must contain six velocities")
                        velocities = [point.velocities[trajectory.joint_names.index(name)]
                                      for name in JOINT_NAMES]
                        self._runtime.set_servo_target(
                            target, velocities, self._servo_command_timeout,
                        )
                    else:
                        self._runtime.set_target(target)
                except ValueError as error:
                    self.get_logger().warning(f"Ignoring Servo command: {error}")
                    return
                self._expected_joint_target = self._runtime.target_positions()
            self._last_servo_command_time = time.monotonic()
            if self._surface_follower is not None or self._surface_stop_pending:
                self._surface_stop_pending = False
                self._set_control_owner(_ControlOwner.SURFACE_FOLLOW)
            elif self._tcp_ball_active:
                self._set_control_owner(_ControlOwner.NATIVE_TCP)
            else:
                self._set_control_owner(_ControlOwner.SERVO)

    def _execute_callback(self, goal_handle):
        points = prepare_trajectory(goal_handle.request.trajectory)
        with self._lock:
            if (
                self._reserved_goal_interrupted
                or self._runtime.control_mode is not ControlMode.POSITION
            ):
                self._reserved_goal_interrupted = False
                self._goal_reserved = False
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = self._reserved_goal_error or (
                    "trajectory interrupted before execution"
                )
                self._reserved_goal_error = ""
                if self._control_owner is _ControlOwner.ACTION:
                    self._set_control_owner(_ControlOwner.IDLE)
                goal_handle.abort()
                return result
            with self._viewer_lock():
                start_state = self._runtime.state()
            active = _ActiveTrajectory(
                goal_handle=goal_handle,
                points=points,
                start_positions=start_state.position,
                start_time=start_state.time,
                done=threading.Event(),
            )
            self._active = active
            self._goal_reserved = False

        while not active.done.wait(0.02):
            if not goal_handle.is_cancel_requested:
                continue
            with self._lock:
                if self._active is active:
                    with self._viewer_lock():
                        target = self._runtime.state().position
                        self._runtime.set_target(target)
                        self._expected_joint_target = target.copy()
            active.error_string = "trajectory canceled"
            active.done.set()

        result = FollowJointTrajectory.Result()
        result.error_code = active.error_code
        result.error_string = active.error_string
        with self._lock:
            if self._active is active:
                self._active = None
            self._goal_reserved = False
            if self._control_owner is _ControlOwner.ACTION:
                self._set_control_owner(_ControlOwner.IDLE)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif active.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _detect_native_joint_control(self) -> None:
        target = self._runtime.target_positions()
        changed = ~np.isclose(
            target,
            self._expected_joint_target,
            rtol=0.0,
            atol=1e-9,
        )
        if not np.any(changed):
            return
        if self._runtime.control_mode is not ControlMode.POSITION:
            self._runtime.set_control_mode(self._runtime.control_mode)
            self._expected_joint_target = self._runtime.target_positions()
            self._expected_joint_torque = self._runtime.torque_controls()
            return
        self._stop_teaching_replay(
            "interrupted by MuJoCo joint control", viewer_locked=True
        )
        self._stop_surface_follow("interrupted by MuJoCo joint control")
        self._surface_stop_pending = False
        self._interrupt_trajectory(
            "trajectory interrupted by MuJoCo joint control"
        )
        self._last_servo_command_time = None
        self._runtime.accept_native_position_targets(target, changed)
        self._stop_tcp_control_ball()
        self._expected_joint_target = self._runtime.target_positions()
        self._expected_joint_torque = self._runtime.torque_controls()
        self._set_control_owner(_ControlOwner.NATIVE_JOINT)

    def _detect_native_torque_control(self) -> None:
        torques = self._runtime.torque_controls()
        changed = ~np.isclose(
            torques,
            self._expected_joint_torque,
            rtol=0.0,
            atol=1e-9,
        )
        if not np.any(changed):
            return
        if self._runtime.control_mode is not ControlMode.POSITION:
            self._runtime.set_control_mode(self._runtime.control_mode)
            self._expected_joint_target = self._runtime.target_positions()
            self._expected_joint_torque = self._runtime.torque_controls()
            return
        self._stop_teaching_replay(
            "interrupted by MuJoCo joint torque control", viewer_locked=True
        )
        self._stop_surface_follow("interrupted by MuJoCo joint torque control")
        self._surface_stop_pending = False
        self._interrupt_trajectory(
            "trajectory interrupted by MuJoCo joint torque control"
        )
        self._last_servo_command_time = None
        self._runtime.accept_native_torque_controls(torques, changed)
        self._stop_tcp_control_ball()
        self._expected_joint_target = self._runtime.target_positions()
        self._expected_joint_torque = self._runtime.torque_controls()
        self._set_control_owner(_ControlOwner.NATIVE_JOINT)

    def _detect_mouse_joint_control(self) -> None:
        if self._viewer is None:
            if self._mouse_joint_index is not None:
                self._runtime.end_mouse_joint_control()
                self._mouse_joint_index = None
                self._mouse_joint_body_id = None
            return
        perturb = self._viewer.perturb
        selected_body = int(perturb.select)
        selected_joint = self._runtime.joint_index_for_body(selected_body)
        perturbing = _joint_mouse_perturbation_active(perturb.active)
        active = (
            perturbing
            and selected_joint is not None
            and selected_body != self._runtime.tcp_control_body_id
            and self._runtime.control_mode is ControlMode.POSITION
        )
        if active:
            if self._mouse_joint_index != selected_joint:
                if self._mouse_joint_body_id is not None:
                    self._runtime.data.xfrc_applied[
                        self._mouse_joint_body_id
                    ] = 0.0
                self._runtime.end_mouse_joint_control()
                self._stop_teaching_replay(
                    "interrupted by MuJoCo mouse joint control",
                    viewer_locked=True,
                )
                self._stop_surface_follow(
                    "interrupted by MuJoCo mouse joint control"
                )
                self._surface_stop_pending = False
                self._interrupt_trajectory(
                    "trajectory interrupted by MuJoCo mouse joint control"
                )
                self._last_servo_command_time = None
                self._stop_tcp_control_ball()
                self._runtime.begin_mouse_joint_control(selected_joint)
                self._mouse_joint_index = selected_joint
                self._mouse_joint_body_id = selected_body
                self._set_control_owner(_ControlOwner.NATIVE_JOINT)
            self._runtime.update_mouse_joint_torque(
                selected_body,
                self._joint_mouse_control_maximum_torque,
            )
            self._expected_joint_target = self._runtime.target_positions()
            self._expected_joint_torque = self._runtime.torque_controls()
        elif self._mouse_joint_index is not None:
            if self._mouse_joint_body_id is not None:
                self._runtime.data.xfrc_applied[
                    self._mouse_joint_body_id
                ] = 0.0
            self._runtime.end_mouse_joint_control()
            self._mouse_joint_index = None
            self._mouse_joint_body_id = None
            self._expected_joint_target = self._runtime.target_positions()
            self._expected_joint_torque = self._runtime.torque_controls()
            if not np.any(self._runtime.torque_controlled):
                self._set_control_owner(_ControlOwner.IDLE)

    def _detect_tcp_control_ball(self) -> None:
        if not self._tcp_ball_enabled:
            return
        target = self._runtime.tcp_control_target()
        moved = not np.allclose(
            target,
            self._tcp_ball_last_target,
            rtol=0.0,
            atol=1e-6,
        )
        if not moved:
            return
        self._tcp_ball_last_target = target.copy()
        if self._runtime.control_mode is not ControlMode.POSITION:
            self._stop_tcp_control_ball()
            return
        if self._tcp_ball_active:
            return
        self._stop_teaching_replay(
            "interrupted by MuJoCo TCP control", viewer_locked=True
        )
        self._stop_surface_follow("interrupted by MuJoCo TCP control")
        self._surface_stop_pending = False
        self._interrupt_trajectory(
            "trajectory interrupted by MuJoCo TCP control"
        )
        self._last_servo_command_time = None
        hold_target = self._runtime.state().position
        self._runtime.set_target(hold_target)
        self._expected_joint_target = hold_target.copy()
        self._expected_joint_torque = self._runtime.torque_controls()
        self._tcp_ball_active = True
        self._set_control_owner(_ControlOwner.NATIVE_TCP)

    def _publish_tcp_velocity(
        self,
        velocity: np.ndarray,
        stamp_seconds: Optional[float] = None,
    ) -> None:
        command = TwistStamped()
        if stamp_seconds is None:
            stamp_seconds = (
                self._clock_offset + float(self._runtime.data.time)
            )
        command.header.stamp = _time_message(stamp_seconds)
        command.header.frame_id = "world"
        command.twist.linear.x = float(velocity[0])
        command.twist.linear.y = float(velocity[1])
        command.twist.linear.z = float(velocity[2])
        self._cartesian_jog_publisher.publish(command)

    def _advance_tcp_control_ball(self) -> None:
        if not self._tcp_ball_active:
            return
        current = self._runtime.state().tool_position
        target = self._runtime.tcp_control_target()
        error = target - current
        distance = float(np.linalg.norm(error))
        if distance > self._tcp_ball_maximum_distance:
            target = current + error * (
                self._tcp_ball_maximum_distance / distance
            )
            self._runtime.set_tcp_control_target(target)
            self._tcp_ball_last_target = target.copy()
        velocity, reached = limited_linear_velocity(
            current,
            target,
            self._tcp_ball_gain,
            self._tcp_ball_maximum_speed,
            self._tcp_ball_tolerance,
        )
        self._publish_tcp_velocity(velocity)
        if reached:
            self._stop_tcp_control_ball()
            if self._control_owner is _ControlOwner.NATIVE_TCP:
                self._set_control_owner(_ControlOwner.IDLE)

    def _advance(self) -> None:
        with self._lock:
            self._servo_owner_is_active()
            restarted = False
            with self._viewer_lock():
                active = self._active
                if active is not None and active.done.is_set():
                    active = None
                if self._runtime.recover_native_reset():
                    restarted = True
                    self._contact_monitor.reset()
                    self._tcp_diagnostics.reset()
                    self._tcp_actual_trace.clear()
                    self._tcp_target_trace.clear()
                    self.get_logger().info(
                        "MuJoCo restarted at the configured initial pose"
                    )
                    if active is not None:
                        active.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        active.error_string = (
                            "trajectory interrupted by MuJoCo restart"
                        )
                        active.done.set()
                        active = None
                    if self._goal_reserved:
                        self._reserved_goal_interrupted = True
                        self._reserved_goal_error = (
                            "trajectory interrupted by MuJoCo restart"
                        )
                        self._goal_reserved = False
                    self._stop_teaching_replay(
                        "interrupted by MuJoCo restart", viewer_locked=True
                    )
                    self._stop_surface_follow("interrupted by MuJoCo restart")
                    self._surface_stop_pending = False
                    self._last_servo_command_time = None
                    self._ignore_servo_commands_until = (
                        time.monotonic() + self._servo_command_timeout
                    )
                    self._set_control_owner(_ControlOwner.IDLE)
                    self._expected_joint_target = self._runtime.target_positions()
                    self._expected_joint_torque = self._runtime.torque_controls()
                    self._mouse_joint_index = None
                    self._mouse_joint_body_id = None
                    self._tcp_ball_active = False
                    self._tcp_ball_last_target = (
                        self._runtime.tcp_control_target()
                    )
                    if self._tcp_ball_enabled and self._viewer is not None:
                        self._select_tcp_control_ball()
                    self._publish_control_mode()
                self._detect_native_torque_control()
                self._detect_mouse_joint_control()
                self._detect_native_joint_control()
                self._detect_tcp_control_ball()
                self._advance_tcp_control_ball()
                pre_step_state = self._runtime.state()
                self._advance_surface_follow(pre_step_state)
                self._advance_teaching_replay(pre_step_state)
                if active is not None and active.done.is_set():
                    active = None
                if active is not None:
                    next_time = (
                        self._runtime.state().time
                        - active.start_time
                        + self._runtime.model.opt.timestep
                        * self._steps_per_update
                    )
                    desired = sample_trajectory(
                        active.start_positions, active.points, next_time
                    )
                    self._runtime.set_target(desired)
                    self._expected_joint_target = desired.copy()
                state = self._runtime.step(self._steps_per_update)
                self._expected_joint_target = state.target_position.copy()
                self._expected_joint_torque = state.commanded_torque.copy()
                contacts = self._runtime.contacts()
                self._latest_contacts = contacts
                stamp_seconds = self._update_clock_time(state.time)
                if (
                    self._control_owner is _ControlOwner.NATIVE_JOINT
                    and not np.any(state.torque_controlled)
                    and self._mouse_joint_index is None
                    and np.max(np.abs(
                        state.position - self._expected_joint_target
                    )) <= self._goal_tolerance
                ):
                    self._set_control_owner(_ControlOwner.IDLE)
                if not self._tcp_ball_active:
                    self._sync_tcp_control_ball()
                if restarted:
                    self._publish_tcp_velocity(
                        np.zeros(3),
                        stamp_seconds=stamp_seconds,
                    )
            self._sync_viewer()
            try:
                self._publish_state(
                    state,
                    contacts,
                    stamp_seconds=stamp_seconds,
                )
            except RuntimeError:
                if rclpy.ok():
                    raise
                return
            if active is not None:
                self._update_active_trajectory(active, state, desired)

    def _update_active_trajectory(self, active, state, desired) -> None:
        elapsed = state.time - active.start_time
        target = np.asarray(active.points[-1].positions)
        error = target - state.position

        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = _time_message(
            self._clock_offset + state.time
        )
        feedback.joint_names = list(JOINT_NAMES)
        feedback.desired = JointTrajectoryPoint(positions=desired.tolist())
        feedback.actual = JointTrajectoryPoint(positions=state.position.tolist())
        feedback.error = JointTrajectoryPoint(positions=(desired - state.position).tolist())
        active.goal_handle.publish_feedback(feedback)

        final_time = active.points[-1].time_from_start
        if elapsed >= final_time and np.max(np.abs(error)) <= self._goal_tolerance:
            active.error_string = "trajectory reached within goal tolerance"
            active.done.set()
        elif elapsed > final_time + self._goal_time_tolerance:
            active.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            active.error_string = (
                "trajectory did not reach %.6f rad tolerance; maximum error %.6f rad"
                % (self._goal_tolerance, np.max(np.abs(error)))
            )
            active.done.set()

    @staticmethod
    def _tcp_pose_message(position, rotation, stamp) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        quaternion = np.zeros(4)
        mujoco.mju_mat2Quat(quaternion, np.asarray(rotation).reshape(9))
        message.pose.orientation.w = float(quaternion[0])
        message.pose.orientation.x = float(quaternion[1])
        message.pose.orientation.y = float(quaternion[2])
        message.pose.orientation.z = float(quaternion[3])
        return message

    @staticmethod
    def _append_trace(trace, position, limit) -> None:
        point = np.asarray(position, dtype=float).copy()
        if not trace or np.linalg.norm(point - trace[-1]) >= 0.0005:
            trace.append(point)
            if len(trace) > limit:
                del trace[:-limit]

    @staticmethod
    def _trace_marker(points, stamp, marker_id, name, color) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "world"
        marker.ns = "massage_mujoco_tcp"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.004
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = name
        marker.points = [
            Point(x=float(item[0]), y=float(item[1]), z=float(item[2]))
            for item in points
        ]
        return marker

    def _publish_tcp_diagnostics(self, state, stamp) -> None:
        if state.time < self._last_tcp_diagnostic_time:
            self._last_tcp_diagnostic_time = -math.inf
        if state.time - self._last_tcp_diagnostic_time < self._tcp_diagnostic_period:
            return
        self._last_tcp_diagnostic_time = state.time
        target_position, target_rotation = self._runtime.tool_pose_for_positions(
            state.target_position
        )
        sample = self._tcp_diagnostics.update(
            state.time, state.tool_position, target_position
        )
        actual_pose = self._tcp_pose_message(
            state.tool_position, state.tool_rotation, stamp
        )
        target_pose = self._tcp_pose_message(
            target_position, target_rotation, stamp
        )
        self._tcp_actual_pose_publisher.publish(actual_pose)
        self._tcp_target_pose_publisher.publish(target_pose)

        velocity = TwistStamped()
        velocity.header = actual_pose.header
        velocity.twist.linear.x = float(sample.actual_velocity[0])
        velocity.twist.linear.y = float(sample.actual_velocity[1])
        velocity.twist.linear.z = float(sample.actual_velocity[2])
        self._tcp_actual_velocity_publisher.publish(velocity)

        command_speed = 0.0
        if (
            self._last_tcp_command_time is not None
            and time.monotonic() - self._last_tcp_command_time
            <= self._servo_command_timeout
        ):
            command_speed = float(np.linalg.norm(self._last_tcp_command))
        ratio = sample.actual_speed / command_speed if command_speed > 1e-6 else 0.0
        status = DiagnosticStatus()
        status.name = "MuJoCo TCP tracking"
        status.hardware_id = "simulation"
        status.level = (
            DiagnosticStatus.WARN
            if command_speed > 0.01 and ratio < 0.25
            else DiagnosticStatus.OK
        )
        status.message = (
            "actual TCP speed is below 25 percent of command"
            if status.level == DiagnosticStatus.WARN
            else "TCP tracking telemetry is valid"
        )
        status.values = [
            KeyValue(key="control_owner", value=self._control_owner.value),
            KeyValue(key="command_speed_mps", value=f"{command_speed:.9f}"),
            KeyValue(key="actual_speed_mps", value=f"{sample.actual_speed:.9f}"),
            KeyValue(key="position_error_m", value=f"{sample.target_error:.9f}"),
            KeyValue(key="speed_ratio", value=f"{ratio:.6f}"),
        ]
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = stamp
        diagnostic.status = [status]
        self._tcp_diagnostic_publisher.publish(diagnostic)

        self._append_trace(
            self._tcp_actual_trace, state.tool_position, self._tcp_trace_length
        )
        self._append_trace(
            self._tcp_target_trace, target_position, self._tcp_trace_length
        )
        traces = MarkerArray()
        traces.markers = [
            self._trace_marker(
                self._tcp_actual_trace,
                stamp,
                0,
                "actual TCP path",
                (0.1, 0.9, 0.2, 0.95),
            ),
            self._trace_marker(
                self._tcp_target_trace,
                stamp,
                1,
                "target TCP path",
                (1.0, 0.2, 0.1, 0.95),
            ),
        ]
        self._tcp_trace_publisher.publish(traces)

    def _publish_state(
        self,
        state=None,
        contacts=None,
        stamp_seconds=None,
    ) -> None:
        if state is None:
            state = self._runtime.state()
        if stamp_seconds is None:
            stamp_seconds = self._update_clock_time(state.time)
        stamp = _time_message(stamp_seconds)
        self._clock_publisher.publish(Clock(clock=stamp))
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = state.position.tolist()
        joint_state.velocity = state.velocity.tolist()
        joint_state.effort = state.actuator_effort.tolist()
        self._joint_state_publisher.publish(joint_state)
        reference = JointTrajectoryPoint(
            positions=state.target_position.tolist(),
            velocities=state.target_velocity.tolist(),
            effort=state.commanded_torque.tolist(),
        )
        feedback = JointTrajectoryPoint(
            positions=state.position.tolist(),
            velocities=state.velocity.tolist(),
            effort=state.actuator_effort.tolist(),
        )
        controller_state = JointTrajectoryControllerState()
        controller_state.header.stamp = stamp
        controller_state.joint_names = list(JOINT_NAMES)
        controller_state.reference = reference
        controller_state.feedback = feedback
        controller_state.error = JointTrajectoryPoint(
            positions=(state.target_position - state.position).tolist(),
        )
        controller_state.output = JointTrajectoryPoint(
            effort=state.actuator_effort.tolist(),
        )
        # Humble keeps these fields for consumers using the older message API.
        controller_state.desired = reference
        controller_state.actual = feedback
        self._controller_state_publisher.publish(controller_state)
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = self._wrench_frame_id
        wrench.wrench.force.x = float(state.tool_wrench[0])
        wrench.wrench.force.y = float(state.tool_wrench[1])
        wrench.wrench.force.z = float(state.tool_wrench[2])
        wrench.wrench.torque.x = float(state.tool_wrench[3])
        wrench.wrench.torque.y = float(state.tool_wrench[4])
        wrench.wrench.torque.z = float(state.tool_wrench[5])
        self._wrench_publisher.publish(wrench)
        if contacts is None:
            contacts = self._runtime.contacts()
        snapshot = self._contact_monitor.update(state.time, contacts)
        contact = ContactState()
        contact.header.stamp = stamp
        contact.header.frame_id = "world"
        for field in ("in_contact", "over_force", "impact", "unexpected_contact",
                      "normal_force", "force_rate", "events"):
            setattr(contact, field, snapshot[field])
        for item in snapshot['contacts']:
            point = ContactPoint()
            for field in ("body_a", "body_b", "geom_a", "geom_b", "penetration",
                          "normal_force", "massage_pair"):
                setattr(point, field, item[field])
            point.position.x, point.position.y, point.position.z = item['position']
            point.normal.x, point.normal.y, point.normal.z = item['normal']
            contact.contacts.append(point)
        self._contact_publisher.publish(contact)
        if contact.events:
            self._contact_event_publisher.publish(contact)
        self._publish_tcp_diagnostics(state, stamp)

    def destroy_node(self):
        with self._lock:
            self._close_viewer()
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the MuJoCo ROS adapter with callbacks on separate executor threads."""
    rclpy.init(args=args)
    node = MujocoNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
