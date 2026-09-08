import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    description = get_package_share_directory("massage_description")
    robot_xacro = os.path.join(
        description, "urdf", "jaka_s5_massage.urdf.xacro"
    )
    semantic_file = os.path.join(
        description, "config", "jaka_s5_massage.srdf"
    )
    moveit_config = (
        MoveItConfigsBuilder("jaka_s5", package_name="jaka_s5_moveit_config")
        .robot_description(
            file_path=robot_xacro,
            mappings={
                "use_gazebo": "false",
                "use_rviz_sim": "false",
                "use_massage_head": "true",
            },
        )
        .robot_description_semantic(file_path=semantic_file)
        .joint_limits(file_path="config/joint_limits_real.yaml")
        .to_moveit_configs()
    )

    # This launch intentionally does not include real.launch.py. The task node
    # first evaluates authorization and backend capability without creating any
    # lifecycle client or starting the JAKA driver. Real infrastructure remains
    # an explicit, separately audited prerequisite until reference tracking in
    # admittance mode is implemented and verified.
    task = Node(
        package="massage_task",
        executable="massage_task_real_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # RobotModelLoader needs this node-local parameter to instantiate
            # the IK plugin. URDF/SRDF can fall back to published topics, but
            # kinematics.yaml cannot.
            moveit_config.robot_description_kinematics,
            {
                "use_sim_time": False,
                "execute": typed("execute", bool),
                "parameters_confirmed": typed("parameters_confirmed", bool),
                "execution_mode": LaunchConfiguration("execution_mode"),
                "real_ptp_backend": LaunchConfiguration("real_ptp_backend"),
                "technique": LaunchConfiguration("technique"),
                "planning_attempts": typed("planning_attempts", int),
                "maximum_ik_attempts": typed("maximum_ik_attempts", int),
                "maximum_unique_ik_candidates": typed(
                    "maximum_unique_ik_candidates", int
                ),
                "ik_base_timeout": typed("ik_base_timeout", float),
                "ik_timeout_per_meter": typed(
                    "ik_timeout_per_meter", float
                ),
                "ik_timeout_per_radian": typed(
                    "ik_timeout_per_radian", float
                ),
                "ik_minimum_timeout": typed("ik_minimum_timeout", float),
                "ik_maximum_timeout": typed("ik_maximum_timeout", float),
                "ik_failure_backoff_factor": typed(
                    "ik_failure_backoff_factor", float
                ),
                "contact_x": typed("contact_x", float),
                "contact_y": typed("contact_y", float),
                "contact_z": typed("contact_z", float),
                "push_length": typed("push_length", float),
                "push_repetitions": typed("push_repetitions", int),
                "press_cycles": typed("press_cycles", int),
                "knead_radius": typed("knead_radius", float),
                "knead_cycle_duration": typed("knead_cycle_duration", float),
                "knead_cycles": typed("knead_cycles", int),
                "knead_maximum_speed": typed("knead_maximum_speed", float),
                "free_space_velocity_scale": typed(
                    "free_space_velocity_scale", float
                ),
                "technique_velocity_scale": typed(
                    "technique_velocity_scale", float
                ),
                "technique_acceleration_scale": typed(
                    "technique_acceleration_scale", float
                ),
                "planning_timeout": typed("planning_timeout", float),
                "execution_timeout_margin": typed(
                    "execution_timeout_margin", float
                ),
                "robot_state_timeout": typed("robot_state_timeout", float),
                "native_cartesian_max_speed_mm_s": typed(
                    "native_cartesian_max_speed_mm_s", float
                ),
                "native_cartesian_max_acceleration_mm_s2": typed(
                    "native_cartesian_max_acceleration_mm_s2", float
                ),
                "native_cartesian_orientation_speed_rad_s": typed(
                    "native_cartesian_orientation_speed_rad_s", float
                ),
                "native_cartesian_orientation_acceleration_rad_s2": typed(
                    "native_cartesian_orientation_acceleration_rad_s2", float
                ),
                "native_cartesian_translation_tolerance_mm": typed(
                    "native_cartesian_translation_tolerance_mm", float
                ),
                "native_cartesian_rotation_tolerance_rad": typed(
                    "native_cartesian_rotation_tolerance_rad", float
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("parameters_confirmed", default_value="false"),
        DeclareLaunchArgument("execution_mode", default_value="plan_only"),
        DeclareLaunchArgument(
            "real_ptp_backend", default_value="native_joint_move"
        ),
        DeclareLaunchArgument("technique", default_value="push"),
        DeclareLaunchArgument("planning_attempts", default_value="5"),
        DeclareLaunchArgument("maximum_ik_attempts", default_value="32"),
        DeclareLaunchArgument(
            "maximum_unique_ik_candidates", default_value="8"
        ),
        DeclareLaunchArgument("ik_base_timeout", default_value="0.02"),
        DeclareLaunchArgument("ik_timeout_per_meter", default_value="0.25"),
        DeclareLaunchArgument("ik_timeout_per_radian", default_value="0.05"),
        DeclareLaunchArgument("ik_minimum_timeout", default_value="0.02"),
        DeclareLaunchArgument("ik_maximum_timeout", default_value="0.50"),
        DeclareLaunchArgument(
            "ik_failure_backoff_factor", default_value="1.35"
        ),
        DeclareLaunchArgument(
            "contact_x", default_value="-0.471238630147741"
        ),  # 硬编码的预接触点
        DeclareLaunchArgument("contact_y", default_value="0.156275068796867"),
        DeclareLaunchArgument("contact_z", default_value="0.281381721182422"),
        DeclareLaunchArgument("push_length", default_value="0.08"),
        DeclareLaunchArgument("push_repetitions", default_value="3"),
        DeclareLaunchArgument("press_cycles", default_value="3"),
        DeclareLaunchArgument("knead_radius", default_value="0.010"),
        DeclareLaunchArgument("knead_cycle_duration", default_value="8.0"),
        DeclareLaunchArgument("knead_cycles", default_value="3"),
        DeclareLaunchArgument("knead_maximum_speed", default_value="0.010"),
        DeclareLaunchArgument("free_space_velocity_scale", default_value="0.15"),
        DeclareLaunchArgument("technique_velocity_scale", default_value="0.10"),
        DeclareLaunchArgument("technique_acceleration_scale", default_value="0.10"),
        DeclareLaunchArgument("planning_timeout", default_value="60.0"),
        DeclareLaunchArgument("execution_timeout_margin", default_value="15.0"),
        DeclareLaunchArgument("robot_state_timeout", default_value="1.0"),
        DeclareLaunchArgument(
            "native_cartesian_max_speed_mm_s", default_value="100.0"
        ),
        DeclareLaunchArgument(
            "native_cartesian_max_acceleration_mm_s2", default_value="500.0"
        ),
        DeclareLaunchArgument(
            "native_cartesian_orientation_speed_rad_s", default_value="0.5"
        ),
        DeclareLaunchArgument(
            "native_cartesian_orientation_acceleration_rad_s2",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "native_cartesian_translation_tolerance_mm", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "native_cartesian_rotation_tolerance_rad", default_value="0.01"
        ),
        task,
    ])
