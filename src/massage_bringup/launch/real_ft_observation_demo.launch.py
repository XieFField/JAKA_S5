from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _parameter(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    node = Node(
        package="massage_jaka",
        executable="real_ft_observation_demo",
        output="screen",
        parameters=[
            {
                "wrench_topic": LaunchConfiguration("wrench_topic"),
                "expected_frame": LaunchConfiguration("expected_frame"),
                "csv_path": LaunchConfiguration("csv_path"),
                "required_response": LaunchConfiguration("required_response"),
                "start_timeout": _parameter("start_timeout", float),
                "baseline_duration": _parameter("baseline_duration", float),
                "load_duration": _parameter("load_duration", float),
                "recovery_duration": _parameter("recovery_duration", float),
                "minimum_sample_rate": _parameter("minimum_sample_rate", float),
                "maximum_sample_gap": _parameter("maximum_sample_gap", float),
                "minimum_force_delta": _parameter("minimum_force_delta", float),
                "minimum_torque_delta": _parameter("minimum_torque_delta", float),
                "maximum_recovery_force_offset": _parameter(
                    "maximum_recovery_force_offset", float
                ),
                "maximum_recovery_torque_offset": _parameter(
                    "maximum_recovery_torque_offset", float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wrench_topic",
                default_value="/jaka_driver/wrench",
                description="待观测的 WrenchStamped 话题",
            ),
            DeclareLaunchArgument(
                "expected_frame",
                default_value="Link_06",
                description="每条 FT 消息必须使用的坐标系，空字符串表示不限定",
            ),
            DeclareLaunchArgument(
                "csv_path",
                default_value="",
                description="CSV 输出路径；留空时在 /tmp 下自动生成",
            ),
            DeclareLaunchArgument(
                "required_response",
                default_value="none",
                description="响应门禁：none、force、torque、either 或 both",
            ),
            DeclareLaunchArgument(
                "start_timeout",
                default_value="5.0",
                description="等待第一条有效 FT 消息的超时，单位秒",
            ),
            DeclareLaunchArgument(
                "baseline_duration",
                default_value="5.0",
                description="无外载基线采集时长，单位秒",
            ),
            DeclareLaunchArgument(
                "load_duration",
                default_value="10.0",
                description="人工加载采集时长，单位秒",
            ),
            DeclareLaunchArgument(
                "recovery_duration",
                default_value="5.0",
                description="卸载恢复采集时长，单位秒",
            ),
            DeclareLaunchArgument(
                "minimum_sample_rate",
                default_value="5.0",
                description="每个阶段允许的最低采样率，0 表示关闭门禁",
            ),
            DeclareLaunchArgument(
                "maximum_sample_gap",
                default_value="0.5",
                description="每个阶段允许的最大样本间隔，0 表示关闭门禁",
            ),
            DeclareLaunchArgument(
                "minimum_force_delta",
                default_value="0.5",
                description="相对基线的最小力变化，单位 N",
            ),
            DeclareLaunchArgument(
                "minimum_torque_delta",
                default_value="0.05",
                description="相对基线的最小力矩变化，单位 N*m",
            ),
            DeclareLaunchArgument(
                "maximum_recovery_force_offset",
                default_value="0.0",
                description="卸载后允许的最大力均值偏差，0 表示关闭门禁",
            ),
            DeclareLaunchArgument(
                "maximum_recovery_torque_offset",
                default_value="0.0",
                description="卸载后允许的最大力矩均值偏差，0 表示关闭门禁",
            ),
            node,
        ]
    )
