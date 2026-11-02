# RMCS 对照：组件清单对应 sentry.yaml 的 Plugin -> name，原生容器负责装载。
# 接线交给 namespace/remap，参数文件只描述节点的运行参数。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


COMPONENTS = [
    ("mvp_core::hardware::MockMotor", "motor", []),
    (
        "mvp_core::controller::pid::PidController",
        "angle_pid",
        [("measurement", "angle"), ("setpoint", "target_angle"), ("control", "target_velocity")],
    ),
    (
        "mvp_core::controller::pid::PidController",
        "velocity_pid",
        [("measurement", "velocity"), ("setpoint", "target_velocity"), ("control", "control_torque")],
    ),
]


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("mvp_bringup"), "config", [LaunchConfiguration("config"), ".yaml"]]
    )
    container = ComposableNodeContainer(
        name="single_joint_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="mvp_core",
                plugin=plugin,
                name=name,
                namespace="yaw",
                parameters=[config_file],
                remappings=remappings,
                extra_arguments=[{"use_intra_process_comms": True}],
            )
            for plugin, name, remappings in COMPONENTS
        ],
    )
    return LaunchDescription(
        [DeclareLaunchArgument("config", default_value="single_joint"), container]
    )
