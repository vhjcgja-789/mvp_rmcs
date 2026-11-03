# 启动顺序:robot_state_publisher(发布 robot_description)→ ros2_control_node(加载各板硬件组件)
# → spawner 激活 joint_state_broadcaster
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("mvp_bringup")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ",
                 PathJoinSubstitution([share, "urdf", "sentry.urdf.xacro"])]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([share, "config", "sentry", "controllers.yaml"])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # 从 /robot_description 话题获取 URDF
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
    ])
