# launch 文件 —— 解决"节点多了、每个开一个终端手动跑"的问题。
# 它声明式地把一组节点一起拉起来,`ros2 launch mvp_basics hello.launch.py` 即可。
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 每个 Node 声明:用哪个包、跑哪个可执行文件(节点名默认取代码里的)
        Node(package="mvp_basics", executable="talker"),
        Node(package="mvp_basics", executable="listener"),
    ])
