# launch 文件 —— 解决"节点多了、每个开一个终端手动跑"的问题。
# 它声明式地把一组节点一起拉起来,`ros2 launch mvp_basics hello.launch.py` 即可。
#
# 阶段3:launch 还负责"装配":给节点塞参数、给话题改线(remap)。
#   - 参数 parameters:改节点"怎么干活"(频率、前缀),值放在 config/*.yaml 里。
#   - 重映射 remappings:改节点"接到哪根线"(话题名),节点代码完全不知情。
#   - 启动参数 LaunchArgument:选用哪一份配置,如 `config:=fast`。
# 对照 RMCS:rmcs_bringup/launch/rmcs.launch.py 正是这个套路 ——
# 用 `robot:=sentry` 选中 config/sentry.yaml,整台机器人的参数都在那一个文件里。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 启动参数:选哪份配置文件(不带 .yaml),默认 hello。
    config_arg = DeclareLaunchArgument(
        "config", default_value="hello", description="config/ 下的参数文件名")

    # 拼出安装目录里的文件路径:share/mvp_basics/config/<config>.yaml。
    # 用的是安装目录而不是源码目录,所以改了 yaml 要重新 colcon build 才生效。
    config_file = PathJoinSubstitution([
        FindPackageShare("mvp_basics"), "config",
        [LaunchConfiguration("config"), ".yaml"],
    ])

    # 改线:把代码里的 /hello 接到 /chatter 上。
    # 发布方和订阅方必须一起改,否则两边各在各的话题上,listener 就收不到了。
    remap_hello = [("/hello", "/chatter")]

    return LaunchDescription([
        config_arg,
        Node(
            package="mvp_basics",
            executable="talker",
            # 列表里可以混放"yaml 文件"和"字典",后面的覆盖前面的。
            # 这里只给文件;临时想改某一项,在后面追加 {"prefix": "..."} 即可。
            parameters=[config_file],
            remappings=remap_hello,
        ),
        Node(
            package="mvp_basics",
            executable="listener",
            remappings=remap_hello,
        ),
    ])
