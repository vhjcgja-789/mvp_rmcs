# 阶段 7 launch:唯一解析规格的地方。
# 读 config/stage7_demo.yaml → 生成组件容器(自动推导接线)+ 为每个组件实例生成
# 一个 wiring_checker,核对"规格期望的图"与"真实 ROS 图";任一校验失败 → 整体 shutdown。
#
# 与阶段 6 的差别:组件清单/接线不再写死在 launch 里,而是规格数据;
# 接线规则只有一条(接口 {joint}/{interface} → /{robot}/{joint}/{interface}),
# 由组件代码(ComponentBase)与这里共用,人不再手写 remap。
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _fail_fast(checker: ExecuteProcess) -> RegisterEventHandler:
    # 校验器退出码非 0 → 整体 shutdown(fail-fast);0(通过)则什么都不做。
    def _on_exit(event, context):
        if event.returncode != 0:
            return Shutdown(reason="wiring check failed")

    return RegisterEventHandler(OnProcessExit(target_action=checker, on_exit=_on_exit))


def generate_launch_description():
    # 本阶段单规格;多规格化(robot:=xxx 选择)留到阶段 12。
    spec_name = "stage7_demo"
    share_dir = get_package_share_directory("mvp_bringup")
    spec_path = os.path.join(share_dir, "config", spec_name + ".yaml")
    with open(spec_path, encoding="utf-8") as f:
        spec = yaml.safe_load(f)

    robot = spec["robot"]

    nodes = []
    checkers = []
    for component in spec["components"]:
        name = component["name"]
        # 接线由规格推导,组件代码按同一约定创建收发(相对话题 {joint}/{interface})。
        expected_subs = [
            f"/{robot}/{c['joint']}/{c['interface']}" for c in component.get("claims", [])
        ]
        expected_pubs = [
            f"/{robot}/{c['joint']}/{c['interface']}" for c in component.get("produces", [])
        ]
        nodes.append(
            ComposableNode(
                package="mvp_core",
                plugin=component["type"],
                name=name,
                namespace=robot,
                parameters=[spec_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # 每个实例一个校验器,核对"期望话题 ↔ 该节点的真实订阅/发布"。
        checker = ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "mvp_core",
                "wiring_checker",
                "--ros-args",
                "-r",
                f"__node:=wiring_checker_{name}",
                "-p",
                f"node_name:={name}",
                "-p",
                f"node_namespace:={robot}",
                "-p",
                f"expected_subs:={expected_subs}",
                "-p",
                f"expected_pubs:={expected_pubs}",
            ],
            output="screen",
        )
        checkers.append(checker)

    container = ComposableNodeContainer(
        name=f"{robot}_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=nodes,
    )

    return LaunchDescription([container] + checkers + [_fail_fast(c) for c in checkers])
