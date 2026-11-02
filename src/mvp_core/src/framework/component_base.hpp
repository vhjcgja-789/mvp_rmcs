// 阶段 7 框架:组件契约基类(header-only,零额外依赖)。
// RMCS 对照:这是 rmcs_executor Input/OutputInterface 的"原生 ROS2 简化版"。
// 组件只声明"我读哪个关节的哪个接口、我写哪个关节的哪个接口",
// 话题名由框架按唯一约定推导({joint}/{interface}),组件代码里不出现话题字符串。
//
// 现状与后续:
// - 阶段 7:契约注册表主要作为"规格 vs 真实图"启动校验的参照与文档;
// - 后续:同进程校验(规格 vs 契约)、节拍驱动调度(阶段 11)都要用这份注册表。
#pragma once

#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mvp_core::framework {

// 一个接口引用 = (关节, 接口语义),如 ("yaw", "position")。
// "position/velocity/effort/command" 的完整语义在阶段 9 的标准消息协议里定义,
// 这里只把它当"名字对"传递。
struct InterfaceRef {
    std::string joint;
    std::string interface;
};

// 组件基类:所有硬件/控制器/模板组件的共同父类。
// 接线约定:接口 (joint, interface) 对应相对话题 "{joint}/{interface}",
// 实际全名由容器 namespace(机器人名)自动补全,例如
// (yaw, position) → /stage7_demo/yaw/position。launch 生成器按同一规则推导期望话题。
class ComponentBase : public rclcpp::Node {
public:
    ComponentBase(const std::string& node_name, const rclcpp::NodeOptions& options)
        : rclcpp::Node(node_name, options) {}

    // 创建发布端并登记契约:声明"本组件产出 (joint, interface)"。
    // 返回值与 rclcpp::Node::create_publisher 一致,只是话题名被框架接管。
    template <typename Msg>
    typename rclcpp::Publisher<Msg>::SharedPtr produce_publisher(
        const std::string& joint, const std::string& interface,
        const rclcpp::QoS& qos = rclcpp::QoS{1}) {
        produces_.push_back({joint, interface});
        return create_publisher<Msg>(topic_for(joint, interface), qos);
    }

    // 创建订阅端并登记契约:声明"本组件消费 (joint, interface)"。
    template <typename Msg, typename Callback>
    typename rclcpp::Subscription<Msg>::SharedPtr claim_subscription(
        const std::string& joint, const std::string& interface, Callback&& callback,
        const rclcpp::QoS& qos = rclcpp::QoS{1}) {
        claims_.push_back({joint, interface});
        return create_subscription<Msg>(
            topic_for(joint, interface), qos, std::forward<Callback>(callback));
    }

    // 契约只读访问(启动校验 / 文档 / 后续同进程调度用)。
    const std::vector<InterfaceRef>& claims() const { return claims_; }
    const std::vector<InterfaceRef>& produces() const { return produces_; }

private:
    // 接口 → 相对话题名的唯一规则:接线由此自动推导,人不再手写 remap。
    static std::string topic_for(const std::string& joint, const std::string& interface) {
        return joint + "/" + interface;
    }

    std::vector<InterfaceRef> claims_;
    std::vector<InterfaceRef> produces_;
};

} // namespace mvp_core::framework
