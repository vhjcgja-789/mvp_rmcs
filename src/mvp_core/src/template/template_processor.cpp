// 阶段 7 模板组件 2:数据处理器。演示"claim 一个接口 → produce 另一个接口"的链式接法。
// 不引入控制语义:收到输入立即转发(带一个纯数据变换),让"规格 → 自动接线 →
// 启动校验"的骨架跑通。后续的控制器(PID 等)就是在这个骨架上填算法。
#include <memory>

#include <framework/component_base.hpp>
#include <mvp_msgs/msg/float64_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mvp_core::template_ {

class TemplateProcessor : public framework::ComponentBase {
public:
    explicit TemplateProcessor(const rclcpp::NodeOptions& options)
        : framework::ComponentBase("template_processor", options) {
        publisher_ = produce_publisher<mvp_msgs::msg::Float64Stamped>("yaw", "command");
        subscription_ = claim_subscription<mvp_msgs::msg::Float64Stamped>(
            "yaw", "position",
            [this](mvp_msgs::msg::Float64Stamped::UniquePtr message) {
                const double forwarded = message->data * 2.0;  // 仅演示数据流,非控制
                message->data = forwarded;
                publisher_->publish(std::move(message));
                RCLCPP_INFO(get_logger(), "转发 %.3f", forwarded);
            });
    }

private:
    rclcpp::Publisher<mvp_msgs::msg::Float64Stamped>::SharedPtr publisher_;
    rclcpp::Subscription<mvp_msgs::msg::Float64Stamped>::SharedPtr subscription_;
};

} // namespace mvp_core::template_

RCLCPP_COMPONENTS_REGISTER_NODE(mvp_core::template_::TemplateProcessor)
