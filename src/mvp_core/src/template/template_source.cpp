// 阶段 7 模板组件 1:数据源。演示"produce 接口 + 定时器驱动"。
// 不引入任何控制语义,只演示:组件声明产出 (joint, interface),
// 话题由框架推导,外部(规格/校验器/echo)按同一规则访问。
#include <chrono>
#include <memory>

#include <builtin_interfaces/msg/time.hpp>
#include <framework/component_base.hpp>
#include <mvp_msgs/msg/float64_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mvp_core::template_ {

class TemplateSource : public framework::ComponentBase {
public:
    explicit TemplateSource(const rclcpp::NodeOptions& options)
        : framework::ComponentBase("template_source", options) {
        const double period = declare_parameter("period", 1.0);
        publisher_ = produce_publisher<mvp_msgs::msg::Float64Stamped>("yaw", "position");
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period), [this] { publish_next(); });
    }

private:
    void publish_next() {
        auto message = std::make_unique<mvp_msgs::msg::Float64Stamped>();
        message->header.stamp = now();
        message->data = static_cast<double>(counter_++);
        publisher_->publish(std::move(message));
    }

    rclcpp::Publisher<mvp_msgs::msg::Float64Stamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
};

} // namespace mvp_core::template_

RCLCPP_COMPONENTS_REGISTER_NODE(mvp_core::template_::TemplateSource)
