// RMCS 对照：定时器承担 executor 的心跳；缓存上一拍力矩，把硬件读写形成的环断开。
// 这只模拟 partner component 的数据流作用，不提供 RMCS 的固定执行顺序。
#include <chrono>
#include <memory>
#include <stdexcept>

#include <builtin_interfaces/msg/time.hpp>
#include <mvp_msgs/msg/float64_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mvp_core::hardware {

class MockMotor : public rclcpp::Node {
public:
    explicit MockMotor(const rclcpp::NodeOptions& options)
        : Node("mock_motor", options) {
        // 只在启动时读取，设为只读，避免运行时 param set "成功"却不生效。
        rcl_interfaces::msg::ParameterDescriptor read_only;
        read_only.read_only = true;
        const double update_rate = declare_parameter("update_rate", 1000.0, read_only);
        inertia_ = declare_parameter("inertia", 0.01, read_only);
        damping_ = declare_parameter("damping", 0.01, read_only);
        if (update_rate <= 0.0 || inertia_ <= 0.0 || damping_ < 0.0)
            throw std::invalid_argument("update_rate、inertia 必须为正，damping 不能为负");
        dt_ = 1.0 / update_rate;

        angle_publisher_ = create_publisher<Msg>("angle", rclcpp::QoS{1});
        velocity_publisher_ = create_publisher<Msg>("velocity", rclcpp::QoS{1});
        torque_subscription_ = create_subscription<Msg>(
            "control_torque", rclcpp::QoS{1}, [this](Msg::UniquePtr message) {
                torque_ = message->data;
                torque_stamp_ = message->header.stamp;
            });
        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { update(); });
    }

private:
    using Msg = mvp_msgs::msg::Float64Stamped;
    using Time = builtin_interfaces::msg::Time;

    void update() {
        // 收到过力矩后，它应当正好属于上一拍；否则说明回调链没能在一个周期内跑完。
        if (torque_stamp_ != Time{} && torque_stamp_ != previous_stamp_)
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "控制链超时：缓存力矩不属于上一拍");

        velocity_ += dt_ * (torque_ - damping_ * velocity_) / inertia_;
        angle_ += dt_ * velocity_;

        const Time stamp = now();
        publish(*angle_publisher_, angle_, stamp);
        publish(*velocity_publisher_, velocity_, stamp);
        previous_stamp_ = stamp;
    }

    // 转移独占所有权，使单接收者的进程内链路零拷贝；外部订阅仍走中间件。
    static void publish(rclcpp::Publisher<Msg>& publisher, double value, const Time& stamp) {
        auto message = std::make_unique<Msg>();
        message->header.stamp = stamp;
        message->data = value;
        publisher.publish(std::move(message));
    }

    double dt_, inertia_, damping_;
    double angle_ = 0.0, velocity_ = 0.0, torque_ = 0.0;
    Time torque_stamp_, previous_stamp_;
    rclcpp::Publisher<Msg>::SharedPtr angle_publisher_, velocity_publisher_;
    rclcpp::Subscription<Msg>::SharedPtr torque_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace mvp_core::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(mvp_core::hardware::MockMotor)
