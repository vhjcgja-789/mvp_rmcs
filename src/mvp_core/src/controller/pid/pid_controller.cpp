// RMCS 对照：用订阅回调代替 update()，用 remap 代替 Input/OutputInterface 的名字配对。
// 原生执行器不保证回调顺序，因此按 stamp 配对两路输入，替代 RMCS 拓扑排序给出的同拍语义。
#include <memory>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <controller/pid/pid_calculator.hpp>
#include <mvp_msgs/msg/float64_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mvp_core::controller::pid {

class PidController : public rclcpp::Node {
public:
    explicit PidController(const rclcpp::NodeOptions& options)
        : Node("pid_controller", options)
        , calculator_(
              declare_parameter("kp", 0.0), declare_parameter("ki", 0.0),
              declare_parameter("kd", 0.0)) {
        // 参数类型已由声明固定，提交成功后直接换增益即可在线调参。
        parameter_callback_ = add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>&) {
                calculator_.set_gains(
                    get_parameter("kp").as_double(), get_parameter("ki").as_double(),
                    get_parameter("kd").as_double());
            });

        control_publisher_ = create_publisher<Msg>("control", rclcpp::QoS{1});
        measurement_subscription_ = create_subscription<Msg>(
            "measurement", rclcpp::QoS{1}, [this](Msg::UniquePtr message) {
                measurement_ = std::move(message);
                update();
            });
        setpoint_subscription_ = create_subscription<Msg>(
            "setpoint", rclcpp::QoS{1}, [this](Msg::UniquePtr message) {
                setpoint_ = std::move(message);
                update();
            });
    }

private:
    using Msg = mvp_msgs::msg::Float64Stamped;

    // 两路谁后到谁触发，回调顺序因此无关紧要；每个测量值只参与一次计算。
    // 链路内的目标必须与测量同拍；外部目标不带 stamp（为 0），随每个测量值计算。
    void update() {
        if (!measurement_ || !setpoint_)
            return;
        const auto& setpoint_stamp = setpoint_->header.stamp;
        if (setpoint_stamp != builtin_interfaces::msg::Time{}
            && setpoint_stamp != measurement_->header.stamp)
            return;

        auto control = std::make_unique<Msg>();
        control->header = measurement_->header;
        control->data = calculator_.update(setpoint_->data - measurement_->data);
        control_publisher_->publish(std::move(control));
        measurement_.reset();
    }

    PidCalculator calculator_;
    Msg::UniquePtr measurement_, setpoint_;
    rclcpp::Publisher<Msg>::SharedPtr control_publisher_;
    rclcpp::Subscription<Msg>::SharedPtr measurement_subscription_, setpoint_subscription_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

} // namespace mvp_core::controller::pid

RCLCPP_COMPONENTS_REGISTER_NODE(mvp_core::controller::pid::PidController)
