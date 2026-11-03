// C 板(librmcs CBoard)的 ros2_control 硬件组件:一块板对应一个 SystemInterface。
// 板子收发逻辑来源:RMCS/rmcs_ws/src/rmcs_core/src/hardware/sentry.cpp(BottomBoard)。
#include <exception>
#include <memory>
#include <string>
#include <string_view>

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <librmcs/agent/c_board.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "mvp_hardware/device/dji_motor.hpp"

namespace mvp_hardware {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// 关联的设备集合
struct BoardDevices {
    device::DjiMotor& wheel_motor;
};

// librmcs 适配器:一块 C 板对应一个 SystemInterface。
class Board final
    : private BoardDevices
    , private librmcs::agent::CBoard {
public:
    Board(
        const BoardDevices& devices, std::string_view serial_number,
        const librmcs::agent::AdvancedOptions& options)
        : BoardDevices{devices}
        , CBoard{serial_number, options} {}

private:
    auto can1_receive_callback(const librmcs::data::CanDataView& data) -> void override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        auto can_id = data.can_id;
        if (can_id == 0x201)
            // 原子操作：当多个线程同时访问同一变量时,需要确保数据的一致性（特别是一读一写）
            wheel_motor.store_status(data.can_data);
    }
};

// ros2_control 组件
class CBoardSystem final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override {
        if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        const auto& hardware_parameters = info_.hardware_parameters;

        const auto serial_it = hardware_parameters.find("serial_number");

        if (serial_it == hardware_parameters.end()) {
            RCLCPP_ERROR(get_logger(), "缺少硬件参数 serial_number");
            return CallbackReturn::ERROR;
        }

        serial_number_ = serial_it->second;
        if (const auto it = hardware_parameters.find("skip_version_check");
            it != hardware_parameters.end())
            skip_version_check_ = hardware_interface::parse_bool(it->second);

        if (info_.joints.size() != 1) {
            RCLCPP_ERROR(get_logger(), "阶段 1 只支持一个关节,实际 %zu 个", info_.joints.size());
            return CallbackReturn::ERROR;
        }

        const auto& joint = info_.joints.front();
        const auto param = [&joint](const std::string& name) -> const std::string* {
            const auto it = joint.parameters.find(name);
            return it == joint.parameters.end() ? nullptr : &it->second;
        };

        auto config = device::DjiMotor::Config{device::DjiMotor::Type::kM3508};
        if (const auto* value = param("reduction_ratio"))
            config.set_reduction_ratio(hardware_interface::stod(*value));
        if (const auto* value = param("reversed"); value && hardware_interface::parse_bool(*value))
            config.set_reversed();
        if (const auto* value = param("multi_turn"); value && hardware_interface::parse_bool(*value))
            config.enable_multi_turn_angle();
        wheel_motor_.configure(config);

        position_interface_ = joint.name + "/" + hardware_interface::HW_IF_POSITION;
        velocity_interface_ = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
        effort_interface_ = joint.name + "/" + hardware_interface::HW_IF_EFFORT;

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        try {
            board_ = std::make_unique<Board>(
                BoardDevices{wheel_motor_}, serial_number_,
                librmcs::agent::AdvancedOptions{skip_version_check_});
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "连接板子 %s 失败:\n%s", serial_number_.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
        board_.reset();
        return CallbackReturn::SUCCESS;
    }

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override {
        wheel_motor_.update_status();
        set_state(position_interface_, wheel_motor_.angle());
        set_state(velocity_interface_, wheel_motor_.velocity());
        set_state(effort_interface_, wheel_motor_.torque());
        return return_type::OK;
    }

    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
        return return_type::OK;
    }

private:
    std::string serial_number_;
    bool skip_version_check_ = false;

    device::DjiMotor wheel_motor_;
    std::string position_interface_, velocity_interface_, effort_interface_;

    // 声明在设备之后:先断开板子(停止回调),再析构设备
    std::unique_ptr<Board> board_;
};

} // namespace mvp_hardware

PLUGINLIB_EXPORT_CLASS(mvp_hardware::CBoardSystem, hardware_interface::SystemInterface)
