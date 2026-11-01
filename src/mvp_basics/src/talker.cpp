// mvp_basics/talker —— 阶段3:发布者 + 服务端 + 动作服务端,配置走参数
// 意图:阶段1 有了"请求-应答"(/set_hello),阶段2 有了长任务 /move_distance 动作;
// 这一阶段把"发布频率、内容前缀"从代码里拿出去,改成启动时由外部注入的参数。
// 话题名则不做成参数,交给 remap 在外部改线(原因见 create_publisher 处的注释)。
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"          // 动作服务端/客户端库
#include "mvp_basics/msg/hello.hpp"
#include "mvp_basics/srv/set_hello.hpp"             // 由 SetHello.srv 生成的类型
#include "mvp_basics/action/move_distance.hpp"      // 由 MoveDistance.action 生成的类型

using namespace std::chrono_literals;

using MoveDistance = mvp_basics::action::MoveDistance;
// GoalHandle = "某一个正在执行的目标"的把柄:通过它发反馈、查是否被取消、交结果。
using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveDistance>;

// 真正"干活"的函数:模拟以 0.5 m/s 移动,每 0.2 s 走 0.1 m 并发一次反馈。
// 它会跑好几秒,所以必须在独立线程里执行(原因见 main 里 handle_accepted 的注释)。
void execute_move(const std::shared_ptr<GoalHandleMove> goal_handle) {
  const float target = goal_handle->get_goal()->distance;
  const float step = 0.1f;
  rclcpp::Rate rate(5);  // 5 Hz,即每拍 0.2 s

  auto feedback = std::make_shared<MoveDistance::Feedback>();
  auto result = std::make_shared<MoveDistance::Result>();
  float traveled = 0.0f;

  while (traveled < target && rclcpp::ok()) {
    // 每一拍先看调用方是否喊停:可取消是 action 区别于服务的关键能力之一。
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->traveled = traveled;
      goal_handle->canceled(result);  // 以"已取消"收尾,结果里带上走了多少
      RCLCPP_INFO(rclcpp::get_logger("talker"), "action: 已取消,走了 %.1f m",
                  traveled);
      return;
    }

    traveled = std::min(traveled + step, target);
    feedback->traveled = traveled;
    feedback->percent = traveled / target * 100.0f;
    goal_handle->publish_feedback(feedback);  // 反馈:过程中可以发很多次
    RCLCPP_INFO(rclcpp::get_logger("talker"), "action: %.1f / %.1f m (%.0f%%)",
                traveled, target, feedback->percent);
    rate.sleep();
  }

  // 走完全程:以"成功"收尾。结果只发一次,发完这个目标就结束了。
  if (rclcpp::ok()) {
    result->success = true;
    result->traveled = traveled;
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("talker"), "action: 完成 %.1f m", traveled);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("talker");

  // 参数:先"声明"(名字 + 默认值),再"读取"。
  // 为什么必须先声明:ROS2 只接受节点声明过的参数,外部传了拼错的名字会被
  // 直接忽略而不是悄悄生效;默认值也让节点不传任何参数时照样能跑。
  // 声明后外部可以通过命令行(--ros-args -p)、launch 文件、YAML 来覆盖默认值。
  node->declare_parameter("publish_rate_hz", 1.0);
  node->declare_parameter("prefix", "");

  double rate_hz = node->get_parameter("publish_rate_hz").as_double();
  if (rate_hz <= 0.0) {
    RCLCPP_WARN(node->get_logger(), "publish_rate_hz=%.2f 非法,改用 1.0", rate_hz);
    rate_hz = 1.0;
  }

  // 定时器回调(发布)和服务回调(改内容)共享的"当前发布内容"。
  // 为什么现在不用加锁:rclcpp 默认单线程执行器,同一时刻只会有一个
  // 回调在跑,所以这个变量不会被两个回调同时读写。阶段2 的动作虽然开了
  // 工作线程,但它只碰自己的 goal_handle,不碰 content,所以仍然安全。
  auto content = std::make_shared<std::string>("你好,ROS2!");

  // 话题名仍写 "/hello",但它只是"默认接线"。想改成别的名字,用 remap 在
  // 外部改(见 launch 文件),代码一行不用动。话题名不做成参数,是因为
  // "接到哪根线上"属于系统怎么连,而不是节点自己怎么干活 —— 这正是 remap 的职责,
  // 它对所有节点、所有话题/服务/动作名都通用,不用每个节点各写一套参数。
  auto publisher =
      node->create_publisher<mvp_basics::msg::Hello>("/hello", 10);

  // 服务端:监听 /set_hello,谁调用我就立刻回答。
  // 与话题的关键区别:调用方必须等这个回调填好 response 才拿到结果,
  // 所以服务天然适合"命令 + 确认",而不是持续数据流。
  // 回调参数:req = 请求数据,resp = 要回填的应答;返回 true 表示应答已就绪。
  auto server = node->create_service<mvp_basics::srv::SetHello>(
      "/set_hello",
      [content](const std::shared_ptr<mvp_basics::srv::SetHello::Request> req,
                std::shared_ptr<mvp_basics::srv::SetHello::Response> resp) {
        *content = req->content;  // 记下新内容,之后的发布都用它
        resp->success = true;
        resp->message = "已改为: " + req->content;
        RCLCPP_INFO(rclcpp::get_logger("talker"), "service: 改为 -> %s",
                    req->content.c_str());
        return true;
      });

  // 动作服务端:监听 /move_distance。和服务不同,它要注册三个回调,
  // 对应一个目标的三个关键时刻:来了要不要接 / 被要求取消准不准 / 接了之后怎么执行。
  auto action_server = rclcpp_action::create_server<MoveDistance>(
      node, "/move_distance",
      // 1) 新目标到达:只做快速检查,决定接受或拒绝(这里拒绝非正数距离)。
      [](const rclcpp_action::GoalUUID &,
         std::shared_ptr<const MoveDistance::Goal> goal) {
        RCLCPP_INFO(rclcpp::get_logger("talker"), "action: 收到目标 %.1f m",
                    goal->distance);
        if (goal->distance <= 0.0f) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // 2) 调用方请求取消:这里一律同意;真正停下来是在 execute_move 里检查后完成的。
      [](const std::shared_ptr<GoalHandleMove>) {
        RCLCPP_INFO(rclcpp::get_logger("talker"), "action: 收到取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // 3) 目标已接受:开一个线程去执行,然后立刻返回。
      // 为什么不能直接在这里循环:单线程执行器同一时刻只跑一个回调,
      // 如果在这里卡住几秒,定时器发不了 /hello、服务也应答不了,
      // 连"取消请求"这个回调都进不来 —— 整个节点就"假死"了。
      [](const std::shared_ptr<GoalHandleMove> goal_handle) {
        std::thread(execute_move, goal_handle).detach();
      });

  std::int32_t seq = 0;

  // 频率只在启动时读一次:定时器周期一旦创建就固定了,运行中改这个参数不会生效。
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  auto timer = node->create_wall_timer(period, [&, content]() {
    // 前缀每次发布都重新读:所以运行中 `ros2 param set` 改它会立刻生效。
    // 与上面的频率对照,体会"启动时读一次"和"每次用时读"的区别。
    const std::string prefix = node->get_parameter("prefix").as_string();
    auto msg = std::make_shared<mvp_basics::msg::Hello>();
    msg->content = prefix + *content;  // 前缀 + 最新内容
    msg->seq = seq++;
    publisher->publish(*msg);
    RCLCPP_INFO(node->get_logger(), "#%d, %s",
                msg->seq, msg->content.c_str());
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
