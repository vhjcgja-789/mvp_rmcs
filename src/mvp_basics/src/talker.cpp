// mvp_basics/talker —— 阶段1:发布者 + 服务端
// 意图:在阶段0的基础上,加一个 /set_hello 服务,让外部能动态改发布内容,
// 并立刻得到"改没改成"的确认 —— 这就是"请求-应答"机制。
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mvp_basics/msg/hello.hpp"
#include "mvp_basics/srv/set_hello.hpp"  // 由 SetHello.srv 生成的类型

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("talker");

  // 定时器回调(发布)和服务回调(改内容)共享的"当前发布内容"。
  // 为什么现在不用加锁:rclcpp 默认单线程执行器,同一时刻只会有一个
  // 回调在跑,所以这个变量不会被两个回调同时读写。等以后上多线程执行器,
  // 或者回调里做耗时操作时,才需要引入互斥 —— 阶段4前后会碰到。
  auto content = std::make_shared<std::string>("你好,ROS2!");

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

  std::int32_t seq = 0;

  auto timer = node->create_wall_timer(1s, [&, content]() {
    auto msg = std::make_shared<mvp_basics::msg::Hello>();
    msg->content = *content;  // 每次发布都读"最新"内容
    msg->seq = seq++;
    publisher->publish(*msg);
    RCLCPP_INFO(node->get_logger(), "time:%ds, %s",
                msg->seq, msg->content.c_str());
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
