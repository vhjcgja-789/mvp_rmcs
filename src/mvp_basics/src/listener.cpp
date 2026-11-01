// mvp_basics/listener —— 阶段0:订阅者
// 意图:订阅 /hello,收到一条打印一条。让你看到"发布/订阅"是
// 一对多、松耦合的:listener 不知道 talker 是谁,只认话题名。
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mvp_basics/msg/hello.hpp"

// 收到消息时被 ROS2 调用的回调函数(订阅者是被动的:数据来找你,
// 而不是你主动去取)。参数是共享指针,消息用完自动释放。
void on_hello(const mvp_basics::msg::Hello::SharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("listener"), "收到：%ds,%s",
              msg->seq, msg->content.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("listener");

  // 创建订阅者:订阅 /hello,有消息来了 ROS2 自动调用 on_hello。
  // 回调模型 = "声明式":你只负责注册"来了怎么办",不负责"去取"。
  auto sub = node->create_subscription<mvp_basics::msg::Hello>(
      "/hello", 10, on_hello);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
