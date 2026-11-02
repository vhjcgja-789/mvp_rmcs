// 阶段 7 框架:启动期接线校验(独立可执行,一个进程核对一个组件实例)。
//
// 做什么:对照"规格推导出的期望话题"与"真实 ROS 图",fail-fast。
// - 期望话题清单由 launch 生成器(唯一解析规格的地方)以参数传入,本节点不解析 YAML;
// - 校验三类问题:期望话题缺失/多余、期望输入没有发布者、同话题类型不一致;
// - 任何不一致 → FATAL + 退出码 1;launch 检测到非零退出即整体 shutdown。
//
// 为什么:把"接线拼错导致控制环静默不闭合"变成"启动即报错"(对应 RMCS init()
// 的配对检查);发现是异步的,所以先等待目标节点出现在图里再核对。
#include <algorithm>
#include <chrono>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace {

// 把图的完全限定名 "/stage7_demo/source" 拆成 {namespace, name},namespace 带前导 '/';
// 根命名空间返回 {"/", name}。per-node 图查询需要这两个参数。
std::pair<std::string, std::string> split_fully_qualified_name(const std::string& fq) {
    const std::string s = fq.empty() || fq.front() != '/' ? fq : fq.substr(1);
    const std::size_t pos = s.find_last_of('/');
    if (pos == std::string::npos)
        return {"/", s};
    return {"/" + s.substr(0, pos), s.substr(pos + 1)};
}

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wiring_checker");

    const std::string node_name =
        node->declare_parameter<std::string>("node_name", "");
    const std::string node_namespace =
        node->declare_parameter<std::string>("node_namespace", "/");
    const std::vector<std::string> expected_subs =
        node->declare_parameter<std::vector<std::string>>("expected_subs", {});
    const std::vector<std::string> expected_pubs =
        node->declare_parameter<std::vector<std::string>>("expected_pubs", {});

    const std::string ns = node_namespace.empty()
                               ? "/"
                               : (node_namespace.front() == '/' ? node_namespace
                                                                : "/" + node_namespace);
    if (node_name.empty()) {
        RCLCPP_FATAL(node->get_logger(), "缺少参数 node_name,拒绝校验");
        rclcpp::shutdown();
        return 2;
    }

    // 发现是异步的:轮询等待目标节点的订阅/发布出现在图里。
    // 图查询在"节点尚未被发现"时可能抛异常或返回空,两种情况都当作"还没就绪"。
    constexpr int kMaxTries = 40;
    constexpr auto kWait = std::chrono::milliseconds(250);
    std::vector<std::string> actual_subs, actual_pubs;
    bool found = false;
    for (int i = 0; i < kMaxTries && !found; ++i) {
        try {
            const auto subs = node->get_subscriber_names_and_types_by_node(node_name, ns);
            const auto pubs = node->get_publisher_names_and_types_by_node(node_name, ns);
            if (!subs.empty() || !pubs.empty()) {
                for (const auto& entry : subs)
                    actual_subs.push_back(entry.first);
                for (const auto& entry : pubs)
                    actual_pubs.push_back(entry.first);
                found = true;
                continue;
            }
        } catch (const std::exception&) {
            // 目标节点未知(未发现):按未就绪处理,继续等。
        }
        rclcpp::sleep_for(kWait);
    }

    std::vector<std::string> problems;
    if (!found) {
        problems.push_back("节点 " + ns + "/" + node_name + " 未在图里发现(容器未启动?规格名写错?)");
    }

    if (found) {
        // 1) 期望 vs 实际:双向比对订阅/发布集合,捕获拼错、多余、缺失。
        const auto missing =
            [&problems](const std::vector<std::string>& expected,
                        const std::vector<std::string>& actual, const char* what) {
                for (const auto& e : expected)
                    if (std::find(actual.begin(), actual.end(), e) == actual.end())
                        problems.push_back(std::string("期望") + what + "缺失: " + e);
                for (const auto& a : actual)
                    if (std::find(expected.begin(), expected.end(), a) == expected.end())
                        problems.push_back(std::string("多余") + what + "(规格未声明): " + a);
            };
        missing(expected_subs, actual_subs, "订阅");
        missing(expected_pubs, actual_pubs, "发布");

        // 2) 连接完整性:每个期望输入话题必须存在发布者(聚合全图节点的发布话题)。
        // 节点可能在列举与查询之间下线,个别查询失败就跳过该节点。
        std::vector<std::string> global_pubs;
        for (const auto& fq : node->get_node_names()) {
            const auto [nns, nname] = split_fully_qualified_name(fq);
            try {
                for (const auto& entry :
                     node->get_publisher_names_and_types_by_node(nname, nns))
                    global_pubs.push_back(entry.first);
            } catch (const std::exception&) {
            }
        }
        for (const auto& s : expected_subs)
            if (std::find(global_pubs.begin(), global_pubs.end(), s) == global_pubs.end())
                problems.push_back("输入话题没有发布者(上游未接或拼错): " + s);

        // 3) 类型一致性:同话题出现两种消息类型 → 实际不会连通,静默失效。
        try {
            const auto global_types = node->get_topic_names_and_types();
            for (const auto& topic : expected_subs) {
                const auto it = global_types.find(topic);
                if (it != global_types.end()) {
                    std::set<std::string> types(it->second.begin(), it->second.end());
                    if (types.size() > 1)
                        problems.push_back("话题类型不一致(发布/订阅对不上): " + topic);
                }
            }
        } catch (const std::exception&) {
            // 图信息获取失败:跳过类型检查,缺失/连接检查已覆盖主要风险。
        }
    }

    if (!problems.empty()) {
        for (const auto& p : problems)
            RCLCPP_FATAL(node->get_logger(), "%s", p.c_str());
        RCLCPP_FATAL(node->get_logger(), "接线校验失败,拒绝运行");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(), "接线校验通过:节点 %s/%s,订阅 %zu,发布 %zu",
        ns.c_str(), node_name.c_str(), actual_subs.size(), actual_pubs.size());
    rclcpp::shutdown();
    return 0;
}
