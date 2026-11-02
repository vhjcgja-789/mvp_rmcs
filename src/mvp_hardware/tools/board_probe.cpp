// 阶段 0 验证工具:按序列号构造 CBoard,能连上就打印 connected,失败时打印 librmcs 的设备发现报告。
// 用法:ros2 run mvp_hardware board_probe <serial> [--skip-version-check]
#include <cstdio>
#include <exception>
#include <string_view>

#include <librmcs/agent/c_board.hpp>

int main(int argc, char** argv) {
    const std::string_view serial = argc > 1 ? argv[1] : "";
    const bool skip_version_check = argc > 2 && std::string_view{argv[2]} == "--skip-version-check";
    try {
        librmcs::agent::CBoard board{serial, librmcs::agent::AdvancedOptions{skip_version_check}};
        std::puts("connected");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
}
