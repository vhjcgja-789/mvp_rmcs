#!/bin/zsh
# mvp_rmcs 环境设置:进 shell 即 source 好 ROS2 环境(RMCS 风格)
#   1) 系统安装的 ROS2 Jazzy
#   2) 项目根目录(挂载点 /workspaces/mvp_rmcs)里编译出来的包(如果有)
# 由 Dockerfile 拷到 ~/env_setup.zsh,并经 ~/.zshrc 自动 source。

: "${MVP_RMCS_PATH:=/workspaces/mvp_rmcs}"

export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/jazzy/setup.zsh

if [ -f "${MVP_RMCS_PATH}/install/setup.zsh" ]; then
    source "${MVP_RMCS_PATH}/install/setup.zsh"
fi
