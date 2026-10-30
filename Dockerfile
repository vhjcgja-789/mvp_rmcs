# mvp_rmcs — 可复用的 ROS2 Jazzy 开发容器模板
FROM ros:jazzy-ros-base

# 时区/语言环境变量: TZ 指定时区; DEBIAN_FRONTEND=noninteractive 让 apt 安装时免交互
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive
# 基础系统包:
#   tzdata   时区数据库(配合上面的 TZ 生效)
#   locales  语言环境生成器(下面用 locale-gen 生成 UTF-8 环境)
# --no-install-recommends: 只装"必需"依赖、跳过推荐包,减小镜像体积
# rm -rf /var/lib/apt/lists/*: 清掉 apt 索引缓存,同样为了减小体积(镜像分层里的"瘦身"惯例)
RUN apt-get update && apt-get install -y --no-install-recommends \
        tzdata locales \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ---- ROS2 构建与调试工具 ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        # python3-colcon-common-extensions: colcon,ROS2 官方多包编译工具
        #   (colcon build 编译工作空间、colcon test 跑测试),替代 ROS1 的 catkin_make
        python3-colcon-common-extensions \
        # ros-jazzy-rviz2:     3D 可视化工具,看机器人模型/TF 坐标/点云/图像
        # ros-jazzy-turtlesim: 入门教学小乌龟仿真(ROS2 版的 "hello world")
        # ros-jazzy-tf2-tools: TF 坐标变换命令行工具(view_frames 导出变换树图)
        # ros-jazzy-ros2bag:   话题数据录制(ros2 bag record)与回放(ros2 bag play)
        ros-jazzy-rviz2 ros-jazzy-turtlesim ros-jazzy-tf2-tools ros-jazzy-ros2bag \
        # 通用开发工具:
        #   git              版本管理
        #   vim              文本编辑器
        #   tmux             终端复用(一个终端开多个窗格)
        #   curl             命令行下载/接口调试(支持多协议)
        #   wget             简单文件下载
        #   zsh              默认 shell(下面配合 oh-my-zsh 使用)
        #   bash-completion  命令 TAB 补全增强
        #   python3-pip      Python 包管理(pip install 装第三方库)
        git vim tmux curl wget zsh bash-completion python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 给 ubuntu 用户免密 sudo:容器里装系统包/改配置不用输密码
RUN echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

COPY --chown=1000:1000 env_setup.bash /home/ubuntu/env_setup.bash
COPY --chown=1000:1000 env_setup.zsh /home/ubuntu/env_setup.zsh
# 让 bash 启动时自动 source 环境脚本
RUN echo '[ -f ~/env_setup.bash ] && . ~/env_setup.bash' >> /home/ubuntu/.bashrc

RUN chsh -s /usr/bin/zsh ubuntu
WORKDIR /home/ubuntu
ENV USER=ubuntu \
    WORKDIR=/workspaces/mvp_rmcs

USER ubuntu

# 安装 oh-my-zsh(zsh 插件/主题框架),换主题为 af-magic,并在 ~/.zshrc 里挂上环境脚本
RUN sh -c "$(wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" \
    && sed -i 's/ZSH_THEME="[a-z0-9\-]*"/ZSH_THEME="af-magic"/g' ~/.zshrc \
    && echo '[ -f ~/env_setup.zsh ] && . ~/env_setup.zsh' >> ~/.zshrc

WORKDIR /workspaces/mvp_rmcs
