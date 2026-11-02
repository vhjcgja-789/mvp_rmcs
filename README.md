# mvp_rmcs — 可复用的 ROS2 Jazzy 开发容器模板

原生 ROS2 + 最小工具集,结构对齐 RMCS 风格。阶段 6 用可组合节点实现单关节双环控制:

- `mvp_msgs`:带时间戳的标量消息
- `mvp_core`:假电机与可复用 PID 组件
- `mvp_bringup`:单线程容器、接线和参数

目标架构(现代工程重构版,面向完整复刻 RMCS)见 [架构文档](docs/architecture.md)。

- 非 root 用户 `ubuntu`(uid 1000),sudo 免密
- 默认 shell zsh + oh-my-zsh(af-magic)
- 项目根目录直接挂载为容器工作目录(`/workspaces/mvp_rmcs`),无 `workspace/` 子目录
- Dev Containers 为唯一容器入口(镜像预构建,devcontainer 不参与构建)

## 快速开始

```bash
./code.sh   # 构建镜像并打开 VSCode → "Dev Containers: Reopen in Container"
```

命令行通道(不带 VSCode):

```bash
docker compose up -d --build
docker compose exec mvp_rmcs bash
```

容器内构建和启动阶段 6:

```bash
cd /workspaces/mvp_rmcs
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch mvp_bringup single_joint.launch.py config:=single_joint
```

另一个已 source 工作空间的终端发送目标角度:

```bash
ros2 topic pub --once /yaw/target_angle mvp_msgs/msg/Float64Stamped '{data: 1.0}'
ros2 topic echo /yaw/angle --field data
```

角度和速度单位分别为 rad、rad/s,力矩为 N·m。PID 增益按拍定义,改变更新频率后需重新整定。

## 阶段 7(基础架构骨架)

规格驱动的装配 + 启动接线校验,暂不含控制,只有模板组件演示契约与自动接线:

```bash
ros2 launch mvp_bringup stage7.launch.py   # 拉起容器 + wiring_checker(失败即 shutdown)
ros2 topic echo /stage7_demo/yaw/position --field data   # source 每 0.5s 递增发布
ros2 topic echo /stage7_demo/yaw/command  --field data   # processor 转发(×2)
```

- 组件清单/接线在 `config/stage7_demo.yaml`,launch 只负责生成容器与校验;
- 组件通过 `ComponentBase` 声明 `claim/produce`(见 `mvp_core/src/framework/`),
  话题名由规则自动推导,不再手写 remap;
- 拼错规格里的接口名 → `wiring_checker` 启动即 FATAL,而非静默断链。

## 目录结构

```
mvp_rmcs/
├── Dockerfile                # 镜像定义
├── docker-compose.yml        # 构建 + CLI 通道
├── .devcontainer/            # Dev Containers 配置(devcontainer.json + 专用 compose + scripts/)
├── env_setup.bash / .zsh     # 环境模板:进 shell 自动 source ROS 环境
├── code.sh                   # 一键构建并打开 Dev Containers
├── src/                      # colcon 源码目录(项目根即工作空间)
└── docs/                     # 配置文档
```
