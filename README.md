# mvp_rmcs — 可复用的 ROS2 Jazzy 开发容器模板

原生 ROS2 + 最小工具集,哨兵硬件层基于 ros2_control,设备协议取自 RMCS。当前进度见 [Plan 01](docs/01_architecture.md):

- `mvp_hardware`:每块板一个 SystemInterface;librmcs 由 FetchContent 拉取(v3.1.0,带 SHA256 校验)
- `mvp_controllers`:控制器插件
- `mvp_bringup`:URDF、控制器配置、launch

总设计见 [架构文档](docs/architecture.md)。

- 非 root 用户 `ubuntu`(uid 1000),sudo 免密
- 默认 shell zsh + oh-my-zsh(af-magic)
- 编译器 GCC 14(librmcs 需要 C++23 `<print>`)
- 项目根目录直接挂载为容器工作目录(`/workspaces/mvp_rmcs`),无 `workspace/` 子目录
- Dev Containers 为唯一容器入口(镜像预构建,devcontainer 不参与构建)

## 宿主机一次性准备:udev 规则

容器以非 root 用户访问板子(VID `a11c`),需在**宿主机**放开权限:

```bash
sudo cp .devcontainer/udev/95-rmcs-slave.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

容器使用 `privileged: true` + 绑定整个 `/dev`(同 RMCS),板子重插后无需重启容器。

## 快速开始

```bash
./code.sh   # 构建镜像并打开 VSCode → "Dev Containers: Reopen in Container"
```

镜像构建时由 rosdep 按各包 `package.xml` 安装依赖;改了 `package.xml` 就重新构建镜像
(devcontainer 里执行 "Rebuild Container")。`rosdep update` 要访问 GitHub,需要代理时
把代理地址写成 docker0 网关(构建容器里的 127.0.0.1 不是宿主机),例如本机:

```bash
HTTPS_PROXY=http://172.17.0.1:7892 HTTP_PROXY=http://172.17.0.1:7892 ./code.sh
```

命令行通道(不带 VSCode):

```bash
docker compose up -d --build
docker compose exec mvp_rmcs bash
```

容器内构建(首次会从 GitHub 下载 librmcs,需要代理时同样设置 `HTTPS_PROXY`):

```bash
cd /workspaces/mvp_rmcs
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 阶段 0 验证:容器能看到板子

```bash
lsusb | grep a11c                                       # a11c:d401 = C 板,a11c:a801 = RmcsBoardLite
ros2 run mvp_hardware board_probe d4-2184               # 打印 connected 即成功
ros2 run mvp_hardware board_probe d4-2184 --skip-version-check   # 上一条报固件版本不符时再试
```

## 阶段 1 验证:一块板 + 一个 3508,只读反馈

```bash
ros2 launch mvp_bringup bringup.launch.py
ros2 control list_hardware_interfaces          # 应有 chassis_wheel_lb/{position,velocity,effort}
ros2 topic echo /joint_states                  # 手拨电机,position(rad)/velocity(rad/s)跟着变
```

板子没连上时 `on_configure` 打印 librmcs 设备发现报告,controller_manager 随即退出。

## 目录结构

```
mvp_rmcs/
├── Dockerfile                # 镜像定义
├── docker-compose.yml        # 构建 + CLI 通道
├── .devcontainer/            # Dev Containers 配置(devcontainer.json + 专用 compose + scripts/ + udev/)
├── env_setup.bash / .zsh     # 环境模板:进 shell 自动 source ROS 环境
├── code.sh                   # 一键构建并打开 Dev Containers
├── src/                      # colcon 源码目录(项目根即工作空间)
└── docs/                     # 配置文档
```
