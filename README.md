# mvp_rmcs — 可复用的 ROS2 Jazzy 开发容器模板

原生 ROS2 + 最小工具集,结构对齐 RMCS 风格:

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
