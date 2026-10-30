#!/usr/bin/env bash
# 构建镜像并打开 Dev Containers(唯一入口)
# .devcontainer/scripts/ensure-proxy.sh 仅本机需要,其他机器无副作用
set -euo pipefail
cd "$(dirname "$0")"

./.devcontainer/scripts/ensure-proxy.sh
docker compose build

code .
