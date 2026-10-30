#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# 本机特殊处理(仅当需要时生效):
# 这台机器的 docker 守护进程配置的代理是 http://127.0.0.1:7892,
# 但当前没有服务监听该端口(真正可用的代理在 127.0.0.1:7897)。
# 于是 docker 拉镜像会报 "proxyconnect ... connection refused"。
# 这里: 如果 7892 连不通、且系统有 socat,就在 7892 上转发到 7897。
# 其他机器上没有这个问题时,本脚本什么都不做。
# 如果以后代理端口变了,改下面 PORT_NEW 即可。
# -----------------------------------------------------------------------------
set -u

OLD=7892   # docker 守护进程以为的代理端口
NEW=7897   # 实际可用的代理端口

if ! command -v socat >/dev/null 2>&1; then
  exit 0
fi

# 7892 已经通了就不管它
if curl -s -m 2 -x "http://127.0.0.1:${OLD}" -o /dev/null https://registry-1.docker.io/v2/ 2>/dev/null; then
  exit 0
fi

# 7892 不通 -> 起一个转发: 7892 -> 7897
nohup socat TCP-LISTEN:${OLD},fork,reuseaddr,keepalive TCP:127.0.0.1:${NEW} >/dev/null 2>&1 &
sleep 0.5

# 确认转发生效
if curl -s -m 2 -x "http://127.0.0.1:${OLD}" -o /dev/null https://registry-1.docker.io/v2/ 2>/dev/null; then
  echo "[ensure-proxy] 已在 127.0.0.1:${OLD} 启动到 127.0.0.1:${NEW} 的转发"
else
  echo "[ensure-proxy] 警告: 7892 转发似乎没生效,构建拉镜像可能失败" >&2
fi
