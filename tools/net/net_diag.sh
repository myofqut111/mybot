#!/usr/bin/env bash
# 一眼看清无线链路被谁吃掉了。
#
# 用法: bash net_diag.sh [网卡名]
#   不填网卡名就自动取默认路由那张。
#
# 输出三段：
#   1) 各话题实际带宽（找出像 /image_raw 这种吃带宽的）
#   2) 多播帧密度（WiFi 路由器最怕这个）
#   3) 接口收发统计与错误计数（看有没有重传/丢包）
set +e

IF="${1:-$(ip route get 8.8.8.8 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')}"
IF="${IF:-wlan0}"

echo "================ 接口 ${IF} ================"

echo
echo "---- 1) 各话题带宽（采样 10 秒）----"
if command -v ros2 >/dev/null 2>&1; then
    timeout 12 ros2 topic bw --all --window 10 2>/dev/null | tail -30
    if [ "${PIPESTATUS[0]}" = "124" ]; then echo "(已按 10 秒窗口截断)"; fi
else
    echo "(没有 ros2 命令，先 source 一下 ROS 环境)"
fi

echo
echo "---- 2) 多播帧密度（3 秒采样）----"
if command -v tcpdump >/dev/null 2>&1; then
    n=$(timeout 3 tcpdump -i "${IF}" -nn -c 2000 'ether multicast' 2>/dev/null | wc -l)
    echo "3 秒内多播帧: ${n} 帧  （>100 就偏多了；关掉多播发现应该接近 0）"
else
    echo "(没装 tcpdump: sudo apt install -y tcpdump)"
fi

echo
echo "---- 3) 接口统计（收发与错误）----"
ip -s link show "${IF}" 2>/dev/null | tail -5

echo
echo "---- 4) 有没有别的机器在抢同一个 IP ----"
if command -v arping >/dev/null 2>&1; then
    echo "(想查 ARP 冲突: sudo arping -D -I ${IF} <你的IP>)"
fi
echo "本机地址:"
ip -brief addr show "${IF}" 2>/dev/null
