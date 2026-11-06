#!/usr/bin/env bash
# 把 DDS 的"发现"从多播改成只跟指定机器单播。
#
# 解决的现象：WiFi 路由器被多播帧压死（所有设备断开、后台进不去）。
# 原理：Fast-DDS 默认靠多播找节点，多播帧在 WiFi 上按 1 Mbps 最低速率
# 广播、无确认、每个客户端都要处理，对便宜路由器的转发队列是灾难。
# 改成 initialPeersList 点名互联之后，空口上就没有多播了。
#
# 用法（在要参与通信的每台机器上各跑一次，互相填对方的 IP）：
#   bash setup_dds_unicast.sh 192.168.1.103 192.168.1.50
#
# 只写 ~/.dds_unicast/ 下的两个文件，不改 /etc、不改 .bashrc。
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "用法: $0 <peer_ip> [peer_ip ...]" >&2
    echo "例:   $0 192.168.1.103 192.168.1.50" >&2
    exit 2
fi

OUT="${HOME}/.dds_unicast"
mkdir -p "$OUT"
XML="${OUT}/no_multicast.xml"

{
    echo '<?xml version="1.0" encoding="UTF-8"?>'
    echo '<dds xmlns="http://www.eprosima.com">'
    echo '  <profiles>'
    echo '    <transport_descriptors>'
    echo '      <transport_descriptor>'
    echo '        <transport_id>shm</transport_id>'
    echo '        <type>SHM</type>'
    echo '      </transport_descriptor>'
    echo '      <transport_descriptor>'
    echo '        <transport_id>udp4</transport_id>'
    echo '        <type>UDPv4</type>'
    echo '      </transport_descriptor>'
    echo '    </transport_descriptors>'
    echo '    <participant profile_name="unicast_only" is_default_profile="true">'
    echo '      <rtps>'
    echo '        <useBuiltinTransports>false</useBuiltinTransports>'
    echo '        <userTransports>'
    echo '          <transport_id>shm</transport_id>'
    echo '          <transport_id>udp4</transport_id>'
    echo '        </userTransports>'
    echo '        <builtin>'
    echo '          <avoid_builtin_multicast>true</avoid_builtin_multicast>'
    # 注意：initialPeersList 是 <builtin> 的直接子节点，不能放进
    # <discovery_config> 里面，否则 Fast-DDS 会报
    # "Invalid element found into 'discoverySettingsType'" 并整个 profile 失效。
    echo '          <initialPeersList>'
    for ip in "$@"; do
        echo "            <locator><udpv4><address>${ip}</address></udpv4></locator>"
    done
    echo '          </initialPeersList>'
    echo '        </builtin>'
    echo '      </rtps>'
    echo '    </participant>'
    echo '  </profiles>'
    echo '</dds>'
} > "${XML}"

cat > "${OUT}/env_dds.sh" <<EOF
# 只在当前终端生效；关掉这个终端，或 unset 下面这行，就回到系统默认。
export FASTRTPS_DEFAULT_PROFILES_FILE="${XML}"
echo "[dds] 单播发现已启用，peers: $*"
EOF

echo "已生成:"
echo "  配置  ${XML}"
echo "  开关  ${OUT}/env_dds.sh"
echo
echo "启用方式（只在跑 demo 的那个终端里）:"
echo "  source ${OUT}/env_dds.sh"
echo
echo "撤销:"
echo "  unset FASTRTPS_DEFAULT_PROFILES_FILE"
