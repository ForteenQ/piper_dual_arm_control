#!/bin/bash
# ==============================================================================
# PIPER 双臂 CAN 接口永久识别与开机自启配置脚本
# 用法: sudo bash setup_dual_can_udev.sh
# 功能:
#   1. 依据两个 USB-to-CAN 适配器出厂唯一序列号创建 udev 规则
#      - Serial 0025002C5547570420303135 -> 永久命名为 can_piper_0 (1Mbps UP)
#      - Serial 002E00344648570220393831 -> 永久命名为 can_piper_1 (1Mbps UP)
#   2. 以后每次开机或重插 USB 线，系统会自动拉起 1Mbps CAN 总线，无需再手动输密码！
#   3. 立即在当前会话生效并激活双臂 CAN 通信。
# ==============================================================================
set -e

if [ "$EUID" -ne 0 ]; then
    echo "❌ 请使用 sudo 运行此脚本: sudo bash $0"
    exit 1
fi

SERIAL_0="0025002C5547570420303135"
SERIAL_1="002E00344648570220393831"

echo "📝 正在写入 /etc/udev/rules.d/99-piper-can.rules ..."
cat << EOF > /etc/udev/rules.d/99-piper-can.rules
# AgileX PIPER Dual-Arm Permanent CAN Naming & Auto-Up (delayed down->bitrate->up avoids gs_usb race)
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="${SERIAL_0}", NAME="can_piper_0", RUN+="/bin/sh -c 'sleep 0.4; /sbin/ip link set can_piper_0 down 2>/dev/null; /sbin/ip link set can_piper_0 type can bitrate 1000000; /sbin/ip link set can_piper_0 up'"
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="${SERIAL_1}", NAME="can_piper_1", RUN+="/bin/sh -c 'sleep 0.4; /sbin/ip link set can_piper_1 down 2>/dev/null; /sbin/ip link set can_piper_1 type can bitrate 1000000; /sbin/ip link set can_piper_1 up'"
EOF

chmod 644 /etc/udev/rules.d/99-piper-can.rules
udevadm control --reload-rules

echo "🔧 正在按硬件序列号激活当前已连接的双臂 CAN 接口..."
modprobe gs_usb || true

for iface in $(ip -br link show type can | awk '{print $1}'); do
    SERIAL_PATH="/sys/class/net/${iface}/device/../serial"
    if [ -f "$SERIAL_PATH" ]; then
        DEV_SER=$(cat "$SERIAL_PATH" | tr -d ' \n\r')
    else
        DEV_SER=""
    fi

    TARGET=""
    if [ "$DEV_SER" = "$SERIAL_0" ]; then
        TARGET="can_piper_0"
    elif [ "$DEV_SER" = "$SERIAL_1" ]; then
        TARGET="can_piper_1"
    fi

    if [ -n "$TARGET" ]; then
        ip link set "$iface" down 2>/dev/null || true
        if [ "$iface" != "$TARGET" ]; then
            ip link set "$iface" name "$TARGET"
            echo "   ✅ 接口 $iface (序列号 $DEV_SER) -> 重命名为 $TARGET"
        fi
        ip link set "$TARGET" type can bitrate 1000000
        ip link set "$TARGET" up
        echo "   🚀 $TARGET 已激活 (1000000 bps UP)"
    else
        echo "   ⚠️ 发现未知序列号 CAN 接口: $iface ($DEV_SER)，激活为 1Mbps"
        ip link set "$iface" down 2>/dev/null || true
        ip link set "$iface" up type can bitrate 1000000
    fi
done

echo ""
echo "=== 当前 CAN 总线状态 ==="
ip -br link show type can
echo "🎉 配置完成！今后开机及热插拔将自动激活，双臂 Web 控制台可秒连！"
