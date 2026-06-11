#!/bin/bash
#===============================================================================
# ARS408 Radar - CAN Interface Setup Script
# 一键配置 CAN 接口，每次重启后执行一次即可
# Usage: sudo ./setup_can.sh [bitrate] [device]
# Default: bitrate=500000, device=can0
#===============================================================================
set -e

BITRATE=${1:-500000}
DEVICE=${2:-can0}

echo "[1/4] Loading kernel module: can"
modprobe can

echo "[2/4] Loading kernel module: can_raw"
modprobe can_raw

echo "[3/4] Loading kernel module: mttcan"
modprobe mttcan

echo "[4/4] Setting up ${DEVICE} with bitrate ${BITRATE}"
ip link set "${DEVICE}" up type can bitrate "${BITRATE}"

echo ""
echo "==================== CAN Setup Complete ===================="
echo "Device: ${DEVICE}"
echo "Bitrate: ${BITRATE}"
echo ""
ip -details link show "${DEVICE}" 2>/dev/null || echo "(device status unavailable)"

# 可选：发送测试帧确认工作
echo ""
echo "Tip: Test with: candump ${DEVICE}"
echo "Tip: Switch to objects mode: cansend ${DEVICE} 200#FF190000089D0100"
echo "Tip: Switch to clusters mode: cansend ${DEVICE} 200#FF050000109D0100"
