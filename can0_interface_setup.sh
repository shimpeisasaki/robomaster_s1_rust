#!/bin/bash
# CAN0インターフェースセットアップスクリプト

echo "CAN0インターフェースを設定中..."

# 既存のcan0インターフェースがあれば停止
sudo ip link set down can2 2>/dev/null

# CANインターフェースを設定（ビットレート1Mbps）
sudo ip link set can2 type can bitrate 1000000

# 送信キューサイズを設定（インターフェースをアップする前に）
sudo ip link set can2 txqueuelen 10000

# インターフェースを有効化
sudo ip link set up can2

echo "CAN2インターフェースの設定完了"
echo "インターフェース情報:"
ip link show can2
echo ""
echo "CANの詳細情報:"
ip -details link show can2
