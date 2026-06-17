#!/bin/bash
# -*- coding: utf-8 -*-
# ============================================
# 比赛一键停止脚本
# 用法: bash stop_competition.sh
# ============================================

echo -e "\033[0;33m正在停止所有比赛服务...\033[0m"

# 停止各节点
pkill -f "facedetect.py" 2>/dev/null && echo "  [OK] facedetect 已停止"
pkill -f "reinovo_bobac3_sim" 2>/dev/null && echo "  [OK] 动态障碍物已停止"
pkill -f "ar_base_sim\|pose_adjust\|ar_track_alvar" 2>/dev/null && echo "  [OK] AR追踪服务已停止"
pkill -f "demo_nav_2d" 2>/dev/null && echo "  [OK] 仿真环境已停止"
pkill -f "voice_navgoal.py" 2>/dev/null && echo "  [OK] voice_navgoal 已停止"
pkill -f "face_verification.py" 2>/dev/null && echo "  [OK] face_rec 已停止"
pkill -f "rei_voice_node" 2>/dev/null && echo "  [OK] rei_voice 已停止"
pkill -f "usb_cam_node" 2>/dev/null && echo "  [OK] usb_cam 已停止"

# 可选：停止 roscore（谨慎使用，会影响其他ROS节点）
# kill $(pgrep -f rosmaster) 2>/dev/null && echo "  [OK] roscore 已停止"

echo -e "\033[0;32m============================================\033[0m"
echo -e "\033[0;32m   所有服务已停止\033[0m"
echo -e "\033[0;32m============================================\033[0m"
