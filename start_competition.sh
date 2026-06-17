#!/bin/bash
# -*- coding: utf-8 -*-
# ============================================
# 比赛一键启动脚本
# 用法: bash start_competition.sh [task]
#   task=1  任务一（迎宾引导，默认）
#   task=2  任务二（巡检模式）
#
# 完整流程：
#   阶段1: 摄像头持续检测 → 识别小侯 → awake1+talk+guide → 导航深圳馆 → back → 返回原点
#   阶段3: 持续检测 → 识别小戴 → awake2+talk → 巡检模式
# ============================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_WS="/home/reicom2025/ros_workspace"
BOBAC3_WS="/home/reicom2025/bobac3_ws"
FACE_DETECT="$ROS_WS/src/reicoures_facedetect/src/facedetect.py"
LOG_DIR="/tmp/competition_logs"

TASK=${1:-1}

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}   比赛一键启动 - 任务${TASK}${NC}"
echo -e "${GREEN}============================================${NC}"

mkdir -p "$LOG_DIR"

# ---- 环境变量 ----
echo -e "\n${BLUE}[1/4] 设置环境变量...${NC}"
export LD_LIBRARY_PATH="$ROS_WS/src/rei_voice/libs/x86_64:$LD_LIBRARY_PATH"
source "$ROS_WS/devel/setup.bash"
source "$BOBAC3_WS/devel/setup.bash" 2>/dev/null || true
echo -e "${GREEN}  OK${NC}"

# ---- roscore ----
echo -e "\n${BLUE}[2/5] 检查 roscore...${NC}"
if pgrep -f "roscore" > /dev/null 2>&1; then
    echo -e "${GREEN}  已运行${NC}"
else
    roscore > "$LOG_DIR/roscore.log" 2>&1 &
    sleep 3
    pgrep -f "roscore" > /dev/null && echo -e "${GREEN}  已启动${NC}" || { echo -e "${RED}  失败！${NC}"; exit 1; }
fi

# ---- 仿真环境（Gazebo + RViz + 摄像头 + move_base）----
echo -e "\n${BLUE}[3/6] 启动仿真环境...${NC}"
source /opt/ros/noetic/setup.bash
source /home/reicom2025/bobac3_ws/devel/setup.bash
roslaunch bobac3_navigation demo_nav_2d.launch > "$LOG_DIR/simulation.log" 2>&1 &
sleep 12
echo -e "${GREEN}  已启动（Gazebo + RViz + 摄像头 + move_base）${NC}"

# ---- 动态障碍物（官方启动器）----
echo -e "\n${BLUE}[3.5/6] 启动动态障碍物...${NC}"
cd /home/reicom2025/Downloads/zhangaiwu
chmod +x reinovo_bobac3_sim 2>/dev/null || true
./reinovo_bobac3_sim > "$LOG_DIR/obstacle.log" 2>&1 &
sleep 3
cd "$ROS_WS"
echo -e "${GREEN}  已启动（reinovo_bobac3_sim 官方动态障碍物）${NC}"

# ---- AR追踪服务（充电桩二维码定位，使用底部摄像头检测AR码）----
echo -e "\n${BLUE}[3.6/6] 启动AR追踪服务...${NC}"
source /opt/ros/noetic/setup.bash
source /home/reicom2025/bobac3_ws/devel/setup.bash
roslaunch ar_pose ar_base_sim.launch > "$LOG_DIR/ar_track.log" 2>&1 &
sleep 3
echo -e "${GREEN}  已启动（AR二维码追踪 /bottom_camera）${NC}"

# ---- 相对移动服务（AR追踪和充电流程依赖此服务）----
echo -e "\n${BLUE}[3.7/6] 启动相对移动服务...${NC}"
roslaunch relative_move relative_move.launch > "$LOG_DIR/relative_move.log" 2>&1 &
sleep 2
echo -e "${GREEN}  已启动（/relative_move 服务）${NC}"

# ---- rei_voice (PCM播放) ----
echo -e "\n${BLUE}[4/6] 启动语音服务...${NC}"
roslaunch rei_voice start_aiui.launch > "$LOG_DIR/rei_voice.log" 2>&1 &
sleep 2
echo -e "${GREEN}  已启动${NC}"

# ---- 人脸识别服务 ----
echo -e "\n${BLUE}[5/6] 启动人脸识别服务...${NC}"
roslaunch face_rec face_verification.launch > "$LOG_DIR/face_rec.log" 2>&1 &
sleep 3
echo -e "${GREEN}  已启动${NC}"

# ---- 主流程控制器（facedetect.py）----
echo -e "\n${BLUE}[6/6] 启动主流程控制器...${NC}"
python3 "$FACE_DETECT" __log:="$LOG_DIR/facedetect.log" _stage:=$TASK &
DETECT_PID=$!
sleep 2
if kill -0 $DETECT_PID 2>/dev/null; then
    echo -e "${GREEN}  已启动 (PID: $DETECT_PID)${NC}"
else
    echo -e "${RED}  启动失败！查看日志: $LOG_DIR/facedetect.log${NC}"
fi

# ---- 汇总 ----
echo -e "\n${GREEN}============================================${NC}"
echo -e "${GREEN}   全部就绪！任务${TASK}模式${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "  流程:"
echo -e "    摄像头检测人脸 → 自动触发后续流程"
echo -e ""
echo -e "  任务一流程:"
echo -e "    识别小侯 → awake1 → talk1 → guide → 导航深圳馆 → back → 回原点"
echo -e ""
echo -e "  任务二流程:"
echo -e "    识别小戴 → awake2 → talk2 → 逐场馆巡检(YOLO检测) → 回原点 → 自动退出"
echo -e ""
echo -e "  动态障碍物: 圆柱体沿路径自动巡逻（模拟行人）"
echo -e ""
echo -e "  日志: ${YELLOW}$LOG_DIR${NC}"
echo -e "  停止: ${YELLOW}bash ~/ros_workspace/stop_competition.sh${NC}"
echo -e "  切阶段: ${YELLOW}rosparam set /face_detect_node/stage 3${NC}"
echo -e ""
echo -e "${GREEN}摄像头对准人脸即可开始！${NC}"
echo ""

wait
