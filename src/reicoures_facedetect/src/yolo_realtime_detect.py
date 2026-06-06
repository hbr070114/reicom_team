#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8 实时目标检测 - ROS摄像头版本
用于线上仿真比赛，订阅 /berxel_base/color/image_raw 话题
检测 fire（火）和 fire_extinguisher（灭火器）

使用方法：
    python3 yolo_realtime_detect.py

按 'q' 键退出
"""

import os
import cv2
# 重定向 ROS 日志到 /tmp（解决只读文件系统问题）
os.environ['ROS_LOG_DIR'] = '/tmp/ros_logs'
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# ==================== 配置区 ====================
MODEL_PATH = '/home/reicom2025/reicom_test_runs/train_exp-4/weights/best.pt'
CAMERA_TOPIC = '/berxel_base/color/image_raw'
CONF_THRESHOLD = 0.25          # 置信度阈值，低于此值不显示
DISPLAY_WINDOW = 'YOLO Detection - Reicom'
# =================================================

def main():
    # 初始化 ROS 节点
    rospy.init_node("yolo_realtime_detect", anonymous=True)
    print("=" * 50)
    print("YOLOv8 实时目标检测")
    print(f"模型: {MODEL_PATH}")
    print(f"摄像头话题: {CAMERA_TOPIC}")
    print(f"置信度阈值: {CONF_THRESHOLD}")
    print("按 'q' 键退出")
    print("=" * 50)

    # 加载 YOLO 模型
    print("\n[INFO] 正在加载模型...")
    model = YOLO(MODEL_PATH)
    print("[INFO] 模型加载完成!")

    # 创建 CvBridge 用于 ROS Image <-> OpenCV 转换
    bridge = CvBridge()

    # 等待摄像头话题就绪
    print(f"\n[INFO] 等待摄像头话题 {CAMERA_TOPIC} ...")
    try:
        image_msg = rospy.wait_for_message(CAMERA_TOPIC, Image, timeout=30)
        print("[INFO] 摄像头已连接!")
    except Exception as e:
        print(f"[ERROR] 等待摄像头超时: {e}")
        print("请确认仿真环境已启动并发布了摄像头图像")
        return

    # 创建显示窗口
    cv2.namedWindow(DISPLAY_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(DISPLAY_WINDOW, 960, 720)

    print("\n[INFO] 开始实时检测...")

    rate = rospy.Rate(15)  # 检测频率 15Hz（可根据性能调整）

    while not rospy.is_shutdown():
        try:
            # 从 ROS 话题获取最新图像
            image_msg = rospy.wait_for_message(CAMERA_TOPIC, Image, timeout=5.0)
            frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as e:
            print(f"[WARN] 获取图像失败: {e}")
            continue

        # YOLO 推理
        results = model(frame, conf=CONF_THRESHOLD, verbose=False)

        # 在图像上绘制检测结果
        annotated_frame = results[0].plot()

        # 显示 FPS 和检测数量信息
        num_detections = len(results[0].boxes)
        info_text = f"Objects: {num_detections}"
        cv2.putText(annotated_frame, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 显示结果
        cv2.imshow(DISPLAY_WINDOW, annotated_frame)

        # 按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n[INFO] 用户退出")
            break

        rate.sleep()

    cv2.destroyAllWindows()
    print("[INFO] 检测节点已关闭")


if __name__ == "__main__":
    main()
