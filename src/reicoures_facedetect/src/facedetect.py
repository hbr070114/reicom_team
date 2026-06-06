#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
from face_rec.srv import recognition_results, recognition_resultsRequest
from robot_audio.srv import robot_tts

# 创建客户端
facedet_client = rospy.ServiceProxy('/face_recognition_results', recognition_results)
tts_client = rospy.ServiceProxy('/robot_tts', robot_tts)

# 身份映射：人脸数据库标签 -> 身份角色
ROLE_MAP = {
    "小侯": "参观者",
    "管理员-小戴": "管理员",
}

# 阶段定义
STAGE_WAIT_VISITOR = 1    # 等待参观者（任务一前）
STAGE_TASK1_RUNNING = 2    # 任务一执行中（静默）
STAGE_WAIT_ADMIN = 3       # 等待管理员（任务二前）
STAGE_FINISHED = 4         # 全部完成


def face_detect():
    labels = []
    try:
        srv = recognition_resultsRequest()
        srv.mode = 1
        srv.str = "/head_camera/image_raw"
        response = facedet_client.call(srv)

        if response.success:
            for face_data in response.result.face_data:
                labels.append(face_data.header.frame_id)
            return labels
        else:
            return labels
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return labels


def speak(text):
    """调用TTS语音合成服务（可选，失败不阻塞）"""
    try:
        rospy.wait_for_service('/robot_tts', timeout=3)
        response = tts_client.call(text, True)
        if response.audiopath:
            rospy.loginfo(f"语音播报成功: {text}")
        else:
            rospy.logwarn(f"语音播报返回为空")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn(f"TTS服务不可用，跳过语音播报（{e}）")


if __name__ == "__main__":
    rospy.init_node("face_detect_node")

    # 当前阶段，可通过 ROS 参数外部修改
    stage = rospy.get_param('~stage', STAGE_WAIT_VISITOR)

    rate = rospy.Rate(2)  # 每秒检测2次

    # 等待识别服务就绪
    rospy.loginfo("等待人脸识别服务启动...")
    rospy.wait_for_service('/face_recognition_results')
    rospy.loginfo("人脸识别服务已连接")
    rospy.loginfo("=" * 40)
    rospy.loginfo("当前阶段: %d (1=等待参观者, 2=任务一, 3=等待管理员, 4=结束)", stage)
    rospy.loginfo("=" * 40)

    while not rospy.is_shutdown():
        # 读取当前阶段（支持运行时修改）
        stage = rospy.get_param('~stage', stage)

        if stage == STAGE_FINISHED:
            rospy.loginfo("人脸识别环节全部完成！")
            break

        if stage == STAGE_TASK1_RUNNING:
            # 任务一执行中，静默等待
            rate.sleep()
            continue

        # 阶段1或3：进行人脸检测
        labels = face_detect()

        if len(labels) > 0:
            for label in labels:
                role = ROLE_MAP.get(label, None)

                if stage == STAGE_WAIT_VISITOR and role == "参观者":
                    msg = f"你好，欢迎{role}，请跟我来。"
                    rospy.loginfo(msg)
                    speak(msg)
                    # 自动切换到任务一阶段
                    rospy.set_param('~stage', STAGE_TASK1_RUNNING)
                    stage = STAGE_TASK1_RUNNING
                    rospy.loginfo(">>> 已进入任务一阶段（人脸检测静默中）<<<")
                    rospy.loginfo(">>> 完成任务一后请执行: rosparam set /face_detect_node/stage 3 <<<")

                elif stage == STAGE_WAIT_ADMIN and role == "管理员":
                    msg = f"{role}，您好！"
                    rospy.loginfo(msg)
                    speak(msg)
                    # 自动切换到结束阶段
                    rospy.set_param('~stage', STAGE_FINISHED)
                    stage = STAGE_FINISHED
                    rospy.loginfo(">>> 人脸识别环节全部完成！<<<")

                elif role is not None:
                    rospy.loginfo(f"识别到: {label}（当前阶段不需要此身份）")

        rate.sleep()
