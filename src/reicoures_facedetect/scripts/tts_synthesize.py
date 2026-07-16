#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
语音合成脚本：为4个场馆生成 back_xxx.pcm 音频文件
使用 /REIService/voice_tts 服务

用法：
  1. 确保 roscore 和 rei_voice 服务已启动
  2. python3 tts_synthesize.py

生成文件：
  - back_beijing.pcm  "这里就是北京馆啦，我要继续回去工作啦！"
  - back_shanghai.pcm "这里就是上海馆啦，我要继续回去工作啦！"
  - back_guangzhou.pcm "这里就是广州馆啦，我要继续回去工作啦！"
  - back_jilin.pcm    "这里就是吉林馆啦，我要继续回去工作啦！"
"""

import rospy
import os
import shutil
from rei_voice.srv import REITts, REITtsRequest

AUDIO_DIR = "/home/reicom2025/bobac3_ws/src/robot_audio/audio"

# 4个场馆的back音频配置
VENUES = [
    ("北京馆", "back_beijing.pcm"),
    ("上海馆", "back_shanghai.pcm"),
    ("广州馆", "back_guangzhou.pcm"),
    ("吉林馆", "back_jilin.pcm"),
]

TEMPLATE = "这里就是{}啦，我要继续回去工作啦！"


def synthesize(text, filename):
    """调用TTS服务合成语音并保存"""
    rospy.loginfo(f"[合成] 文本: \"{text}\"")
    rospy.loginfo(f"[合成] 目标文件: {filename}")

    try:
        rospy.wait_for_service('/REIService/voice_tts', timeout=10)
        tts_client = rospy.ServiceProxy('/REIService/voice_tts', REITts)

        req = REITtsRequest()
        req.text = text
        req.is_play = True  # 合成并播放

        resp = tts_client.call(req)

        if resp.success:
            # filePath 返回的是生成的文件路径
            src_path = resp.filePath
            dst_path = os.path.join(AUDIO_DIR, filename)

            if src_path and os.path.exists(src_path):
                shutil.copy2(src_path, dst_path)
                rospy.loginfo(f"[合成] ✅ {filename} 合成成功并保存到 {dst_path}")
                return True
            else:
                rospy.logwarn(f"[合成] TTS成功但源文件不存在: {src_path}")
                return False
        else:
            rospy.logerr(f"[合成] ❌ 合成失败: {resp.message}")
            return False

    except rospy.ServiceException as e:
        rospy.logerr(f"[合成] TTS服务调用失败: {e}")
        return False


def main():
    rospy.init_node("tts_synthesizer", anonymous=True)

    rospy.loginfo("=" * 50)
    rospy.loginfo("   语音合成器 - 生成4个场馆的back音频")
    rospy.loginfo("=" * 50)

    # 检查音频目录是否存在
    if not os.path.exists(AUDIO_DIR):
        rospy.logerr(f"音频目录不存在: {AUDIO_DIR}")
        return

    success_count = 0
    for venue_name, filename in VENUES:
        text = TEMPLATE.format(venue_name)
        rospy.loginfo(f"\n--- 正在合成 {venue_name} ---")
        if synthesize(text, filename):
            success_count += 1
        rospy.sleep(3)  # 间隔3秒，避免TTS服务过载

    rospy.loginfo("=" * 50)
    rospy.loginfo(f"  完成！成功 {success_count}/{len(VENUES)} 个文件")
    rospy.loginfo(f"  文件目录: {AUDIO_DIR}")
    rospy.loginfo("=" * 50)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
