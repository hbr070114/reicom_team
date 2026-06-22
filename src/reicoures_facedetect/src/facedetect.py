#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
比赛主流程控制器
流程：
  阶段1: 持续检测小侯人脸 → awake1+talk1 → 等待语音确认"去深圳馆" → guide+导航深圳馆+back → 回原点
  阶段2: 导航执行中（静默）
  阶段3: 持续检测小戴人脸 → awake2+talk2 → 开始巡检模式
  阶段4: 全部完成 → 自动退出
"""

import rospy
import time
import os
import sys
import actionlib
import threading
from face_rec.srv import recognition_results, recognition_resultsRequest
from rei_voice.srv import REIPlayer, REIPlayerRequest
from std_srvs.srv import SetBool, SetBoolRequest
from relative_move.srv import SetRelativeMove, SetRelativeMoveRequest
from ar_pose.srv import Track, TrackRequest
from rei_voice.msg import REIResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# ==================== 路径配置 ====================
AUDIO_DIR = "/home/reicom2025/bobac3_ws/src/robot_audio/audio"

# ==================== 音频映射 ====================
FACE_AUDIO = {
    "小侯": {
        "awake": os.path.join(AUDIO_DIR, "awake1.pcm"),
        "talk": os.path.join(AUDIO_DIR, "talk1.pcm"),
        "guide": os.path.join(AUDIO_DIR, "guide.pcm"),
        "arrived": os.path.join(AUDIO_DIR, "back.pcm"),
    },
    "管理员-小戴": {
        "awake": os.path.join(AUDIO_DIR, "awake2.pcm"),
        "talk": os.path.join(AUDIO_DIR, "talk2.pcm"),
    }
}

# ==================== 导航点坐标 [x, y, z, w]（来自voice_navgoal.py 原始数据，不可修改）====================
NAV_POINTS = {
    "北京馆": [2.47933, 0.18254, 0.0125, 1.0],
    "上海馆": [1.034243, 2.20507, 0.0125, 1.0],    # x+0.05，更靠近牌子
    "吉林馆": [2.479169, 2.19698, 0.0125, 1.0],
    "深圳馆": [1.011638, 1.2328, 0.0125, 1.0],      # x+0.05，更靠近牌子
    "广州馆": [2.46769, 1.1995, 0.0125, 1.0],
    "原点": [0.0, 0.0, 0.0, 1.0],
    "充电桩": [0.45, 1.9499, -0.7, 0.7],            # 充电桩位置（来自auto_charge.py）
}

# 巡检顺序（比赛要求）：上海 → 深圳 → 北京 → 广州 → 吉林
PATROL_ORDER = ["上海馆", "深圳馆", "北京馆", "广州馆", "吉林馆"]

# YOLO模型路径
YOLO_MODEL_PATH = '/home/reicom2025/reicom_test_runs/train_exp-4/weights/best.pt'
CAMERA_TOPIC = '/berxel_base/color/image_raw'

# 异常音频映射：场馆 -> {异常类型: 音频文件}
ABNORMAL_AUDIO = {
    "北京馆": {"fire": os.path.join(AUDIO_DIR, "beijing1.pcm"), "no_extinguisher": os.path.join(AUDIO_DIR, "beijing2.pcm")},
    "上海馆": {"fire": os.path.join(AUDIO_DIR, "shanghai1.pcm"), "no_extinguisher": os.path.join(AUDIO_DIR, "shanghai2.pcm")},
    "吉林馆": {"fire": os.path.join(AUDIO_DIR, "jilin1.pcm"), "no_extinguisher": os.path.join(AUDIO_DIR, "jilin2.pcm")},
    "深圳馆": {"fire": os.path.join(AUDIO_DIR, "shenzhen1.pcm"), "no_extinguisher": os.path.join(AUDIO_DIR, "shenzhen2.pcm")},
    "广州馆": {"fire": os.path.join(AUDIO_DIR, "guangzhou1.pcm"), "no_extinguisher": os.path.join(AUDIO_DIR, "guangzhou2.pcm")},
}

# ==================== 身份映射 ====================
ROLE_MAP = {
    "小侯": "参观者",
    "管理员-小戴": "管理员",
}

# ==================== 阶段定义 ====================
STAGE_WAIT_VISITOR = 1   # 等待参观者（任务一前）
STAGE_TASK1_RUNNING = 2   # 任务一执行中
STAGE_WAIT_ADMIN = 3      # 等待管理员（任务二前）
STAGE_FINISHED = 4        # 全部完成

# 语音确认关键词
CONFIRM_KEYWORDS_TASK1 = ["去", "带我去", "参观", "带我去参观", "好的", "确认", "是"]
CONFIRM_KEYWORDS_TASK2 = ["开始巡检", "开始执行巡检", "执行巡检", "开始检查", "好的", "确认", "是"]

# 场馆语音识别关键词映射（用户说"去北京馆"→识别为北京馆）
VENUE_KEYWORDS = {
    "北京馆": ["北京馆", "北京"],
    "上海馆": ["上海馆", "上海"],
    "吉林馆": ["吉林馆", "吉林"],
    "深圳馆": ["深圳馆", "深圳"],
    "广州馆": ["广州馆", "广州"],
}

# 场馆中文名转拼音映射（用于构造guide/back音频文件名）
PINYIN_MAP = {
    "北京馆": "beijing",
    "上海馆": "shanghai",
    "吉林馆": "jilin",
    "深圳馆": "shenzhen",
    "广州馆": "guangzhou",
}


# ==================== 客户端初始化 ====================
rospy.init_node("face_detect_node")

# 人脸识别客户端
rospy.loginfo("等待人脸识别服务...")
rospy.wait_for_service('/face_recognition_results')
facedet_client = rospy.ServiceProxy('/face_recognition_results', recognition_results)
rospy.loginfo("人脸识别服务已连接")

# PCM音频播放客户端
rospy.wait_for_service('/REIService/PcmPlayer', timeout=10)
player_client = rospy.ServiceProxy('/REIService/PcmPlayer', REIPlayer)
rospy.loginfo("PCM播放服务已连接")

# 录音控制客户端（用于语音交互时控制录音开关）
rospy.wait_for_service('/REIService/RecordAudio', timeout=10)
record_audio_client = rospy.ServiceProxy('/REIService/RecordAudio', SetBool)
rospy.loginfo("录音控制服务已连接")

# 相对移动客户端（用于充电时靠近/离开充电桩）
try:
    rospy.wait_for_service('/relative_move', timeout=10)
    relmove_client = rospy.ServiceProxy('/relative_move', SetRelativeMove)
    rospy.loginfo("相对移动服务已连接")
except Exception as e:
    rospy.logwarn(f"相对移动服务未找到: {e}")
    relmove_client = None

# AR追踪客户端（用于充电桩二维码定位）
try:
    rospy.wait_for_service('/track', timeout=10)
    track_client = rospy.ServiceProxy('/track', Track)
    rospy.loginfo("AR追踪服务已连接")
except Exception as e:
    rospy.logwarn(f"AR追踪服务未找到: {e}")
    track_client = None

# 巡检命令发布器
patrol_pub = rospy.Publisher("/patrol_command", String, queue_size=10)

# 语音识别结果（用于等待用户确认）
_voice_result = None
_voice_result_lock = threading.Lock()


def _aiui_result_cb(msg):
    """订阅AIUI语音识别结果"""
    global _voice_result
    with _voice_result_lock:
        _voice_result = msg
    # 调试：打印所有收到的消息（使用正确的字段名）
    msg_type = getattr(msg, 'type', 'unknown')
    msg_iat = getattr(msg, 'iat', '')
    # NLP意图的query字段
    nlp_text = ''
    if hasattr(msg, 'intent') and len(msg.intent) > 0:
        nlp_text = getattr(msg.intent[0], 'query', '') or ''
    rospy.loginfo(f"[语音回调] type={msg_type}, iat=\"{msg_iat}\", nlp_query=\"{nlp_text}\"")


# 订阅语音识别话题
rospy.Subscriber("/REITopic/AIUIResult", REIResult, _aiui_result_cb, queue_size=10)
rospy.loginfo("语音识别话题已订阅: /REITopic/AIUIResult")


# YOLO模型初始化
rospy.loginfo("加载YOLO检测模型...")
_yolo_model = YOLO(YOLO_MODEL_PATH)
_cv_bridge = CvBridge()
rospy.loginfo(f"YOLO模型已加载: {YOLO_MODEL_PATH}")


# ==================== 功能函数 ====================

def get_pcm_duration(file_path, sample_rate=16000, channels=1, sample_width=2):
    """
    根据PCM文件大小计算音频时长（秒）
    默认参数：16kHz 单声道 16-bit（标准TTS输出格式）
    返回时长，若文件不存在返回0
    """
    if not file_path or not os.path.exists(file_path):
        return 0
    size = os.path.getsize(file_path)
    return size / (sample_rate * channels * sample_width)


def playAudio(file_path):
    """使用PCM播放器播放音频文件，返回估算时长（秒）"""
    if not file_path or not os.path.exists(file_path):
        rospy.logwarn(f"音频不存在: {file_path}")
        return 0
    try:
        msg = REIPlayerRequest()
        msg.PcmPath = file_path
        resp = player_client.call(msg)
        if resp.success:
            duration = get_pcm_duration(file_path)
            rospy.loginfo(f"[音频] 播放: {os.path.basename(file_path)} (~{duration:.1f}秒)")
            return duration
        else:
            rospy.logwarn(f"[音频] 失败: {resp.message}")
            return 0
    except Exception as e:
        rospy.logwarn(f"[音频] 异常: {e}")
        return 0


def face_detect():
    """调用人脸识别服务，返回识别到的标签列表"""
    labels = []
    try:
        srv = recognition_resultsRequest()
        srv.mode = 1
        srv.str = "/head_camera/image_raw"
        response = facedet_client.call(srv)
        if response.success:
            for face_data in response.result.face_data:
                labels.append(face_data.header.frame_id)
    except rospy.ServiceException as e:
        rospy.logerr(f"人脸识别异常: {e}")
    return labels


def wait_for_voice_confirm(timeout_sec=60, keywords=None, detect_venue=False):
    """
    等待用户通过语音确认。
    用户需要说"元宝元宝"唤醒，然后说出匹配关键词。

    REIResult消息结构：
      string sid
      string type         # iat / nlp / tts
      string iat          # 语音识别原始文本（iat类型）
      REIResultNlp[] intent  # NLP意图列表，每个有 .query 字段
      string anwser

    参数：
      detect_venue: 是否同时检测场馆名称（任务一用）
    
    返回：detect_venue=True时返回 (confirmed, venue_name)
          detect_venue=False时返回 True/False
    参考 voice_navgoal.py 的 voiceResult_cb 逻辑。
    """
    global _voice_result
    if keywords is None:
        keywords = CONFIRM_KEYWORDS_TASK1

    rospy.loginfo("=" * 40)
    rospy.loginfo("[确认] 等待语音确认...")
    hint = "去XXX馆" if keywords is CONFIRM_KEYWORDS_TASK1 else "开始巡检"
    rospy.loginfo(f"[确认] 请说: 元宝元宝 -> {hint}")
    rospy.loginfo(f"[确认] 超时: {timeout_sec}秒")
    rospy.loginfo("=" * 40)

    # 开启录音（让语音模块开始监听）
    try:
        record_audio_client.call(SetBoolRequest(True))
        rospy.loginfo("[确认] 录音已开启")
    except Exception as e:
        rospy.logwarn(f"[确认] 开启录音失败: {e}")

    start_time = time.time()
    rate = rospy.Rate(10)
    msg_count = 0

    while not rospy.is_shutdown() and (time.time() - start_time) < timeout_sec:
        with _voice_result_lock:
            result = _voice_result
            _voice_result = None  # 消费掉

        if result is not None:
            msg_count += 1
            msg_type = getattr(result, 'type', '')

            # 提取文本：优先从 intent[].query 获取，其次从 iat 获取
            text = ''
            if hasattr(result, 'intent') and len(result.intent) > 0:
                text = getattr(result.intent[0], 'query', '') or ''
            if not text:
                text = getattr(result, 'iat', '') or ''
            text = str(text).strip()

            rospy.loginfo(f"[确认] 收到消息#{msg_count}: type=\"{msg_type}\", text=\"{text}\"")

            # 在文本中匹配关键词
            if text:
                # 如果开启了场馆检测，先尝试匹配场馆名称
                venue_name = None
                if detect_venue:
                    for venue, venue_keys in VENUE_KEYWORDS.items():
                        for vk in venue_keys:
                            if vk in text:
                                venue_name = venue
                                rospy.loginfo(f"[确认] 检测到场馆: {venue_name}")
                                break
                        if venue_name:
                            break

                # 匹配通用确认关键词
                for keyword in keywords:
                    if keyword in text:
                        rospy.loginfo(f"[确认] 匹配到关键词 \"{keyword}\" → 确认成功！")
                        try:
                            record_audio_client.call(SetBoolRequest(False))
                        except:
                            pass
                        if detect_venue:
                            return (True, venue_name)  # venue_name可能为None
                        return True

        rate.sleep()

    # 超时，停止录音
    try:
        record_audio_client.call(SetBoolRequest(False))
    except:
        pass
    rospy.logwarn(f"[确认] 超时！共收到{msg_count}条消息，未匹配关键词")
    if detect_venue:
        return (False, None)
    return False


def clear_costmaps():
    """
    彻底清除costmap中的障碍物残留（解决动态障碍物移动后遗留问题）
    清除所有层：obstacles + virtual_wall，多次发送确保生效
    """
    topics = [
        '/move_base/local_costmap/clear_obstacles',
        '/move_base/global_costmap/clear_obstacles',
        '/move_base/local_costmap/clear_virtual_walls',  # 新增：清除虚拟墙
        '/move_base/global_costmap/clear_virtual_walls',
    ]
    cleared = 0
    for topic in topics:
        try:
            pub = rospy.Publisher(topic, std_msgs.msg.Empty, queue_size=1, latch=True)
            # 发送3次确保清除成功
            for _ in range(3):
                pub.publish(std_msgs.msg.Empty())
                time.sleep(0.1)
            cleared += 1
            rospy.loginfo(f"[清除] 已发布: {topic}")
        except Exception as e:
            rospy.logwarn(f"[清除] 失败: {topic} - {e}")

    if cleared > 0:
        # 等待costmap更新（关键！给激光雷达时间重新扫描）
        time.sleep(2)  # 增加到2秒，确保障碍物层完全刷新
        rospy.loginfo(f"[清除] 共清除 {cleared} 个costmap层，等待2秒让雷达重新扫描")
    return cleared > 0


def wait_for_path_clear(timeout_sec=10):
    """
    等待路径畅通（给动态障碍物时间移开）
    每隔1秒清除一次costmap，等待雷达重新扫描
    返回 True 表示已等待完成
    """
    rospy.loginfo(f"[等待] 等待路径畅通... (最多{timeout_sec}秒)")
    start_time = time.time()
    waited = 0

    while not rospy.is_shutdown() and (time.time() - start_time) < timeout_sec:
        clear_costmaps()
        time.sleep(1)
        waited += 1
        if waited % 3 == 0:  # 每3秒打印一次
            rospy.loginfo(f"[等待] 已等待{waited}秒，继续观察...")

    rospy.loginfo(f"[等待] 等待结束，共{waited}秒")
    return True


# 需要特殊处理的场馆（靠近白线边缘，需要过冲+回退策略）
PRECISION_VENUES = {"深圳馆", "上海馆"}
# 过冲距离：先导航到目标前方这么多米
OVERSHOOT_DIST = 0.05  # 5cm（用户要求从10cm改为5cm）


def nav_to_goal(goal_name, max_retries=2):
    """
    导航到指定目标点，返回是否成功
    支持自动重试：失败后清除costmap再试
    新增：导航前先等待路径畅通（解决动态障碍物阻塞问题）
    特殊处理：深圳馆/上海馆使用过冲+回退策略避免踩白线
    """
    coords = NAV_POINTS.get(goal_name)
    if not coords:
        rospy.logerr(f"导航目标不存在: {goal_name}")
        return False

    # 判断是否需要精确过冲策略
    need_precision = goal_name in PRECISION_VENUES

    for attempt in range(max_retries + 1):
        # 清除costmap中的障碍物残留（解决动态障碍物扫描后遗留问题）
        target_x = coords[0]
        if need_precision:
            # 过冲策略：目标点前移10cm，越过白线后再回退
            target_x = coords[0] + OVERSHOOT_DIST
            rospy.loginfo(f"[导航] 精确模式: 目标({coords[0]}, {coords[1]}) → 过冲到({target_x}, {coords[1]})")
        else:
            rospy.loginfo(f"[导航] 准备前往 {goal_name} ({coords[0]}, {coords[1]}) [尝试{attempt+1}/{max_retries+1}]")

        # 首次尝试时，先等待路径畅通（给障碍物时间移开）
        if attempt == 0:
            wait_for_path_clear(timeout_sec=5)  # 等待5秒让障碍物移开
        else:
            # 重试时更长时间等待和清除
            clear_costmaps()
            time.sleep(3)  # 多等3秒让雷达完全刷新

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo(f"[导航] 等待 move_base 服务...")
        if not client.wait_for_server(rospy.Duration(60)):
            rospy.logerr("[导航] move_base 连接超时！")
            continue
        rospy.loginfo(f"[导航] 连接成功，前往 {goal_name} ({target_x}, {coords[1]})")

        client.cancel_all_goals()
        time.sleep(0.5)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = coords[1]
        goal.target_pose.pose.orientation.z = coords[2]
        goal.target_pose.pose.orientation.w = coords[3]

        client.send_goal(goal)
        rate = rospy.Rate(10)  # 提高检测频率（5→10Hz）
        start_time = time.time()
        nav_timeout = 120  # 给足120秒时间到达精确位置
        success_count = 0
        SUCCESS_THRESHOLD = 3  # 连续3次SUCCEEDED才算真成功

        while not rospy.is_shutdown() and (time.time() - start_time) < nav_timeout:
            state = client.get_state()

            if state == actionlib.GoalStatus.SUCCEEDED:
                success_count += 1
                if success_count >= SUCCESS_THRESHOLD:
                    rospy.loginfo(f"[导航] ✅ 已到达 {goal_name}！（连续{success_count}次确认）")
                    return True
                continue
            else:
                success_count = 0

                if state in (actionlib.GoalStatus.ABORTED,
                             actionlib.GoalStatus.REJECTED,
                             actionlib.GoalStatus.LOST):
                    rospy.logwarn(f"[导航] ❌ 失败(状态{state}): {goal_name}")
                    break
                elif state == actionlib.GoalStatus.PREEMPTED:
                    # recovery中间态，继续等待
                    time.sleep(0.5)
            rate.sleep()

        # 最终检查
        try:
            final_state = client.get_state()
            if final_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"[导航] ✅ 最终检查: 已到达 {goal_name}！")

                # 精确场馆：过冲后需要回退到精确位置
                if need_precision:
                    rospy.loginfo(f"[导航] 执行精确回退: 后退{OVERSHOOT_DIST}m到目标点...")
                    time.sleep(0.5)
                    if relmove_client:
                        try:
                            # 分两步后退：每步5cm，更稳定
                            for step in range(2):
                                relmove_client.call(SetRelativeMoveRequest(
                                    goal=SetRelativeMoveRequest().goal.__class__(
                                        x=-OVERSHOOT_DIST/2, y=0, z=0, theta=0
                                    ),
                                    global_frame="odom"
                                ))
                                time.sleep(0.3)
                            rospy.loginfo(f"[导航] ✅ 已精确到达{goal_name}真实坐标({coords[0]}, {coords[1]})")
                        except Exception as e:
                            rospy.logwarn(f"[导航] 回退失败: {e}，但已过冲到达附近")

                return True
            client.cancel_goal()
            time.sleep(0.3)
        except Exception as e:
            rospy.logwarn(f"[导航] 异常: {e}")

        if attempt < max_retries:
            rospy.logwarn(f"[导航] 第{attempt+1}次失败，2秒后重试...")
            time.sleep(2)

    rospy.logerr(f"[导航] {max_retries+1}次尝试均失败: {goal_name}")
    return False


def do_task1():
    """任务一：迎宾引导 - 支持5个场馆（需语音确认用户想去哪个馆）"""
    rospy.loginfo("=" * 50)
    rospy.loginfo(">>> 开始执行任务一：迎宾引导 <<<")
    rospy.loginfo("=" * 50)

    audio = FACE_AUDIO["小侯"]

    # 1. 停止录音 → 播放唤醒音
    try:
        record_audio_client.call(SetBoolRequest(False))
    except:
        pass
    time.sleep(0.2)
    dur = playAudio(audio["awake"])
    time.sleep(dur + 0.3)  # 等音频播完

    # 2. 【等待用户语音确认】检测用户想去哪个场馆
    confirmed, venue_name = wait_for_voice_confirm(timeout_sec=60, detect_venue=True)

    if not confirmed:
        rospy.logwarn("[任务一] 未收到确认，再等一轮...")
        confirmed, venue_name = wait_for_voice_confirm(timeout_sec=60, detect_venue=True)

    if not confirmed:
        rospy.logerr("[任务一] 用户多次未确认，取消任务")
        return

    # 如果检测到具体场馆名则使用，否则默认深圳馆
    if venue_name is None:
        venue_name = "深圳馆"
        rospy.logwarn(f"[任务一] 未检测到具体场馆，默认前往 {venue_name}")

    rospy.loginfo(f"[任务一] 用户确认 → 前往 {venue_name}")

    # 3. 用户确认后，停止录音 → 播放欢迎语 → 等待播完 → 再播放场馆引导介绍
    try:
        record_audio_client.call(SetBoolRequest(False))
    except:
        pass
    time.sleep(0.2)
    dur = playAudio(audio["talk"])  # talk1.pcm 通用欢迎语（~2.2s）
    time.sleep(dur + 0.3)  # 确保talk1完全播完再播guide

    # 播放场馆引导音频 guide_{pinyin}.pcm（预留接口，用户后续补充pcm文件）
    venue_pinyin = PINYIN_MAP.get(venue_name, "shenzhen")
    guide_path = os.path.join(AUDIO_DIR, f"guide_{venue_pinyin}.pcm")
    if os.path.exists(guide_path):
        dur = playAudio(guide_path)
        time.sleep(dur + 0.3)  # 等guide音频播完
    else:
        rospy.logwarn(f"[任务一] guide_{venue_pinyin}.pcm 不存在，跳过引导（预留接口，请后续补充）")

    # 注意：导航期间不恢复录音！
    # 避免云端AIUI在导航过程中生成无关TTS应答（如"去厨房"）排队覆盖back_xxx.pcm
    # 深圳馆因guide_shenzhen.pcm时长14s足够让TTS过期，其他馆无此缓冲必须禁用

    # 4. 导航前往目标场馆（使用过冲+回退策略的精确模式）
    rospy.loginfo(f"[任务一] 开始导航前往 {venue_name}...")
    success = nav_to_goal(venue_name)

    # 5. 到达后播报 back_{pinyin}.pcm（预留接口）
    if success:
        try:
            record_audio_client.call(SetBoolRequest(False))
        except:
            pass
        time.sleep(0.2)
        back_path = os.path.join(AUDIO_DIR, f"back_{venue_pinyin}.pcm")
        if os.path.exists(back_path):
            dur = playAudio(back_path)
            time.sleep(dur + 0.3)  # 等back音频播完
        else:
            rospy.logwarn(f"[任务一] back_{venue_pinyin}.pcm 不存在，跳过到达播报（预留接口，请后续补充）")
        time.sleep(0.5)
        try:
            record_audio_client.call(SetBoolRequest(True))
        except:
            pass
    else:
        rospy.logwarn(f"[任务一] 导航到{venue_name}失败，尝试继续...")

    # 6. 返回原点
    rospy.loginfo("[任务一] 导航返回原点...")
    nav_to_goal("原点")
    time.sleep(1)

    rospy.loginfo(">>> 任务一完成！<<<")


def yolo_check_venue(venue_name):
    """
    到达场馆后，用YOLO检测火源和灭火器。
    返回: {"fire": bool, "extinguisher": bool}
    """
    result = {"fire": False, "extinguisher": False}

    try:
        # 获取摄像头图像
        rospy.loginfo(f"[巡检] {venue_name} - 获取图像进行YOLO检测...")
        image_msg = rospy.wait_for_message(CAMERA_TOPIC, Image, timeout=10.0)
        frame = _cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        # YOLO推理
        detections = _yolo_model(frame, conf=0.25, verbose=False)

        # 分析检测结果
        detected_classes = []
        for box in detections[0].boxes:
            cls_id = int(box.cls[0])
            cls_name = detections[0].names[cls_id]
            conf = float(box.conf[0])
            detected_classes.append(cls_name)
            rospy.loginfo(f"[巡检] {venue_name} - 检测到: {cls_name} (置信度: {conf:.2f})")

        if "fire" in detected_classes:
            result["fire"] = True
        if "fire_extinguisher" in detected_classes:
            result["extinguisher"] = True

    except Exception as e:
        rospy.logerr(f"[巡检] {venue_name} - YOLO检测异常: {e}")

    return result


def do_patrol_venue(venue_name):
    """巡检单个场馆：导航 → YOLO检测 → 异常播报"""
    rospy.loginfo(f"[巡检] ====== 开始检查: {venue_name} ======")

    # 1. 导航到场馆
    success = nav_to_goal(venue_name)
    if not success:
        rospy.logwarn(f"[巡检] 无法到达{venue_name}，跳过")
        return

    # 2. 到达后稍作停留，让摄像头稳定
    time.sleep(3)

    # 3. YOLO检测（多次检测提高准确率）
    all_results = {"fire": False, "extinguisher": False}
    for i in range(3):  # 检测3次取结果
        r = yolo_check_venue(venue_name)
        if r["fire"]:
            all_results["fire"] = True
        if r["extinguisher"]:
            all_results["extinguisher"] = True
        time.sleep(1)

    # 4. 根据检测结果播放异常音频
    venue_audio = ABNORMAL_AUDIO.get(venue_name, {})

    if all_results["fire"]:
        rospy.logwarn(f"[巡检] {venue_name} - 发现火源！播报警告音频")
        playAudio(venue_audio.get("fire", ""))
        time.sleep(2)

    if not all_results["extinguisher"]:
        rospy.logwarn(f"[巡检] {venue_name} - 未检测到灭火器！播报提示音频")
        playAudio(venue_audio.get("no_extinguisher", ""))
        time.sleep(2)

    if not all_results["fire"] and all_results["extinguisher"]:
        rospy.loginfo(f"[巡检] {venue_name} - 检查正常（无火源、有灭火器）")

    rospy.loginfo(f"[巡检] ====== {venue_name} 检查完成 ======")


def do_task2():
    """任务二：巡检模式（依次检查5个场馆）"""
    rospy.loginfo("=" * 50)
    rospy.loginfo(">>> 开始执行任务二：巡检模式 <<<")
    rospy.loginfo("=" * 50)

    audio = FACE_AUDIO["管理员-小戴"]

    # 1. 停止录音 → 播放唤醒音
    try:
        record_audio_client.call(SetBoolRequest(False))
    except:
        pass
    time.sleep(0.2)
    dur = playAudio(audio["awake"])
    time.sleep(dur + 0.3)  # 等音频播完

    # 2. 等待用户语音确认
    confirmed = wait_for_voice_confirm(timeout_sec=60, keywords=CONFIRM_KEYWORDS_TASK2)

    if not confirmed:
        rospy.logwarn("[任务二] 未收到确认，等待中...")
        confirmed = wait_for_voice_confirm(timeout_sec=60, keywords=CONFIRM_KEYWORDS_TASK2)

    # 3. 用户确认后，停止录音 → 播放进入巡检模式提示
    try:
        record_audio_client.call(SetBoolRequest(False))
    except:
        pass
    time.sleep(0.2)
    dur = playAudio(audio["talk"])  # talk2.pcm (~2.8s)
    rospy.loginfo(f"[任务二] 等待{dur:.1f}秒确保音频播放完成...")
    time.sleep(dur + 0.3)

    # 4. 发布巡检开始指令
    patrol_pub.publish(String(data="start_patrol"))
    rospy.loginfo("[任务二] 巡检指令已发布！开始逐个场馆检查...")

    # 5. 依次巡检5个场馆
    for venue_name in PATROL_ORDER:
        if rospy.is_shutdown():
            break
        do_patrol_venue(venue_name)
        time.sleep(1)  # 场馆间间隔

    # 6. 所有场馆巡检完成，前往充电桩充电
    rospy.loginfo("[任务二] 所有场馆检查完成，前往充电桩...")
    do_charge()

    rospy.loginfo(">>> 任务二（巡检+充电）全部完成！<<<")


def do_charge():
    """
    充电流程（优化版 - 分段导航提高成功率）：
      策略：先到安全中转点 → 再精确导航 → AR追踪精确定位

      1. 导航到充电桩附近安全位置 (0.2, 1.5) 避开costmap残留
      2. 尝试精确导航到充电桩坐标 (0.45, 1.9499) 带重试
      3. AR二维码追踪定位 (marker id=0, 距离0.4m)
      4. 相对移动靠近充电桩 (-0.18m)
      5. 等待充电完成 (2秒)
      6. 相对移动回退离开 (0.18m)
    """
    rospy.loginfo("=" * 50)
    rospy.loginfo(">>> 开始执行：充电流程 <<<")
    rospy.loginfo("=" * 50)

    try:
        # 第1段：先导航到充电桩附近的"安全中转点"（避开边缘costmap残留）
        safe_approach = [0.2, 1.5, 0.0, 1.0]

        rospy.loginfo("[充电] 第1步: 导航到充电桩附近安全位置...")
        clear_costmaps()

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("[充电] move_base 连接超时！")

        client.cancel_all_goals()
        time.sleep(0.5)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = safe_approach[0]
        goal.target_pose.pose.position.y = safe_approach[1]
        goal.target_pose.pose.orientation.z = safe_approach[2]
        goal.target_pose.pose.orientation.w = safe_approach[3]

        client.send_goal(goal)
        rate = rospy.Rate(5)
        start_time = time.time()

        while not rospy.is_shutdown() and (time.time() - start_time) < 60:
            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[充电] 已到达安全中转点")
                break
            elif state in (actionlib.GoalStatus.ABORTED,
                           actionlib.GoalStatus.PREEMPTED,
                           actionlib.GoalStatus.REJECTED,
                           actionlib.GoalStatus.LOST):
                rospy.logwarn("[充电] 中转点导航失败，尝试直接导航...")
                break
            rate.sleep()

        time.sleep(1)

        # 第2段：从中转点精确导航到充电桩坐标（此时距离近，更容易成功）
        rospy.loginfo("[充电] 第2步: 精确导航到充电桩位置...")
        clear_costmaps()

        charge_coords = NAV_POINTS["充电桩"]
        nav_success = False

        for attempt in range(3):  # 最多3次重试
            rospy.loginfo(f"[充电] 尝试{attempt+1}/3: ({charge_coords[0]}, {charge_coords[1]})")

            client.cancel_all_goals()
            time.sleep(0.3)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = charge_coords[0]
            goal.target_pose.pose.position.y = charge_coords[1]
            goal.target_pose.pose.orientation.z = charge_coords[2]
            goal.target_pose.pose.orientation.w = charge_coords[3]

            client.send_goal(goal)
            attempt_start = time.time()

            while not rospy.is_shutdown() and (time.time() - attempt_start) < 40:
                state = client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    nav_success = True
                    rospy.loginfo("[充电] ✅ 已到达充电桩！")
                    break
                elif state in (actionlib.GoalStatus.ABORTED,
                               actionlib.GoalStatus.PREEMPTED,
                               actionlib.GoalStatus.REJECTED,
                               actionlib.GoalStatus.LOST):
                    rospy.logwarn(f"[充电] 第{attempt+1}次失败，清除costmap后重试...")
                    clear_costmaps()
                    time.sleep(1)
                    break
                rate.sleep()

            if nav_success:
                break

        if not nav_success:
            rospy.logwarn("[充电] ⚠️ 精确导航未成功，但继续AR追踪...")

        time.sleep(1)

        # 3. AR二维码追踪定位
        # 关键：ar_track_alvar需要时间检测并发布marker到base_camera/ar_pose_marker话题
        # pose_adjust的GetMarkerPoseFromTopic()只等1秒，所以必须提前稳定
        rospy.loginfo("[充电] 第3步: 等待AR码检测稳定...")
        time.sleep(3)  # 等待3秒让ar_track_alvar充分检测

        ar_success = False
        # 尝试不同goal_dist：0.3/0.4/0.5（不同距离可能影响追踪成功率）
        goal_dists = [0.4, 0.3, 0.5]

        for ar_attempt in range(5):  # 最多尝试5次
            dist = goal_dists[ar_attempt % len(goal_dists)]
            rospy.loginfo(f"[充电] AR追踪尝试 {ar_attempt+1}/5 (goal_dist={dist}m)...")

            if ar_track(0, dist):
                ar_success = True
                break
            else:
                rospy.logwarn(f"[充电] 第{ar_attempt+1}次失败(goal_dist={dist})，调整位置后重试...")
                time.sleep(2)  # 等待2秒让检测结果更新

                # 微调位置帮助AR检测
                if relmove_client and ar_attempt < 4:
                    try:
                        # 偶数次前进一点，奇数次后退+旋转
                        if ar_attempt % 2 == 0:
                            relmove_client.call(SetRelativeMoveRequest(
                                goal=SetRelativeMoveRequest().goal.__class__(x=0.05, y=0, z=0, theta=0),
                                global_frame="odom"
                            ))
                        else:
                            relmove_client.call(SetRelativeMoveRequest(
                                goal=SetRelativeMoveRequest().goal.__class__(x=-0.03, y=0, z=0, theta=0.15),
                                global_frame="odom"
                            ))
                    except Exception as e:
                        rospy.logwarn(f"[充电] 微调失败: {e}")
                    time.sleep(1)

        if not ar_success:
            rospy.logerr("[充电] ⚠️ AR追踪5次均失败！")

        time.sleep(1)  # 等待AR追踪到位

        # 4. 相对移动靠近充电桩（分小步前进，避免一次移动太大导致失败）
        rospy.loginfo("[充电] 第4步: 靠近充电桩...")
        move_success = False
        for move_step in range(2):  # 分2步靠近：每步9cm
            if rel_move(-0.09, 0, 0):
                time.sleep(0.5)
                move_success = True
            else:
                rospy.logwarn(f"[充电] 靠近第{move_step+1}步失败")
                break

        if not move_success:
            rospy.logerr("[充电] 靠近充电桩完全失败，尝试回退...")
            rel_move(0.18, 0, 0)  # 尝试回退
            return False

        # 5. 等待充电
        rospy.loginfo("[充电] 第5步: 正在充电... (等待2秒)")
        time.sleep(2.0)

        # 6. 回退离开
        rospy.loginfo("[充电] 第6步: 充电完成，回退离开...")
        rel_move(0.18, 0, 0)

        rospy.loginfo("[充电] >>> 充电完成！<<<")
        return True

    except Exception as e:
        rospy.logerr(f"[充电] 异常: {e}")
        return False


def ar_track(ar_id, goal_dist):
    """AR二维码追踪服务（/track）"""
    try:
        rospy.loginfo(f"[AR追踪] 执行二次定位，id:{ar_id}，距离:{goal_dist}m")
        rospy.wait_for_service('/track', timeout=10)

        req = TrackRequest()
        req.ar_id = ar_id
        req.goal_dist = goal_dist

        response = track_client.call(req)
        if response.success:
            rospy.loginfo(f"[AR追踪] 成功: {response.message}")
            return True
        else:
            rospy.logerr(f"[AR追踪] 失败: {response.message}")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"[AR追踪] 服务调用失败: {e}")
        return False


def rel_move(x, y, theta):
    """相对移动服务（/relative_move）"""
    try:
        rospy.loginfo(f"[相对移动] x:{x}, y:{y}, theta:{theta}")
        rospy.wait_for_service('/relative_move', timeout=10)

        srv = SetRelativeMoveRequest()
        srv.goal.x = x
        srv.goal.y = y
        srv.goal.theta = theta
        srv.global_frame = "odom"

        response = relmove_client.call(srv)
        if response.success:
            rospy.loginfo(f"[相对移动] 成功: {response.message}")
            return True
        else:
            rospy.logerr(f"[相对移动] 失败: {response.message}")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"[相对移动] 服务调用失败: {e}")
        return False


def cleanup_and_exit():
    """清理并自动退出"""
    rospy.loginfo("")
    rospy.loginfo("=" * 50)
    rospy.loginfo("   全部任务已完成！程序即将自动退出...")
    rospy.loginfo("=" * 50)
    time.sleep(2)
    rospy.signal_shutdown("比赛全部完成，自动退出")
    sys.exit(0)


# ==================== 主循环 ====================

stage = rospy.get_param('~stage', STAGE_WAIT_VISITOR)
rate = rospy.Rate(2)  # 每秒检测2次

rospy.loginfo("")
rospy.loginfo("=" * 50)
rospy.loginfo("  比赛主流程控制器已启动")
rospy.loginfo("  当前阶段: %d", stage)
rospy.loginfo("  1=等待小侯 | 2=任务一中 | 3=等待小戴 | 4=结束")
rospy.loginfo("=" * 50)
rospy.loginfo("")

while not rospy.is_shutdown():
    stage = rospy.get_param('~stage', stage)

    if stage == STAGE_FINISHED:
        cleanup_and_exit()
        break

    if stage == STAGE_TASK1_RUNNING:
        rate.sleep()
        continue

    # ---- 人脸检测 ----
    labels = face_detect()

    if len(labels) > 0:
        for label in labels:
            role = ROLE_MAP.get(label, None)

            if stage == STAGE_WAIT_VISITOR and role == "参观者":
                rospy.loginfo(f"[检测] 识别到: {label} ({role})")
                # 切换阶段防止重复触发
                rospy.set_param('~stage', STAGE_TASK1_RUNNING)
                stage = STAGE_TASK1_RUNNING
                # 执行任务一（含语音确认）
                do_task1()
                # 任务一完成后切换到等待管理员
                rospy.set_param('~stage', STAGE_WAIT_ADMIN)
                stage = STAGE_WAIT_ADMIN
                rospy.loginfo(">>> 进入阶段3：等待管理员小戴 <<<")
                rospy.loginfo(">>> 或手动: rosparam set /face_detect_node/stage 3 <<<")

            elif stage == STAGE_WAIT_ADMIN and role == "管理员":
                rospy.loginfo(f"[检测] 识别到: {label} ({role})")
                rospy.set_param('~stage', STAGE_FINISHED)
                stage = STAGE_FINISHED
                # 执行任务二
                do_task2()
                # 全部完成 → 自动退出
                cleanup_and_exit()

            elif role is not None:
                rospy.loginfo(f"[检测] 识别到: {label}（当前阶段不需要）")

    rate.sleep()
