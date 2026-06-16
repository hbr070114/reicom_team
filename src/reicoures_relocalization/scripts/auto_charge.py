#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from relative_move.srv import SetRelativeMove, SetRelativeMoveRequest
from ar_pose.srv import Track, TrackRequest

status_dict = {
    0: "等待中",
    1: "正在执行",
    2: "任务被取消",
    3: "导航成功",
    4: "导航失败",
    5: "目标被拒绝",
    6: "正在取消",
    7: "正在撤回",
    8: "已撤回",
    9: "连接丢失"
}

# 创建客户端
relmove_client = rospy.ServiceProxy('/relative_move', SetRelativeMove)
track_client = rospy.ServiceProxy('/track', Track)

def set_ARtrack(id, dist):
    try:
        rospy.loginfo(f"执行二次定位，id:{id}，dist:{dist}")
        rospy.wait_for_service('/track',timeout=10)
        # 构造请求
        req = TrackRequest()
        req.ar_id = id
        req.goal_dist = dist
        # 发送请求
        response = track_client.call(req)
        # 处理结果
        if response.success:
            rospy.loginfo(f"二次定位成功：{response.message}")
            return True
        else:
            rospy.logerr(f"二次定位失败：{response.message}")
            return False
        
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return False

def set_relmove(x,y,theta):  
    try:
        rospy.loginfo(f"执行相对移动控制，x:{x},y:{y},theta:{theta}")
        rospy.wait_for_service('/relative_move',timeout=10)
        # 构造请求
        srv = SetRelativeMoveRequest()
        srv.goal.x = x
        srv.goal.y = y
        srv.goal.theta = theta

        srv.global_frame = "odom"

        # 发送请求
        response = relmove_client.call(srv)

        # 处理结果
        if response.success:
            rospy.loginfo(f"移动成功：{response.message}")
            return True
        else:
            rospy.logerr(f"移动失败：{response.message}")
            return False

    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return False

def nav_to_goal(x, y, z, w):
    # 创建 move_base 客户端
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待连接 move_base 服务器...")
    # 等待服务器连接（最多等待60秒）
    client.wait_for_server(rospy.Duration(60))
    rospy.loginfo("连接成功！")
    
    # 取消所有之前的目标
    client.cancel_all_goals()
    rospy.logwarn("已清空所有导航任务！")

    # 构造导航目标
    goal = MoveBaseGoal()
    # 设置坐标系为 map
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # 设置目标朝向（四元数）
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    # 发送目标
    client.send_goal(goal)
    rospy.loginfo("发送导航目标...")

    # 循环监听导航状态（5Hz）
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # 获取当前状态
        state = client.get_state()
        state_str = status_dict.get(state, f"UNKNOWN({state})")
        rospy.loginfo(f"当前导航状态：[{state}] {state_str}")

        # 导航成功
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("导航成功：已到达目标点！")
            return True
        
        # 导航失败
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logerr("导航失败：无法到达目标！")
            return False
        
        # 任务被取消
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn("导航任务已被取消！")
            return False
        
        # 目标被拒绝
        elif state == actionlib.GoalStatus.REJECTED:
            rospy.logerr("导航目标被服务器拒绝！")
            return False
        
        # 连接丢失
        elif state == actionlib.GoalStatus.LOST:
            rospy.logerr("导航连接丢失！")
            return False
        rate.sleep()

    # ROS 节点关闭
    return False

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('relocalization_node', anonymous=False)
        # 第一步：导航到目标点
        if not nav_to_goal(0.45, 1.9499, -0.7, 0.7):
            exit()

        # 第二步：设置AR追踪
        if not set_ARtrack(0, 0.4):
            exit()

        # 第三步：执行相对移动
        if not set_relmove(-0.18, 0, 0):
            exit()

        # 延时2秒
        rospy.sleep(2.0)

        # 第四步：回退移动
        if not set_relmove(0.18, 0, 0):
            exit()

    except rospy.ROSInterruptException:
        rospy.logerr("程序被中断！")
