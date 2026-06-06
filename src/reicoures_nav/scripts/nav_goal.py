#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def nav_to_goal():
    # 1. 初始化ROS节点
    rospy.init_node('move_base_action_client', anonymous=True)

    # 2. 创建Action客户端
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待连接 move_base 服务器...")

    # 3. 等待服务器连接成功
    client.wait_for_server()
    rospy.loginfo("连接 move_base 服务器成功！")

    # 4. 构造导航目标消息 MoveBaseGoal
    goal = MoveBaseGoal()

    # 设置坐标系为 map
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标坐标 (自行修改 x, y)
    goal.target_pose.pose.position.x = 2.543
    goal.target_pose.pose.position.y = 2.148

    # 设置目标朝向（四元数，朝前 w=1）
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1.0

    # 5. 发送导航目标
    rospy.loginfo("发送导航目标点...")
    client.send_goal(goal)

    # 6. 循环监听导航状态（模拟Feedback监听）
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # 获取当前状态
        state = client.get_state()
        rospy.loginfo(f"当前导航状态码: {state}")

        # 7. 判断导航状态
        if state == 1:
            rospy.loginfo("状态：正在导航中...")
        elif state == 3:
            rospy.loginfo("✅ 导航成功：已到达目标点！")
            break
        elif state == 4:
            rospy.logerr("❌ 导航失败：机器人被卡住或无法到达！")
            break
        
        rate.sleep()

    rospy.loginfo("导航任务结束！")

if __name__ == '__main__':
    try:
        nav_to_goal()
    except rospy.ROSInterruptException:
        rospy.logerr("程序异常退出！")