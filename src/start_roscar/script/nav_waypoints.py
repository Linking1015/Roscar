#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯定点导航脚本，支持角度控制，无红绿灯干扰"""

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def move_to_goal(x, y, yaw=0.0):
    """导航到目标点
    x, y  : 目标坐标（map 坐标系）
    yaw   : 目标朝向（弧度），0=朝右(X+)，1.57=朝上(Y+)，3.14=朝左，-1.57=朝下
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待 move_base 服务器...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # yaw → 四元数
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    rospy.loginfo(f"前往: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.0f}°")
    client.send_goal(goal)

    wait = client.wait_for_result(rospy.Duration(60))
    if not wait:
        rospy.logerr("超时！取消导航")
        client.cancel_goal()
        return False
    else:
        rospy.loginfo("到达目标点！")
        return True


if __name__ == '__main__':
    rospy.init_node('multi_point_nav_node')

    # ===== 在这里填入你的目标点 =====
    # 格式: (x, y, yaw)
    # yaw 是弧度：0=朝右, 1.57=朝上, 3.14=朝左, -1.57=朝下
    waypoints = [
        (5, -0.4,  0),         # 点1: 朝右
        (5, -2.5, -1.57),      # 点2: 朝上
        (2.6, -2.5,  3.14),     # 点3: 朝左
        (2.5,  0, 1.57),     # 点4: 朝下
        (0,   0,  0),         # 点5: 朝右（回到起点）
    ]
    # ================================

    rospy.loginfo(f"共 {len(waypoints)} 个目标点，开始导航...")
    for i, (x, y, yaw) in enumerate(waypoints):
        rospy.loginfo(f"--- 第 {i+1} 个目标点 ---")
        success = move_to_goal(x, y, yaw)
        if success:
            rospy.sleep(2)  # 到达后停顿2秒
        else:
            rospy.logwarn("跳过失败点，继续下一个...")

    rospy.loginfo("所有目标点处理完毕！")
