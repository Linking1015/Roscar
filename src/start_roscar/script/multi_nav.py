#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x, y, w):
    # 这里的 'move_base' 名字，精准对应了你 navigation.launch 文件中的 <node pkg="move_base" name="move_base" ...>
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待 move_base 动作服务器...")
    client.wait_for_server()
    rospy.loginfo("成功连接到 move_base 服务器！")

    # 创建目标点消息
    goal = MoveBaseGoal()
    # "map" 对应 navigation.launch 里 AMCL 定位使用的全局坐标系
    goal.target_pose.header.frame_id = "map"  
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标坐标（这里只给平面平移 x, y。w=1.0 代表朝向默认正前方，不考虑车头旋转角度）
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w 

    rospy.loginfo(f"正在前往目标点: x={x}, y={y}")
    client.send_goal(goal)

    # 等待到达目标（设置60秒超时时间，防止小车卡在死胡同里出不来）
    wait = client.wait_for_result(rospy.Duration(60))
    if not wait:
        rospy.logerr("导航超时，未能到达目标！正在取消任务...")
        client.cancel_goal()
        return False
    else:
        rospy.loginfo("成功到达目标点！")
        return True

if __name__ == '__main__':
    try:
        rospy.init_node('multi_point_nav_node')
        
        # 【极其重要：必须替换】
        # 这里缺失的是你的场地地图实际坐标，必须用 /clicked_point 查出来填进去！
        # 提取的四个真实物理坐标点 (x, y, w)
        waypoints = [
            (4.714,  0.285, -0.711,  0.703),  # 第 1 个点
            (4.998, -2.043, -0.795,  0.607),  # 第 2 个点：已向右侧强行补偿约 15 度的偏角误差
            (2.575, -2.489,  1.000,  0.001),  # 第 3 个点
            (2.710, -1.522,  0.825,  0.565),  # 第 4 个点
            (1.906,  0.010,  1.000,  0.001),  # 第 5 个点
            (0.020,  -0.284, -1.000,  0.002)   # 第 6 个点：回到起点
        ]
        

        rospy.loginfo("开始执行多点导航任务...")
        for i, point in enumerate(waypoints):
            rospy.loginfo(f"--- 准备前往第 {i+1} 个目标点 ---")
            success = move_to_goal(point[0], point[1], point[2])
            
            if success:
                rospy.loginfo("停顿 2 秒后前往下一个点...")
                rospy.sleep(2) # 到达后停顿2秒
            else:
                rospy.logwarn("由于上一个点导航失败，已终止后续导航序列。")
                break
                
        rospy.loginfo("多点导航任务全部结束。")
                
    except rospy.ROSInterruptException:
        pass