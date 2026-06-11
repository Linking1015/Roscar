#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

class SmartNav:
    def __init__(self, waypoints):
        rospy.init_node('smart_multi_point_nav')
        
        # --- 1. 红绿灯状态变量 ---
        self.current_light = "none"
        self.light_distance = 999.0
        
        # 订阅 YOLO 节点发出的红绿灯话题
        rospy.Subscriber('/traffic_light_status', String, self.light_callback)
        
        # --- 2. 连接导航服务器 ---
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("⏳ 等待 move_base 导航服务器启动...")
        self.client.wait_for_server()
        rospy.loginfo("✅ 已成功连接到 move_base 服务器！")
        
        self.waypoints = waypoints

    def light_callback(self, msg):
        # 解析 YOLO 发来的消息，格式例如 "red,1.5"
        data = msg.data.split(',')
        self.current_light = data[0]
        
        # 提取距离
        if len(data) > 1 and data[1] != "N/A":
            try:
                self.light_distance = float(data[1])
            except ValueError:
                self.light_distance = 999.0
        else:
            self.light_distance = 999.0

    def run(self):
        for i, wp in enumerate(self.waypoints):
            rospy.loginfo(f"🚀 起步！前往第 {i+1} 个目标点...")
            
            # 创建目标点
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = wp[0]
            goal.target_pose.pose.position.y = wp[1]
            goal.target_pose.pose.orientation.z = wp[2]
            goal.target_pose.pose.orientation.w = wp[3]

            # 发送目标点
            self.client.send_goal(goal)
            
            # --- 核心逻辑：在行驶中持续监控状态 ---
            while not rospy.is_shutdown():
                state = self.client.get_state()
                
                # 情况 A：顺利到达目标点
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"🎯 成功到达第 {i+1} 个目标点！原地休息 2 秒...")
                    rospy.sleep(2)
                    break # 跳出监控循环，准备去下一个点
                    
                # 情况 B：导航遇到死胡同失败了
                elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                    rospy.logerr(f"❌ 导航到第 {i+1} 个点失败（可能被障碍物彻底卡死了）！")
                    break
                    
                # --- 情况 C：红绿灯干预逻辑 ---
                # 触发条件：发现红灯，且红灯距离小车小于 1.1 米 (防止因为远处的红灯误停)
                if self.current_light == "red" and self.light_distance < 1.1:
                    rospy.logwarn(f"🛑 紧急刹车！前方 {self.light_distance:.2f} 米处有红灯！")
                    self.client.cancel_goal() # 立刻取消当前导航任务，小车会急刹车
                    
                    # 进入死循环，死死盯住红绿灯，直到变绿
                    while not rospy.is_shutdown():
                        if self.current_light == "green" or self.current_light == "none":
                            rospy.loginfo("🟢 绿灯亮起 (或红绿灯移出视野)，继续前进！")
                            # 重新更新时间戳，再次把刚才的目标点发给底层
                            goal.target_pose.header.stamp = rospy.Time.now()
                            self.client.send_goal(goal)
                            break # 跳出等待循环，回到行驶监控
                        
                        rospy.sleep(0.5) # 每半秒检查一次灯色
                        
                # 主循环刷新率
                rospy.sleep(0.1) 

        rospy.loginfo("🎉 太棒了！所有的巡航任务都已经完美结束！")

if __name__ == '__main__':
    # 👇 请把你刚才用遥控记录的 5 个点填进这里 👇
    my_5_points = [
        (4.081, -0.301, -0.070, 0.998), 
        (4.880, -2.761, -0.725, 0.689),  
        (2.391, -2.628, 0.999, 0.044), 
        (2.442, -1.640, 0.687, 0.727), 
        (0.321, -0.082, 1.000, 0.010)  
    ]
    
    try:
        nav = SmartNav(my_5_points)
        nav.run()
    except rospy.ROSInterruptException:
        pass