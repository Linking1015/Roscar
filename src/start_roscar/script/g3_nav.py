#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from collections import deque

class SmartNavigator:
    def __init__(self, waypoints):
        rospy.init_node('smart_navigator')

        # 红绿灯状态
        self.current_light = "none"
        self.light_distance = 999.0
        self.light_history = deque(maxlen=5)  # 简单滤波

        # 订阅红绿灯话题
        rospy.Subscriber('/traffic_light_status', String, self.light_callback)

        # 连接 move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 动作服务器...")
        if not self.client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("无法连接到 move_base 服务器！")
            rospy.signal_shutdown("move_base 不可用")
            return
        rospy.loginfo("成功连接到 move_base 服务器！")

        self.waypoints = waypoints  # 目标点列表 [(x, y, z, w), ...]
        self.max_retries = 3         # 真正的导航失败（如死胡同卡死）最大重试次数

    def light_callback(self, msg):
        """解析红绿灯消息，格式 'color,distance'"""
        try:
            data = msg.data.split(',')
            if len(data) != 2:
                return
            color = data[0].strip()
            dist_str = data[1].strip()

            # 更新灯色历史（简单多数滤波）
            if color in ["red", "green", "none"]:
                self.light_history.append(color)
                if len(self.light_history) == self.light_history.maxlen:
                    self.current_light = max(set(self.light_history),
                                             key=self.light_history.count)
                else:
                    self.current_light = color

            # 更新距离
            if dist_str != "N/A":
                try:
                    self.light_distance = float(dist_str)
                except:
                    self.light_distance = 999.0
            else:
                self.light_distance = 999.0
        except Exception as e:
            rospy.logerr(f"解析红绿灯消息出错: {e}")

    def create_goal(self, x, y, z, w):
        """创建 move_base 目标点：修复四元数传递逻辑，必须同时包含 z 和 w"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # 补全了之前丢失的 z 分量
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        return goal

    def wait_for_green(self):
        """红灯停车等待绿灯，超时强制恢复"""
        rospy.logwarn("🛑 执行紧急刹车！等待绿灯...")
        self.client.cancel_goal()  # 取消当前导航，机器人停止
        rospy.sleep(0.5)  # 给予底层充分的刹车响应时间

        wait_start = rospy.Time.now()
        timeout = rospy.Duration(30.0) # type: ignore

        while not rospy.is_shutdown():
            # 必须明确看到 "green"，或者等待超过 30 秒超时才放行。
            if self.current_light == "green":
                rospy.loginfo("🟢 明确识别到绿灯，恢复行驶！")
                return True
                
            if rospy.Time.now() - wait_start > timeout:
                rospy.logwarn("⏰ 等待绿灯超时(30秒)，可能是识别故障，强制恢复行驶！")
                return True
                
            rospy.sleep(0.5)
        return False

    def get_result_text(self, state):
        status_map = {
            GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
            GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED", GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.RECALLED: "RECALLED", GoalStatus.LOST: "LOST"
        }
        return status_map.get(state, "UNKNOWN")

    def navigate_to_point(self, goal, index):
        """导航到单个目标点，监控红绿灯"""
        retries = 0
        while retries < self.max_retries and not rospy.is_shutdown():
            # 更新时间戳并发送目标
            goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(goal)
            rospy.loginfo(f"🚀 前往第 {index+1} 个目标点 (尝试 {retries+1}/{self.max_retries})")

            # 状态监控循环
            while not rospy.is_shutdown():
                state = self.client.get_state()
                state_str = self.get_result_text(state)

                # 到达目标
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"✅ 成功到达第 {index+1} 个目标点")
                    return True

                # 导航因物理卡死或规划失败被中止
                if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                    rospy.logerr(f"❌ 导航发生底盘级失败，状态: {state_str}")
                    break  # 跳出监控，真正消耗一次 retry 进入重试

                # ========== 修复后的红绿灯干预逻辑 ==========
                if self.current_light == "red":
                    # 使用 loginfo_throttle，每 2 秒打印一次，防止终端被疯狂刷屏
                    if self.light_distance < 999.0:
                        rospy.loginfo_throttle(2.0, f"👀 看到红灯，距离还有 {self.light_distance:.2f} 米...")
                    
                    # 设定 1.2 米触发刹车。抵消约 0.2 米的滑行惯性，使得最终稳稳停在 1.0 米左右。
                    if self.light_distance <= 1.2:
                        rospy.logwarn(f"🚦 前方 {self.light_distance:.2f}m 达到警戒距离！")
                        
                        # 进入死等绿灯逻辑
                        if self.wait_for_green():
                            # 看到绿灯后重新下发目标点，无缝恢复监控。
                            goal.target_pose.header.stamp = rospy.Time.now()
                            self.client.send_goal(goal)
                            rospy.loginfo("🚗 已重新下发目标点，继续当前旅程。")
                # ============================================

                rospy.sleep(0.1)  # 监控频率

            # 只有当遇到 ABORTED (死胡同卡死) 且 break 后，才会执行到这里，增加重试次数
            retries += 1
            if retries < self.max_retries:
                rospy.loginfo("准备重新规划路径... (等待2秒)")
                rospy.sleep(2.0) 

        rospy.logerr(f"⚠️ 第 {index+1} 个目标点彻底卡死，重试 {self.max_retries} 次失败，放弃并跳过该点")
        return False

    def run(self):
        if not self.waypoints:
            rospy.logerr("没有目标点！")
            return

        rospy.loginfo(f"🎯 共 {len(self.waypoints)} 个目标点，开始导航...")
        # 修正：现在枚举时解包 4 个变量 (x, y, z, w)
        for i, (x, y, z, w) in enumerate(self.waypoints):
            rospy.loginfo(f"--- 准备前往第 {i+1} 个目标点: ({x:.2f}, {y:.2f}) ---")
            goal = self.create_goal(x, y, z, w)
            success = self.navigate_to_point(goal, i)
            if not success:
                rospy.logwarn(f"跳过第 {i+1} 个点，继续下一个")
            rospy.sleep(1.0)  # 点间停顿

        rospy.loginfo("🏁 所有目标点处理完毕，巡航任务结束！")

if __name__ == '__main__':
    # ====== 多点闭环导航路线坐标 ======
    # 格式为: (x, y, z, w)
    # yaw → 四元数对照：
    #   yaw=0     → z=0.0,    w=1.0    (朝右/+x)
    #   yaw=1.57  → z≈0.707,  w≈0.707  (朝上/+y)
    #   yaw=3.14  → z≈1.0,    w≈0.0    (朝左/-x)
    #   yaw=-1.57 → z≈-0.707, w≈0.707  (朝下/-y)
    waypoints = [
        (4.8,  -0.7, -0.087, 0.996),    # 第 1 个点: 朝右/+x 补偿 -10deg (yaw=-0.17)
        (4.7, -3.0, -0.707, 0.707),  # 第 2 个点: 朝下/-y (yaw=-1.57)，从WP1驶入方向
        (2.2, -3.0,  1.0, 0.0),    # 第 3 个点: 朝左/-x (yaw=3.14)，从WP2驶入方向
        (2.5,  -0.2,  0.707, 0.707),  # 第 4 个点: 朝上/+y (yaw=1.57)，从WP3驶入方向
        (0.0,  0.0,  0.0, 1.0),    # 第 5 个点: 朝左/-x (yaw=3.14)，从WP4驶入方向 — 修复：z=1.0, w≈0.0
    ]
    # =========================================
    try:
        navigator = SmartNavigator(waypoints)
        navigator.run()
    except rospy.ROSInterruptException:
        pass