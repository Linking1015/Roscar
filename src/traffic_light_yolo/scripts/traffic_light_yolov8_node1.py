#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ["LD_PRELOAD"]="/usr/lib/aarch64-linux-gnu/libgomp.so.1"

import rospy
import cv2
import numpy as np
import time
import tf  # 新增：用于坐标变换
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan  # 新增：LaserScan用于接收雷达数据
from cv_bridge import CvBridge
from ultralytics import YOLO

class TrafficLightDetector:

    def __init__(self):

        rospy.init_node('traffic_light_yolov8_node')

        # 模型路径
        model_path = rospy.get_param("~model_path", "/home/gdut/catkin_roscar/src/traffic_light_yolo/best_new.pt")

        # 加载YOLOv8模型
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 新增：创建一个专门发布红绿灯状态的话题
        self.light_pub = rospy.Publisher('/traffic_light_status', String, queue_size=10)

        # --- 新增：雷达与相机参数初始化 ---
        self.laser_data = None
        self.tf_listener = tf.TransformListener()
        self.image_width = 640
        self.image_height = 480
        # 假设摄像头的水平视场角为60度 (如果实际角度偏差大，可以尝试改大如90.0)
        self.camera_fov_h = 60.0  
        self.pixel_per_degree = self.image_width / self.camera_fov_h

        # 订阅雷达数据
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # 订阅图像
        self.sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # txt文件
        self.file = open("traffic_light_result.txt", "w")

        # FPS统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started.")

    # --- 新增：接收雷达数据 ---
    def laser_callback(self, data):
        self.laser_data = data

    # --- 新增：通过角度去雷达点云里找距离 ---
    def get_distance_from_laser(self, camera_angle_deg):
        if self.laser_data is None:
            return None
        try:
            # 获取 usb_cam 相对于雷达的角度偏差
            (trans, rot) = self.tf_listener.lookupTransform(self.laser_data.header.frame_id, '/usb_cam', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            camera_yaw_in_laser = np.degrees(euler[2]) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            camera_yaw_in_laser = 0.0

        # 计算总偏差角度
        total_laser_angle_deg = camera_yaw_in_laser + camera_angle_deg
        
          # ================= 关键修复：雷达反向补偿 =================
        total_laser_angle_deg += 180.0

        while total_laser_angle_deg > 180: total_laser_angle_deg -= 360
        while total_laser_angle_deg < -180: total_laser_angle_deg += 360
        
        angle_rad = np.radians(total_laser_angle_deg)
        angle_index = int((angle_rad - self.laser_data.angle_min) / self.laser_data.angle_increment)
        
         
        # 提取有效距离
        if 0 <= angle_index < len(self.laser_data.ranges):
            dist = self.laser_data.ranges[angle_index]
            if 0.1 < dist < 10.0:  # 过滤掉太近或太远的噪点
                return dist
        return None

    # --- 修改：不仅返回标签，还返回识别框的坐标 ---
    def detect_light(self, results):
        label = "none"
        best_box = None

        if len(results[0].boxes) == 0:
            return label, best_box

        names = results[0].names
        boxes = results[0].boxes

        for i, cls in enumerate(boxes.cls):
            name = names[int(cls)]
            
            # 如果是红灯，立刻返回红灯及对应的框
            if name == "red":
                return "red", boxes.xyxy[i].cpu().numpy()
            
            # 如果是绿灯，先记录下来 (防止画面里同时有红绿灯时，红灯优先级更高)
            if name == "green":
                label = "green"
                best_box = boxes.xyxy[i].cpu().numpy()

        return label, best_box

    # --- 修改：核心回调逻辑 ---
    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # YOLO推理
        results = self.model(frame, verbose=False, device=0)

        # 判断红绿灯，并获取框的坐标
        result_label, box = self.detect_light(results)

        angle_str = "N/A"
        dist_str = "N/A"

        # 如果检测到了红绿灯，开始计算角度和距离
        if box is not None:
            x1, y1, x2, y2 = box
            u_c = (x1 + x2) / 2  # 框的中心X像素点
            
            # 将像素偏差转化为角度
            pixel_offset = u_c - (self.image_width / 2)
            angle_deg = pixel_offset / self.pixel_per_degree
            
            # 找雷达要距离
            distance_m = self.get_distance_from_laser(angle_deg)

            angle_str = f"{angle_deg:.1f}"
            if distance_m is not None:
                dist_str = f"{distance_m:.2f}"

        # 写入txt (增加保存角度和距离，格式为: 颜色,角度,距离)
        self.file.write(f"{result_label},{angle_str},{dist_str}\n")
        # 新增：把状态发布给导航节点 (格式：颜色,距离)
        self.light_pub.publish(f"{result_label},{dist_str}")    

        # 统计FPS
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        # 终端漂亮地打印结果
        if result_label != "none":
            rospy.loginfo(f"[{result_label.upper()}] 偏角: {angle_str}度 | 距离: {dist_str}米 | FPS: {fps:.1f}")
        else:
            # 没看到灯的时候，不打印距离和角度，防刷屏
            rospy.loginfo(f"[NONE] FPS: {fps:.1f}")

    def shutdown(self):

        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        print("\n==============================")
        print("Total frames:", self.frame_count)
        print("Average FPS:", fps)
        print("==============================")

        self.file.close()


if __name__ == '__main__':

    detector = TrafficLightDetector()
    rospy.on_shutdown(detector.shutdown)
    rospy.spin()