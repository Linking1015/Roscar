#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class TrafficLightDetector:
    def __init__(self):
        rospy.init_node('traffic_light_yolov8_node')

        # 模型路径
        model_path = rospy.get_param("~model_path", "/home/gdut/catkin_roscar/src/traffic_light_yolo/best_new.pt")
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 发布红绿灯状态 (用于给导航节点发指令)
        self.light_pub = rospy.Publisher('/traffic_light_status', String, queue_size=10)
        
        # 发布带画框的识别图像 (用于网页或 rqt 查看)
        self.image_pub = rospy.Publisher('/yolo/result_image', Image, queue_size=1)

        # ================= 核心测距参数 =================
        self.image_width = 640.0
        
        # 【极其关键，需验证】：这是视觉测距的标定系数 K。
        # 它的物理意义是：当红绿灯正好在 1.0 米处时，YOLO框的高度是多少像素。
        # 我先暂定给了一个经验值 180.0。如果不准，请看下一步的校准指南进行修改！
        self.distance_constant_k = 90.0 
        # ===============================================

        # 仅订阅图像，彻底抛弃雷达
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback,
                                          queue_size=1, buff_size=2**24)

        # 记录文件（可选）
        self.file = open("traffic_light_result.txt", "w")

        # FPS统计
        self.frame_count = 0
        self.start_time = time.time()

        rospy.loginfo("Traffic light detection node started. Using Pure Vision Monocular Distance Estimation.")


    def detect_light(self, results):
        """返回 (颜色, 完整识别框的像素坐标 [x1, y1, x2, y2])"""
        if len(results[0].boxes) == 0:
            return None, None

        names = results[0].names
        boxes = results[0].boxes
        
        # 优先返回红灯
        for i, cls in enumerate(boxes.cls):
            name = names[int(cls)]
            if name == "red":
                box_coords = boxes.xyxy[i].cpu().numpy()
                return "red", box_coords
                
        # 其次绿灯
        for i, cls in enumerate(boxes.cls):
            name = names[int(cls)]
            if name == "green":
                box_coords = boxes.xyxy[i].cpu().numpy()
                return "green", box_coords
                
        return None, None

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # 推理
        results = self.model(frame, verbose=False, device=0) 
        
        # 提取 YOLO 画好框的图像并发布
        annotated_frame = results[0].plot()
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            pass 

        # 获取完整的边界框坐标
        color, box = self.detect_light(results)

        angle_str = "N/A"
        dist_str = "N/A"
        
        if color is not None and box is not None:
            x1, y1, x2, y2 = box
            
            # 1. 计算偏角 (这里改回用像素差近似，因为纯视觉不需要和雷达严格对齐，足够用了)
            u_center = (x1 + x2) / 2.0
            pixel_offset = u_center - (self.image_width / 2.0)
            angle_deg = pixel_offset / (self.image_width / 60.0) # 假设 60 度 FOV
            angle_str = f"{angle_deg:.1f}"

            # 2. 核心：纯视觉计算距离
            # 计算识别框在画面中占据的垂直像素高度
            pixel_height = y2 - y1 
            
            # 防止除以 0 的崩溃保护
            if pixel_height > 0:
                # 距离 = 常数 K / 像素高度
                distance = self.distance_constant_k / pixel_height
                dist_str = f"{distance:.2f}"

        # 发布状态给导航节点 (例如: "red,1.25")
        status_msg = f"{color if color else 'none'},{dist_str}"
        self.light_pub.publish(status_msg)

        # 写入 txt
        if not self.file.closed:
            self.file.write(f"{color if color else 'none'},{angle_str},{dist_str}\n")

        # 终端打印日志
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed
        rospy.loginfo(f"[{color.upper() if color else 'NONE'}] 偏角:{angle_str} 距离:{dist_str}  FPS:{fps:.1f}")

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
