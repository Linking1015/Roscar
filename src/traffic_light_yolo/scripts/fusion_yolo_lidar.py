#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import cv2
import time

from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from ultralytics import YOLO

import tf2_ros
import tf2_geometry_msgs


class FusionNode:
    def __init__(self):

        rospy.init_node('fusion_node')

        # ===== TF2 =====
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ===== 相机内参 =====
        self.fx = 400.0
        self.fy = 400.0
        self.cx = 320.0
        self.cy = 240.0

        # ===== YOLO 模型 =====
        model_path = rospy.get_param(
            "~model_path",
            "/home/gdut/catkin_roscar/src/traffic_light_yolo/best_new.pt"
        )
        rospy.loginfo("Loading YOLOv8 model: %s", model_path)
        self.model = YOLO(model_path) # type: ignore

        self.bridge = CvBridge()

        # ===== 最新数据缓存 =====
        self.latest_scan = None
        self.latest_scan_stamp = rospy.Time(0)

        # ===== 订阅 =====
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback,
                         queue_size=1, buff_size=2**24)

        # ===== 发布 =====
        # 距离（保留原有话题格式）
        self.front_pub = rospy.Publisher(
            '/traffic_light_distance', Float32MultiArray, queue_size=10)
        # 红绿灯状态：颜色 + 距离
        self.status_pub = rospy.Publisher(
            '/traffic_light_status', String, queue_size=10)
        # 带 YOLO 框的可视化图像
        self.image_pub = rospy.Publisher(
            '/yolo/result_image', Image, queue_size=1)

        # FPS 统计
        self.frame_count = 0
        self.start_time = time.time()

        # 日志限流：检测到目标时也每 2 秒才打印一次
        self.last_detect_log_time = 0.0

        rospy.loginfo("FusionNode started (Camera + LiDAR via TF2)")

    # =====================================
    # LiDAR 回调：仅缓存最新一帧
    # =====================================
    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_stamp = msg.header.stamp

    # =====================================
    # TF2：相机像素 → 激光坐标系方位角
    # =====================================
    def _camera_pixel_to_laser_angle(self, u, v, image_stamp):
        """像素 (u, v) -> 激光坐标系中的方位角 (rad)，失败返回 None"""

        # 像素 -> 归一化坐标
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        p_cv = np.array([x_norm, y_norm, 1.0])

        # OpenCV -> ROS camera 坐标系
        R_cv2ros = np.array([
            [0,  0,  1],
            [-1, 0,  0],
            [0, -1,  0]
        ])
        p_cam = R_cv2ros @ p_cv

        # 源帧用 "camera_link" -- robot_model_visualization.launch 中
        # 有 base_footprint->camera_link 的 static TF，TF 树可走通
        point_cam = PointStamped()
        point_cam.header.frame_id = "camera_link"
        point_cam.header.stamp = image_stamp
        point_cam.point.x = p_cam[0]
        point_cam.point.y = p_cam[1]
        point_cam.point.z = p_cam[2]

        try:
            # 目标帧 "laser" -- rplidar 的 frame_id
            point_laser = self.tf_buffer.transform(
                point_cam, "laser", timeout=rospy.Duration(0.2) # type: ignore
            )
            x = point_laser.point.x
            y = point_laser.point.y
            angle = math.atan2(y, x)
            return angle

        except (tf2_ros.LookupException, # type: ignore
                tf2_ros.ConnectivityException, # type: ignore
                tf2_ros.ExtrapolationException) as e: # type: ignore
            rospy.logwarn_throttle(5.0, "TF failed (camera_link->laser): %s", e)
            return None
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Unexpected TF error: %s", e)
            return None

    # =====================================
    # 在激光点云中查找距离
    # =====================================
    def _get_distance_at_angle(self, target_angle, angle_tolerance_deg=2.0):
        """在最新雷达帧中查找 target_angle(rad) 对应的最近距离"""
        if self.latest_scan is None:
            rospy.logwarn_throttle(5.0, "No LiDAR scan data yet")
            return None

        scan = self.latest_scan
        tolerance = math.radians(angle_tolerance_deg)
        candidates = []

        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < 0.2 or r > 10.0:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # 激光坐标系下的角度直接匹配
            if abs(angle - target_angle) < tolerance:
                candidates.append(r)

        if candidates:
            return min(candidates)
        return None

    # =====================================
    # 图像回调：YOLO 推理 + TF 变换 + 雷达查询
    # =====================================
    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s", e)
            return

        # ---- YOLO 推理 ----
        results = self.model(frame, verbose=False, device=0)

        # ---- 提取红绿灯检测框 ----
        label = "none"
        best_box = None
        found_red = False

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            name = results[0].names[cls_id]

            if name == "red":
                label = "red"
                found_red = True
                best_box = box.xyxy[0].cpu().numpy()
                break

            if name == "green" and not found_red:
                label = "green"
                best_box = box.xyxy[0].cpu().numpy()

        # ---- 生成可视化图像并发布 ----
        annotated_frame = results[0].plot()
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            img_msg.header = msg.header
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logwarn("Failed to publish annotated image: %s", e)

        # ---- TF2 变换 + 雷达距离查询（在 image_callback 中同步完成）----
        angle_deg = None
        dist_m = None

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            u_c = (x1 + x2) / 2.0
            v_c = (y1 + y2) / 2.0

            laser_angle = self._camera_pixel_to_laser_angle(
                u_c, v_c, msg.header.stamp)

            if laser_angle is not None:
                angle_deg = math.degrees(laser_angle)
                dist_m = self._get_distance_at_angle(laser_angle)

        # ---- 发布 ----
        angle_str = f"{angle_deg:.1f}" if angle_deg is not None else "N/A"
        dist_str = f"{dist_m:.2f}" if dist_m is not None else "N/A"

        # Float32MultiArray 距离
        dist_val = dist_m if dist_m is not None else float('inf')
        dist_msg = Float32MultiArray()
        dist_msg.data = [dist_val]
        self.front_pub.publish(dist_msg)

        # String 状态（供导航节点使用）
        status_msg = f"{label},{dist_str}"
        self.status_pub.publish(status_msg)

        # ---- FPS 统计 ----
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0.0

        # ---- 日志（检测到目标时隔 2 秒打印，未检测到也隔 2 秒）----
        now = time.time()
        if label != "none":
            if now - self.last_detect_log_time >= 2.0:
                rospy.loginfo("[%s] angle: %s deg | dist: %s m | FPS: %.1f",
                              label.upper(), angle_str, dist_str, fps)
                self.last_detect_log_time = now
        else:
            rospy.loginfo_throttle(2.0, "[NONE] FPS: %.1f", fps)

    def shutdown(self):
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0.0
        rospy.loginfo("FusionNode stopped. Frames: %d, Avg FPS: %.1f",
                      self.frame_count, fps)


if __name__ == '__main__':
    try:
        node = FusionNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
