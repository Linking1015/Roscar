#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ["LD_PRELOAD"]="/usr/lib/aarch64-linux-gnu/libgomp.so.1"

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from ultralytics import YOLO

class TrafficLightDetector:

    def __init__(self):

        rospy.init_node('traffic_light_yolov8_node')

        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.light_topic = rospy.get_param("~light_topic", "/traffic_light/info")
        self.fov_deg = float(rospy.get_param("~fov_deg", 60.0))
        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))

        # 模型路径
        model_path = "/home/gdut/catkin_roscar/src/traffic_light_yolo/best.pt"

        # 加载YOLOv8模型
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        self.pub = rospy.Publisher(self.light_topic, String, queue_size=10)
        self._last_pub_time = rospy.Time(0)

        # 订阅图像
        self.sub = rospy.Subscriber(
            self.image_topic,
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

    def detect_light(self, results):

        label = "none"
        cx = None

        if len(results) == 0 or len(results[0].boxes) == 0:
            return label, cx

        names = results[0].names
        boxes = results[0].boxes

        best = {"red": None, "green": None}
        for i in range(len(boxes)):
            try:
                cls_id = int(boxes.cls[i])
                name = names[cls_id]
            except Exception:
                continue

            if name not in ("red", "green"):
                continue

            try:
                conf = float(boxes.conf[i]) if boxes.conf is not None else 0.0
            except Exception:
                conf = 0.0

            try:
                xyxy = boxes.xyxy[i].cpu().numpy().tolist()
            except Exception:
                try:
                    xyxy = boxes.xyxy[i].numpy().tolist()
                except Exception:
                    continue

            if len(xyxy) != 4:
                continue

            x1, y1, x2, y2 = xyxy
            cand = {"conf": conf, "x1": x1, "x2": x2}
            if (best[name] is None) or (conf > best[name]["conf"]):
                best[name] = cand

        chosen = None
        if best["red"] is not None:
            label = "red"
            chosen = best["red"]
        elif best["green"] is not None:
            label = "green"
            chosen = best["green"]

        if chosen is not None:
            cx = 0.5 * (float(chosen["x1"]) + float(chosen["x2"]))

        return label, cx

    def _compute_angle_rad(self, cx: float, image_width: int) -> float:
        if cx is None or image_width <= 0:
            return 0.0
        fov_rad = float(self.fov_deg) * math.pi / 180.0
        w2 = 0.5 * float(image_width)
        if w2 == 0.0:
            return 0.0
        return (float(cx) - w2) / w2 * (0.5 * fov_rad)

    def _maybe_publish(self, label: str, angle_rad: float):
        if self.publish_hz <= 0:
            return
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < (1.0 / self.publish_hz):
            return
        self._last_pub_time = now
        msg = String()
        msg.data = f"{label},{angle_rad:.6f},{now.to_sec():.3f}"
        self.pub.publish(msg)

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        # YOLO推理
        results = self.model(frame, verbose=False,device=0)

        # 判断红绿灯
        result_label, cx = self.detect_light(results)
        angle_rad = self._compute_angle_rad(cx, frame.shape[1])
        self._maybe_publish(result_label, angle_rad)

        # 写入txt
        self.file.write(result_label + "\n")

        # 统计FPS
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed

        rospy.loginfo("Result: %s   FPS: %.2f", result_label, fps)

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