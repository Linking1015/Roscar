#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
radar_only_obstacle.py
纯毫米波雷达障碍物检测节点 —— 不依赖 YOLO，不做 TF，不做二次聚类。

策略（基于已知场地假设）:
  除建图时的静态障碍物外，只在 1.3m 附近可能有人出现。
  ARS408 已输出聚类目标，直接按 dist 过滤 → 状态机 → 发布。

滤波器链:
  a) 距离 + 角度过滤（保留原有逻辑）
  b) RCS 过滤：丢弃低雷达散射截面积的目标（噪声/鬼影）
  c) 空间聚类：将抖动散点汇聚成簇质心
  d) 时序持久性滤波：只发布连续多帧稳定存在的点

障碍物清除机制:
  costmap_2d::ObstacleLayer 的内部 costmap 只能通过 raytraceFreespace 清除，
  空 PointCloud2 无法触发 raytrace（没有点 → 没有射线）。
  因此本节点采用"检测到就发点，连续超时未检测到就调 clear_costmaps"策略:
    检测到障碍物 → 发布 PointCloud2
    障碍物消失 → 先停发点云，等 observation_persistence 让 buffer 过期
               → 再调 /move_base/clear_costmaps 重置内部 costmap

输出话题:
  /person_radar/obstacles  : PointCloud2  → costmap obstacle_layer 订阅
  /person_radar/markers    : MarkerArray  → RViz 可视化
"""

import rospy
import math
import time
from collections import defaultdict, deque

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


# ================================================================
# 辅助：空间聚类（欧氏距离阈值）
# ================================================================
def cluster_points(points, radius):
    """将三维点按欧氏距离聚类，返回每个簇的质心列表。

    points:   [(x, y, z, dist, ...), ...]
    radius:   聚类半径（m）

    返回: [(cx, cy, cz, mean_dist), ...]  每个簇的质心坐标和平均距离
    """
    if not points:
        return []

    unvisited = list(points)
    clusters = []

    while unvisited:
        cluster = [unvisited.pop(0)]
        changed = True
        while changed:
            changed = False
            remaining = []
            for u in unvisited:
                # 检查 u 是否属于当前簇
                ux, uy, uz = u[0], u[1], u[2]
                belongs = any(
                    math.hypot(ux - c[0], uy - c[1]) <= radius
                    for c in cluster
                )
                if belongs:
                    cluster.append(u)
                    changed = True
                else:
                    remaining.append(u)
            unvisited = remaining

        # 计算质心
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        cz = sum(p[2] for p in cluster) / len(cluster)
        mean_dist = sum(p[3] for p in cluster) / len(cluster)
        clusters.append((cx, cy, cz, mean_dist))

    return clusters


# ================================================================
# 辅助：持久性检查
# ================================================================
class PersistenceFilter:
    """时序持久性滤波器：只有最近 window 帧内至少出现 frames 次的点才通过。"""

    def __init__(self, persistence_frames, persistence_window, bin_size=0.2):
        self.frames = persistence_frames
        self.window = persistence_window
        self.bin_size = bin_size
        self._history = deque(maxlen=persistence_window)  # 每帧存一组 (xbin, ybin)

    def _bin(self, x, y):
        return (int(round(x / self.bin_size)), int(round(y / self.bin_size)))

    def feed(self, points):
        """输入一帧的点列表 [(x, y, z, dist), ...]，输出通过持久性检查的点列表。"""
        if not points:
            # 空帧也记录（障碍物消失）
            self._history.append(set())
            return []

        current_bins = set()
        for p in points:
            current_bins.add(self._bin(p[0], p[1]))

        # 增量更新：追加当前帧的 bin 集合
        self._history.append(current_bins)

        # 统计每个 bin 在最近 window 帧内的出现次数
        bin_counts = defaultdict(int)
        for frame_bins in self._history:
            for b in frame_bins:
                bin_counts[b] += 1

        # 只有出现次数 >= frames 的 bin 才发布
        passed = []
        for p in points:
            if bin_counts[self._bin(p[0], p[1])] >= self.frames:
                passed.append(p)

        return passed


class RadarOnlyObstacle:
    """纯雷达障碍物检测：按距离过滤 → RCS过滤 → 聚类 → 持久性滤波 → 发布。"""

    def __init__(self):
        rospy.init_node('radar_only_obstacle')

        # ===== 可配置参数：距离 =====
        self.target_distance = float(
            rospy.get_param("~target_distance", 1.3)) # type: ignore
        self.distance_tolerance = float(
            rospy.get_param("~distance_tolerance", 0.3)) # type: ignore

        # ===== 可配置参数：角度 =====
        self.max_angle_deg = float(
            rospy.get_param("~max_angle_deg", 90.0)) # type: ignore

        # ===== 可配置参数：RCS =====
        self.rcs_min = float(
            rospy.get_param("~rcs_min", -10.0)) # type: ignore

        # ===== 可配置参数：聚类 =====
        self.cluster_radius = float(
            rospy.get_param("~cluster_radius", 0.3)) # type: ignore

        # ===== 可配置参数：持久性 =====
        self.persistence_frames = int(
            rospy.get_param("~persistence_frames", 3)) # type: ignore
        self.persistence_window = int(
            rospy.get_param("~persistence_window", 5)) # type: ignore

        # ===== 可配置参数：清除 =====
        self.clear_timeout = float(
            rospy.get_param("~clear_timeout", 0.5)) # type: ignore
        self.clear_cooldown = float(
            rospy.get_param("~clear_cooldown", 1.0)) # type: ignore

        # ===== 话题 =====
        radar_topic = rospy.get_param("~radar_topic", "/radar/pointcloud")

        # ===== 发布者 =====
        self.obstacle_pub = rospy.Publisher(
            "/person_radar/obstacles", PointCloud2, queue_size=10)
        self.marker_pub = rospy.Publisher(
            "/person_radar/markers", MarkerArray, queue_size=10)

        # ===== 持久性滤波器 =====
        self._persistence = PersistenceFilter(
            self.persistence_frames,
            self.persistence_window,
            bin_size=0.2
        )

        # ===== 统计 =====
        self._radar_count = 0
        self._obstacle_count = 0
        self._clear_count = 0
        self._rcs_drop_count = 0
        self._cluster_count = 0
        self._persist_drop_count = 0
        self._start_time = time.time()

        # ===== 障碍物状态追踪 =====
        self._detected_now = False
        self._last_detected_time = time.time()
        self._last_clear_time = 0.0

        # ===== 订阅者 =====
        rospy.Subscriber(radar_topic, PointCloud2,
                         self._radar_callback, queue_size=10)

        # ===== clear_costmaps 服务（懒连接）=====
        self._clear_costmaps = rospy.ServiceProxy(
            '/move_base/clear_costmaps', Empty)
        self._clear_svc_ok = False

        rospy.loginfo(
            "[radar_only] started. "
            "target=%.1fm +/-%.1fm angle=%.1f deg "
            "rcs_min=%.1f cluster_radius=%.1fm "
            "persistence=%d/%d "
            "clear_timeout=%.2fs clear_cooldown=%.2fs",
            self.target_distance, self.distance_tolerance, self.max_angle_deg,
            self.rcs_min, self.cluster_radius,
            self.persistence_frames, self.persistence_window,
            self.clear_timeout, self.clear_cooldown)

        # ===== 定时器：检查是否需要清除（频率 = 10Hz） =====
        self._check_timer = rospy.Timer(
            rospy.Duration(0.1), self._clear_check_callback) # type: ignore

    # ================================================================
    # 回调
    # ================================================================
    def _radar_callback(self, radar_msg: PointCloud2):
        self._radar_count += 1

        min_dist = self.target_distance - self.distance_tolerance
        max_dist = self.target_distance + self.distance_tolerance
        max_angle_rad = math.radians(self.max_angle_deg)

        # ------------------------------------------------------------
        # 阶段 1: 距离 + 角度过滤
        # ------------------------------------------------------------
        raw_total = 0
        after_dist_angle = []

        for pt in pc2.read_points(radar_msg, skip_nans=True):
            raw_total += 1
            x, y, z = pt[0], pt[1], pt[2]
            dist = pt[4]  # ARS408 驱动已计算好 dist

            # 距离过滤
            if dist < min_dist or dist > max_dist:
                continue

            # 角度过滤
            angle = abs(math.atan2(y, x))
            if angle > max_angle_rad:
                continue

            # 同时提取 RCS（字段索引 5）
            rcs = pt[5] if len(pt) > 5 else 0.0
            after_dist_angle.append((x, y, z, dist, rcs))

        # ------------------------------------------------------------
        # 阶段 2: RCS 过滤
        # ------------------------------------------------------------
        after_rcs = []
        for p in after_dist_angle:
            if p[4] >= self.rcs_min:
                after_rcs.append(p)
            else:
                self._rcs_drop_count += 1

        # ------------------------------------------------------------
        # 阶段 3: 空间聚类
        # ------------------------------------------------------------
        if after_rcs:
            pre_cluster_count = len(after_rcs)
            clustered = cluster_points(after_rcs, self.cluster_radius)
            self._cluster_count += len(clustered)
            # 转换为统一格式：(x, y, z, dist)
            after_cluster = [
                (cx, cy, cz, mean_dist)
                for cx, cy, cz, mean_dist in clustered
            ]
            if len(after_cluster) < pre_cluster_count:
                pass  # 聚类有效减少了点数
        else:
            after_cluster = []

        # ------------------------------------------------------------
        # 阶段 4: 时序持久性滤波
        # ------------------------------------------------------------
        after_persist = self._persistence.feed(after_cluster)
        persist_dropped = len(after_cluster) - len(after_persist)
        self._persist_drop_count += persist_dropped

        # ------------------------------------------------------------
        # 状态追踪 + 发布
        # ------------------------------------------------------------
        if after_persist:
            self._detected_now = True
            self._last_detected_time = time.time()
            self._obstacle_count += 1
            self._publish_obstacles(after_persist, radar_msg.header)

            dists = ", ".join(f"{p[3]:.2f}m" for p in after_persist)
            rospy.loginfo_throttle(2.0,
                "[radar_only] #%d: raw=%d dist_angle=%d rcs=%d "
                "cluster=%d persist=%d dists=[%s]",
                self._radar_count, raw_total, len(after_dist_angle),
                len(after_rcs), len(after_persist), len(after_persist), dists)
        else:
            self._detected_now = False
            rospy.loginfo_throttle(2.0,
                "[radar_only] #%d: raw=%d dist_angle=%d rcs=%d "
                "cluster=%d persist=0 (no detection)",
                self._radar_count, raw_total, len(after_dist_angle),
                len(after_rcs), len(after_cluster))

    def _clear_check_callback(self, event):
        """定时检查是否需要调用 clear_costmaps。"""
        if self._detected_now:
            return

        now = time.time()
        since_detection = now - self._last_detected_time
        since_clear = now - self._last_clear_time

        if since_detection < self.clear_timeout:
            return

        if since_clear < self.clear_cooldown:
            return

        try:
            self._clear_costmaps()
            if not self._clear_svc_ok:
                self._clear_svc_ok = True
                rospy.loginfo("[radar_only] clear_costmaps service connected")
            self._last_clear_time = now
            self._clear_count += 1
            rospy.loginfo(
                "[radar_only] CLEAR costmaps #%d: "
                "no detection for %.2fs (timeout=%.2fs)",
                self._clear_count, since_detection, self.clear_timeout)
        except (rospy.ServiceException, rospy.ROSException):
            pass

    # ================================================================
    # 发布
    # ================================================================
    def _publish_obstacles(self, points: list, radar_header: Header):
        cloud_header = Header()
        cloud_header.stamp = rospy.Time.now()
        cloud_header.frame_id = "radar_link"

        cloud_pts = [[p[0], p[1], p[2]] for p in points]
        dist_list = [p[3] for p in points]

        cloud = pc2.create_cloud(cloud_header, [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ], cloud_pts)
        self.obstacle_pub.publish(cloud)

        # ---- MarkerArray ----
        now = cloud_header.stamp
        marker_array = MarkerArray()

        clear_m = Marker()
        clear_m.header.frame_id = "radar_link"
        clear_m.header.stamp = now
        clear_m.ns = "radar_obstacle"
        clear_m.action = Marker.DELETEALL
        marker_array.markers.append(clear_m)  # type: ignore

        for i, p in enumerate(points):
            m = Marker()
            m.header.frame_id = "radar_link"
            m.header.stamp = now
            m.ns = "radar_obstacle"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = p[0]
            m.pose.position.y = p[1]
            m.pose.position.z = p[2]
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.0
            m.color.a = 0.9
            m.lifetime = rospy.Duration(0.5) # type: ignore
            marker_array.markers.append(m)   # type: ignore

            # 文字标签
            t = Marker()
            t.header.frame_id = "radar_link"
            t.header.stamp = now
            t.ns = "radar_obstacle_label"
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = p[0]
            t.pose.position.y = p[1]
            t.pose.position.z = p[2] + 0.5
            t.text = f"OBST {p[3]:.1f}m"
            t.scale.z = 0.3
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 0.9
            t.lifetime = rospy.Duration(0.5) # type: ignore
            marker_array.markers.append(t)   # type: ignore

        self.marker_pub.publish(marker_array)

    def shutdown(self):
        elapsed = time.time() - self._start_time
        avg_hz = self._radar_count / elapsed if elapsed > 0 else 0
        rospy.loginfo(
            "[radar_only] Shutdown. "
            "Radars=%d Obstacles=%d Clears=%d "
            "RCS_drops=%d Clustered=%d Persist_drops=%d "
            "Runtime=%.1fs (%.1f Hz avg)",
            self._radar_count, self._obstacle_count, self._clear_count,
            self._rcs_drop_count, self._cluster_count, self._persist_drop_count,
            elapsed, avg_hz)


if __name__ == '__main__':
    try:
        node = RadarOnlyObstacle()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
