#!/usr/bin/env python3
# coding=utf-8
import rospy
import math
from can_msgs.msg import Frame
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

CLUSTER_STATUS  = 0x600
CLUSTER_GENERAL = 0x701

class ARS408Node:
    def __init__(self):
        rospy.init_node('ars408_node')
        self.clusters = {}
        self.num_of_cluster = 0
        self.cluster_counter = 0

        self.pub_markers = rospy.Publisher('/radar/targets', MarkerArray, queue_size=10)
        self.pub_raw     = rospy.Publisher('/radar/raw', Float32MultiArray, queue_size=10)
        # 新增：带时间戳的点云话题，用于时间同步
        self.pub_cloud   = rospy.Publisher('/radar/pointcloud', PointCloud2, queue_size=10)
        rospy.Subscriber('/received_messages', Frame, self.can_callback)
        rospy.loginfo("ARS408节点启动(cluster模式+时间戳)...")

    def can_callback(self, msg):
        if msg.id == CLUSTER_STATUS and len(msg.data) >= 2:
            if self.clusters:
                self.publish()
                self.clusters = {}
            self.num_of_cluster = min(msg.data[0] + msg.data[1], 250)
            self.cluster_counter = 0
        elif msg.id == CLUSTER_GENERAL and len(msg.data) >= 8:
            self.parse_cluster_general(msg.data)

    def parse_cluster_general(self, data):
        cid       = data[0]
        dist_long = (((data[1] << 5) | (data[2] >> 3)) * 0.2) - 500.0
        dist_lat  = ((((data[2] & 0x03) << 8) | data[3]) * 0.2) - 102.3
        vrel_long = (((data[4] << 2) | (data[5] >> 6)) * 0.25) - 128.0
        vrel_lat  = ((((data[5] & 0x3F) << 3) | (data[6] >> 5)) * 0.25) - 64.0
        rcs       = data[7] * 0.5 - 64.0
        dist      = math.sqrt(dist_long**2 + dist_lat**2)
        self.clusters[cid] = {
            'x': dist_long, 'y': dist_lat,
            'vx': vrel_long, 'vy': vrel_lat,
            'rcs': rcs, 'dist': dist
        }
        self.cluster_counter += 1
        if self.cluster_counter >= self.num_of_cluster > 0:
            self.publish()
            self.clusters = {}
            self.cluster_counter = 0

    def publish(self):
        now = rospy.Time.now()
        marker_array = MarkerArray()
        raw_data     = Float32MultiArray()
        points       = []

        for cid, obj in self.clusters.items():
            if obj['dist'] < 0.5 or obj['dist'] > 200:
                continue

            # MarkerArray（RViz可视化）
            m = Marker()
            m.header.frame_id = "radar_link"
            m.header.stamp    = now
            m.ns = "radar"; m.id = int(cid)
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = obj['x']
            m.pose.position.y = obj['y']
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.r = 1.0; m.color.g = 0.3; m.color.b = 0.0; m.color.a = 0.9
            m.lifetime = rospy.Duration(0.2) # type: ignore
            marker_array.markers.append(m)   # type: ignore

            # Float32MultiArray（融合节点用）
            raw_data.data.extend([   # type: ignore
                float(cid), obj['x'], obj['y'],
                obj['vx'], obj['vy'], obj['dist'], obj['rcs']
            ])

            # PointCloud2（带时间戳，用于message_filters同步）
            points.append([obj['x'], obj['y'], 0.5,
                           obj['vx'], obj['dist'], obj['rcs']])

        self.pub_markers.publish(marker_array)
        if raw_data.data:
            self.pub_raw.publish(raw_data)

        # 发布带时间戳点云
        header = std_msgs.msg.Header()
        header.stamp    = now
        header.frame_id = "radar_link"
        fields = [
            PointField('x',    0,  PointField.FLOAT32, 1),
            PointField('y',    4,  PointField.FLOAT32, 1),
            PointField('z',    8,  PointField.FLOAT32, 1),
            PointField('vx',   12, PointField.FLOAT32, 1),
            PointField('dist', 16, PointField.FLOAT32, 1),
            PointField('rcs',  20, PointField.FLOAT32, 1),
        ]
        cloud = pc2.create_cloud(header, fields, points)
        self.pub_cloud.publish(cloud)

if __name__ == '__main__':
    ARS408Node()
    rospy.spin()
