#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def publish_static_tf():
    rospy.init_node('sensor_tf_publisher')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []

    # base_link -> radar_link
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = 'base_link'
    t1.child_frame_id = 'radar_link'
    t1.transform.translation.x = 0.10   # 雷达前方10cm
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.30   # 雷达比摄像头高30cm
    t1.transform.rotation.w = 1.0
    transforms.append(t1)

    # base_link -> camera_link
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = 'base_link'
    t2.child_frame_id = 'camera_link'
    t2.transform.translation.x = 0.15   # 摄像头前方15cm
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.0
    t2.transform.rotation.w = 1.0
    transforms.append(t2)

    broadcaster.sendTransform(transforms)
    rospy.loginfo("TF发布完成: radar_link相对camera_link: x=-0.05m z=+0.30m")
    rospy.spin()

if __name__ == '__main__':
    publish_static_tf()
