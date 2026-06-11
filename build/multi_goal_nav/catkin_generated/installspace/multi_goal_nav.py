#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler


DEFAULT_INITIAL_POSE = {"x": -0.535, "y": -0.411, "yaw": 0.010}

DEFAULT_WAYPOINTS = [
    {"x": 3.521, "y": -0.262, "yaw": -0.005},
    {"x": 5.071, "y": -0.997, "yaw": -1.574},
    {"x": 5.060, "y": -2.307, "yaw": -1.591},#3
    {"x": 3.645, "y": -2.996, "yaw": -3.136},
    {"x": 2.490, "y": -2.598, "yaw": -3.131},
    {"x": 2.305, "y": -1.442, "yaw": 1.583},
    {"x": 1.686, "y": -0.583, "yaw": -3.140},
    {"x": -0.261, "y": -0.437, "yaw": 3.122},
]


def _yaw_to_quat(yaw: float) -> Quaternion:
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    q = Quaternion()
    q.x = qx
    q.y = qy
    q.z = qz
    q.w = qw
    return q


def _yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MultiGoalNavigator:
    def __init__(self):
        rospy.init_node("multi_goal_nav", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.server_name = rospy.get_param("~move_base_action", "move_base")
        self.wait_timeout = rospy.get_param("~wait_timeout", 60.0)
        self.goal_timeout = rospy.get_param("~goal_timeout", 300.0)
        self.loop = rospy.get_param("~loop", False)

        self.set_initial_pose = rospy.get_param("~set_initial_pose", True)
        self.initial_pose = rospy.get_param("~initial_pose", dict(DEFAULT_INITIAL_POSE))

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.light_topic = rospy.get_param("~light_topic", "/traffic_light/info")

        self.green_wait_goal_counts = rospy.get_param("~green_wait_goal_counts", {"2": 3, "4": 1})
        if not isinstance(self.green_wait_goal_counts, dict):
            self.green_wait_goal_counts = {"2": 3, "4": 1}

        self._light_color = "none"
        self._light_stamp = rospy.Time(0)
        self._green_consecutive_count = 0

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.light_sub = rospy.Subscriber(self.light_topic, String, self._light_cb, queue_size=1)
        self.print_timer = rospy.Timer(rospy.Duration(1.0), self._print_status)

        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.pose_timeout_sec = float(rospy.get_param("~pose_timeout_sec", 2.0))
        self.print_pose_srv = rospy.Service("~print_pose", Trigger, self._handle_print_pose)
        self.print_waypoint_srv = rospy.Service("~print_waypoint", Trigger, self._handle_print_waypoint)

        self.success_distance_tolerance = float(rospy.get_param("~success_distance_tolerance", 0.4))
        self.success_yaw_tolerance = float(rospy.get_param("~success_yaw_tolerance", 0.6))

        self.waypoints = rospy.get_param("~waypoints", [])
        if not isinstance(self.waypoints, list) or len(self.waypoints) == 0:
            self.waypoints = list(DEFAULT_WAYPOINTS)

        self.client = actionlib.SimpleActionClient(self.server_name, MoveBaseAction)
        rospy.loginfo("Waiting for action server /%s ...", self.server_name)
        if not self.client.wait_for_server(rospy.Duration(self.wait_timeout)):
            raise rospy.ROSInitException("等待 move_base action server 超时")

        if self.set_initial_pose:
            self._publish_initial_pose()

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("MultiGoalNavigator ready. waypoints=%d", len(self.waypoints))

    def _get_current_pose_once(self):
        msg = rospy.wait_for_message(self.pose_topic, PoseWithCovarianceStamped, timeout=self.pose_timeout_sec)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q)
        return p, q, yaw

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _is_success_reasonable(self, goal: MoveBaseGoal) -> bool:
        try:
            p, _q, yaw = self._get_current_pose_once()
        except Exception:
            return True

        gx = float(goal.target_pose.pose.position.x)
        gy = float(goal.target_pose.pose.position.y)
        gyaw = self._quat_to_yaw(goal.target_pose.pose.orientation)

        dx = float(p.x) - gx
        dy = float(p.y) - gy
        dist = math.hypot(dx, dy)
        dyaw = abs(self._normalize_angle(float(yaw) - float(gyaw)))

        if dist > self.success_distance_tolerance:
            rospy.logwarn(
                "Goal reported SUCCEEDED but pose far from target: dist=%.3f tol=%.3f (pose=%.3f,%.3f target=%.3f,%.3f)",
                dist,
                self.success_distance_tolerance,
                float(p.x),
                float(p.y),
                gx,
                gy,
            )
            return False

        if dyaw > self.success_yaw_tolerance:
            rospy.logwarn(
                "Goal reported SUCCEEDED but yaw far from target: dyaw=%.3f tol=%.3f (yaw=%.3f target=%.3f)",
                dyaw,
                self.success_yaw_tolerance,
                float(yaw),
                float(gyaw),
            )
            return False

        return True

    def _handle_print_pose(self, _req):
        try:
            p, q, yaw = self._get_current_pose_once()
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

        out = (
            "Setting goal: Frame:map, "
            f"Position({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"Orientation({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}) = Angle: {yaw:.3f}"
        )
        rospy.loginfo(out)
        return TriggerResponse(success=True, message=out)

    def _handle_print_waypoint(self, _req):
        try:
            p, _q, yaw = self._get_current_pose_once()
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

        out = f'{{"x": {p.x:.3f}, "y": {p.y:.3f}, "yaw": {yaw:.3f}}},'
        rospy.loginfo(out)
        return TriggerResponse(success=True, message=out)

    def _publish_initial_pose(self):
        try:
            x = float(self.initial_pose.get("x", DEFAULT_INITIAL_POSE["x"]))
            y = float(self.initial_pose.get("y", DEFAULT_INITIAL_POSE["y"]))
            yaw = float(self.initial_pose.get("yaw", DEFAULT_INITIAL_POSE["yaw"]))
        except Exception:
            x = float(DEFAULT_INITIAL_POSE["x"])
            y = float(DEFAULT_INITIAL_POSE["y"])
            yaw = float(DEFAULT_INITIAL_POSE["yaw"])

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _yaw_to_quat(yaw)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        for _ in range(3):
            msg.header.stamp = rospy.Time.now()
            self.initialpose_pub.publish(msg)
            rospy.sleep(0.2)

        rospy.loginfo("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, yaw, self.frame_id)

    def _on_shutdown(self):
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass

        try:
            self._publish_stop()
        except Exception:
            pass

    def _light_cb(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return

        parts = [p.strip() for p in text.replace(";", ",").replace(" ", ",").split(",") if p.strip()]
        if len(parts) == 0:
            return

        color = parts[0].lower()
        if color not in ("red", "green", "none"):
            color = "none"

        stamp = rospy.Time.now()
        if len(parts) >= 3:
            try:
                stamp = rospy.Time.from_sec(float(parts[2]))
            except Exception:
                stamp = rospy.Time.now()

        self._light_color = color
        self._light_stamp = stamp

        if color == "green":
            self._green_consecutive_count += 1
        else:
            self._green_consecutive_count = 0

    def _print_status(self, _evt):
        rospy.loginfo("light=%s green_consecutive=%d", self._light_color, self._green_consecutive_count)

    def _publish_stop(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def _build_goal(self, wp: dict) -> MoveBaseGoal:
        x = float(wp.get("x", 0.0))
        y = float(wp.get("y", 0.0))
        yaw = float(wp.get("yaw", 0.0))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = _yaw_to_quat(yaw)
        return goal

    def run(self):
        rate = rospy.Rate(5)
        idx = 0
        while not rospy.is_shutdown():
            if idx >= len(self.waypoints):
                if self.loop:
                    idx = 0
                else:
                    rospy.loginfo("All waypoints completed.")
                    self._publish_stop()
                    return

            wp = self.waypoints[idx]
            goal = self._build_goal(wp)

            rospy.loginfo(
                "Sending goal %d/%d: x=%.3f y=%.3f yaw=%.3f(rad)",
                idx + 1,
                len(self.waypoints),
                goal.target_pose.pose.position.x,
                goal.target_pose.pose.position.y,
                self._quat_to_yaw(goal.target_pose.pose.orientation),
            )

            self.client.send_goal(goal)

            start_time = rospy.Time.now()
            finished = False
            while not rospy.is_shutdown():
                if (rospy.Time.now() - start_time).to_sec() > self.goal_timeout:
                    break

                finished = self.client.wait_for_result(rospy.Duration(0.2))
                if finished:
                    break

                rate.sleep()

            if not finished:
                rospy.logwarn("Goal %d timeout, canceling...", idx + 1)
                self.client.cancel_goal()
                rate.sleep()
                continue

            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal %d reached.", idx + 1)

                if not self._is_success_reasonable(goal):
                    rospy.logwarn("Treating goal %d as false success; retrying...", idx + 1)
                    try:
                        self.client.cancel_goal()
                    except Exception:
                        pass
                    rate.sleep()
                    continue

                required_greens = 0
                try:
                    required_greens = int(self.green_wait_goal_counts.get(str(idx), 0))
                except Exception:
                    required_greens = 0

                if required_greens > 0:
                    rospy.loginfo(
                        "Waiting for %d consecutive green frames at goal %d...",
                        required_greens,
                        idx + 1,
                    )
                    self._wait_for_green_events(required_greens)
                    rospy.loginfo("Green condition satisfied at goal %d, continuing.", idx + 1)

                idx += 1
            else:
                rospy.logwarn("Goal %d failed with state=%s, retrying...", idx + 1, str(state))

            rate.sleep()

    def _wait_for_green_events(self, required_events: int):
        if required_events <= 0:
            return

        self._reset_green_event_counter()
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass
        rate = rospy.Rate(10)
        while (not rospy.is_shutdown()) and self._green_consecutive_count < required_events:
            self._publish_stop()
            rate.sleep()

    def _reset_green_event_counter(self):
        self._green_consecutive_count = 0

    @staticmethod
    def _quat_to_yaw(q: Quaternion) -> float:
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


if __name__ == "__main__":
    try:
        node = MultiGoalNavigator()
        service_only = rospy.get_param("~service_only", False)
        if service_only:
            rospy.loginfo("service_only=true: services ready, navigation loop will not run.")
            rospy.spin()
        else:
            node.run()
    except rospy.ROSInterruptException:
        pass
