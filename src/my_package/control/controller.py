#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
controller.py

Author: Mahboob Alam
Email: ma.mahboob2002@gmail.com
Date: 05 October 2025
Description: This ROS node controls individual wheels of the autocar.

Copyright (c) 2025 Mahboob Alam
All rights reserved.

License: MIT
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time
import tf
import tf2_ros
import geometry_msgs.msg

# Wheel parameters
WHEEL_RADIUS = 0.12
WHEEL_BASE = 0.94  # distance between front and rear axle
TRACK_WIDTH = 0.654  # distance between left and right wheels

# Wheel commands
wheel_velocities = {
    "front_left_wheel_joint": 0.0,
    "front_right_wheel_joint": 0.0,
    "rear_left_wheel_joint": 0.0,
    "rear_right_wheel_joint": 0.0
}

# Joint positions (for visualization in RViz)
wheel_positions = {
    "front_left_wheel_joint": 0.0,
    "front_right_wheel_joint": 0.0,
    "rear_left_wheel_joint": 0.0,
    "rear_right_wheel_joint": 0.0
}

# Robot pose
x = 0.0
y = 0.0
theta = 0.0

# Callback for Twist commands
def cmd_vel_callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z

    speed_scale = 0.3   # Reduce to 30% of actual commanded speed
    linear *= speed_scale
    angular *= speed_scale
    
    # Differential drive kinematics
    v_l = linear - angular * TRACK_WIDTH / 2.0
    v_r = linear + angular * TRACK_WIDTH / 2.0

    # Convert linear velocity to wheel angular velocity (rad/s)
    wheel_velocities["front_left_wheel_joint"] = v_l / WHEEL_RADIUS
    wheel_velocities["rear_left_wheel_joint"]  = v_l / WHEEL_RADIUS
    wheel_velocities["front_right_wheel_joint"] = v_r / WHEEL_RADIUS
    wheel_velocities["rear_right_wheel_joint"]  = v_r / WHEEL_RADIUS

def wheel_controller():
    global x, y, theta

    rospy.init_node("wheel_controller")

    # Subscribers
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    # Publishers for joint states (for RViz visualization)
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    # TF broadcaster
    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(50)  # 50Hz
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        # --- Update joint positions ---
        for joint, vel in wheel_velocities.items():
            delta = vel * dt
            delta = max(min(delta, 0.5), -0.5)  # limit per frame
            wheel_positions[joint] += delta

        # --- Publish joint states ---
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = list(wheel_positions.keys())
        js.position = list(wheel_positions.values())
        js.velocity = list(wheel_velocities.values())
        joint_pub.publish(js)

        # --- Compute odometry ---
        v_left = (wheel_velocities["front_left_wheel_joint"] + wheel_velocities["rear_left_wheel_joint"]) / 2.0 * WHEEL_RADIUS
        v_right = (wheel_velocities["front_right_wheel_joint"] + wheel_velocities["rear_right_wheel_joint"]) / 2.0 * WHEEL_RADIUS

        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / TRACK_WIDTH

        # Update pose
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += omega * dt

        # --- Publish TF ---
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        br.sendTransform(t)

        # Log info
        rospy.loginfo_throttle(1, f"Wheel velocities: {wheel_velocities} | Pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

        rate.sleep()

if __name__ == "__main__":
    try:
        wheel_controller()
    except rospy.ROSInterruptException:
        pass
