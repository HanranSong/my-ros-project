#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

WHEEL_BASE = 0.1  # Distance between wheels in meters
WHEEL_RADIUS = 0.0318  # Wheel radius in meters
ROTATION_SPEED = 0.4  # Rotation speed for the wheels
TARGET_ANGLE = -math.pi / 2  # 90 degrees in radians


class RotationTaskNode(DTROS):

    def __init__(self, node_name):
        super(RotationTaskNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC
        )
        vehicle_name = os.environ["VEHICLE_NAME"]

        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        encoder_left_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        encoder_right_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # Publisher for wheel commands
        self.pub = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Subscribers for wheel encoders
        self.sub_left = rospy.Subscriber(
            encoder_left_topic, WheelEncoderStamped, self.callback_left
        )
        self.sub_right = rospy.Subscriber(
            encoder_right_topic, WheelEncoderStamped, self.callback_right
        )

        # Variables to store encoder ticks and resolution
        self.ticks_left = None
        self.ticks_right = None
        self.res_left = None
        self.res_right = None

        rospy.loginfo("RotationTaskNode initialized.")

    def callback_left(self, msg):
        self.ticks_left = msg.data
        if self.res_left is None:
            self.res_left = msg.resolution
            rospy.loginfo_once(f"Left encoder resolution: {self.res_left}")

    def callback_right(self, msg):
        self.ticks_right = msg.data
        if self.res_right is None:
            self.res_right = msg.resolution
            rospy.loginfo_once(f"Right encoder resolution: {self.res_right}")

    def stop_robot(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub.publish(stop_msg)

    def rotate(self, target_angle, speed):
        init_left = self.ticks_left
        init_right = self.ticks_right

        rate = rospy.Rate(1000)
        # Determine the direction based on target_angle
        direction = 1 if target_angle >= 0 else -1
        cmd_vel_left = -direction * speed
        cmd_vel_right = direction * speed

        while not rospy.is_shutdown():
            # Compute tick differences (since ticks only increase)
            delta_left = abs(self.ticks_left - init_left)
            delta_right = abs(self.ticks_right - init_right)

            # Convert tick differences to distances
            dist_per_tick_left = (2 * math.pi * WHEEL_RADIUS) / self.res_left
            dist_per_tick_right = (2 * math.pi * WHEEL_RADIUS) / self.res_right

            # Impose the expected sign based on the wheel command:
            distance_left = (
                delta_left * dist_per_tick_left * (1 if cmd_vel_left >= 0 else -1)
            )
            distance_right = (
                delta_right * dist_per_tick_right * (1 if cmd_vel_right >= 0 else -1)
            )

            # Compute angular rotation: θ = (d_right - d_left) / wheel_base
            theta = (distance_right - distance_left) / WHEEL_BASE

            rospy.loginfo(f"Rotated angle: {theta:.4f} rad")

            # Check if the target rotation has been reached
            if (target_angle >= 0 and (theta + 0.4) >= target_angle) or (
                target_angle < 0 and (theta - 0.2) <= target_angle
            ):  # Manually compensate the overturn
                rospy.loginfo("Target rotation reached.")
                break

            # Publish wheel commands
            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = cmd_vel_left
            cmd_msg.vel_right = cmd_vel_right
            self.pub.publish(cmd_msg)

            rate.sleep()

        # Stop the robot after rotation
        self.stop_robot()
        rospy.sleep(1.0)

    def run(self):
        rate = rospy.Rate(1000)  # 500 Hz

        rospy.loginfo("Waiting for encoder messages...")
        while (
            self.ticks_left is None or self.ticks_right is None
        ) and not rospy.is_shutdown():
            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)

        rospy.loginfo("Rotating 90 degrees clockwise.")
        self.rotate(TARGET_ANGLE, ROTATION_SPEED)

        rospy.loginfo("Rotating back to original position.")
        self.rotate(-TARGET_ANGLE, ROTATION_SPEED)

        rospy.loginfo("Rotation task complete.")

    def on_shutdown(self):
        rospy.loginfo("Shutting down RotationTaskNode, stopping robot.")
        self.stop_robot()


if __name__ == "__main__":
    node = RotationTaskNode(node_name="rotation_task_node")
    node.run()
