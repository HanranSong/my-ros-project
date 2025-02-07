#!/usr/bin/env python3

import math
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


class Motions:
    def __init__(self, vehicle_name):
        self.vehicle_name = vehicle_name

        # Define topics.
        self.wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.encoder_left_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self.encoder_right_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # Publisher for wheel commands.
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)

        # Variables to store encoder ticks and resolution.
        self.ticks_left = None
        self.ticks_right = None
        self.res_left = None
        self.res_right = None

        # Subscribers for wheel encoder messages.
        self.sub_left = rospy.Subscriber(
            self.encoder_left_topic, WheelEncoderStamped, self.callback_left
        )
        self.sub_right = rospy.Subscriber(
            self.encoder_right_topic, WheelEncoderStamped, self.callback_right
        )

        # Wheel parameters.
        self.WHEEL_RADIUS = 0.0318  # meters
        self.WHEEL_BASE = 0.1  # meters

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

    def wait_for_encoders(self):
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for encoder messages...")
        while (
            self.ticks_left is None or self.ticks_right is None
        ) and not rospy.is_shutdown():
            rate.sleep()

    def stop_robot(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub.publish(stop_msg)

    def move_straight(self, target_distance, speed):
        init_left = self.ticks_left
        init_right = self.ticks_right
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Compute the absolute tick differences.
            delta_left = abs(self.ticks_left - init_left)
            delta_right = abs(self.ticks_right - init_right)

            # Calculate distance traveled by each wheel.
            distance_left = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_left / self.res_left
            )
            distance_right = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_right / self.res_right
            )
            traveled = (distance_left + distance_right) / 2.0

            rospy.loginfo("Traveled distance: {:.4f} m".format(traveled))

            if traveled >= abs(target_distance):
                rospy.loginfo("Target distance reached.")
                break

            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = speed
            cmd_msg.vel_right = speed
            self.pub.publish(cmd_msg)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)

    def rotate_robot(self, target_angle, speed):
        init_left = self.ticks_left
        init_right = self.ticks_right
        rate = rospy.Rate(10)
        direction = 1 if target_angle >= 0 else -1
        cmd_vel_left = -direction * speed
        cmd_vel_right = direction * speed

        while not rospy.is_shutdown():
            delta_left = abs(self.ticks_left - init_left)
            delta_right = abs(self.ticks_right - init_right)
            dist_per_tick_left = (2 * math.pi * self.WHEEL_RADIUS) / self.res_left
            dist_per_tick_right = (2 * math.pi * self.WHEEL_RADIUS) / self.res_right

            distance_left = (
                delta_left * dist_per_tick_left * (1 if cmd_vel_left >= 0 else -1)
            )
            distance_right = (
                delta_right * dist_per_tick_right * (1 if cmd_vel_right >= 0 else -1)
            )

            theta = (distance_right - distance_left) / self.WHEEL_BASE
            rospy.loginfo("Rotated angle: {:.4f} rad".format(theta))

            if abs(theta) >= abs(target_angle):
                rospy.loginfo("Target rotation reached.")
                break

            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = cmd_vel_left
            cmd_msg.vel_right = cmd_vel_right
            self.pub.publish(cmd_msg)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)
