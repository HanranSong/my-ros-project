#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


WHEEL_RADIUS = 0.0318  # In meter

FORWARD_SPEED = 0.5
BACKWARD_SPEED = -0.5
TARGET_DISTANCE = 1.25


class StraightLineNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(StraightLineNode, self).__init__(
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

        # Variables to store the latest encoder tick counts and resolutions.
        self.ticks_left = None
        self.ticks_right = None
        self.res_left = None
        self.res_right = None

        rospy.loginfo("StraightLineNode initialized.")

    def callback_left(self, msg):
        self.ticks_left = msg.data
        # Save the resolution once (log it only once)
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

    def move(self, target_distance, speed):
        # Record the initial tick counts for both wheels.
        init_left = self.ticks_left
        init_right = self.ticks_right

        rate = rospy.Rate(10)  # 10 Hz command update rate

        while not rospy.is_shutdown():
            # Compute the absolute tick differences since starting this move.
            delta_left = abs(self.ticks_left - init_left)
            delta_right = abs(self.ticks_right - init_right)

            # Compute traveled distance for each wheel:
            # distance = wheel circumference * (ticks / resolution)
            distance_left = (2 * math.pi * WHEEL_RADIUS) * (delta_left / self.res_left)
            distance_right = (2 * math.pi * WHEEL_RADIUS) * (
                delta_right / self.res_right
            )
            # Average the two distances.
            traveled = (distance_left + distance_right) / 2.0

            rospy.loginfo(f"Traveled distance: {traveled:.2f} m")

            # Check if we have reached (or exceeded) the target distance.
            if traveled >= target_distance:
                rospy.loginfo("Target distance reached.")
                break

            # Publish wheel commands with the desired speed.
            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = speed
            cmd_msg.vel_right = speed
            self.pub.publish(cmd_msg)

            rate.sleep()

        # Stop the robot once the target distance has been reached.
        self.stop_robot()
        # Pause briefly before the next command phase.
        rospy.sleep(1.0)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        # Wait until encoder data is available.
        rospy.loginfo("Waiting for encoder messages...")
        while (
            self.ticks_left is None or self.ticks_right is None
        ) and not rospy.is_shutdown():
            rate.sleep()

        # Make sure the robot is stopped before starting.
        self.stop_robot()
        rospy.sleep(1.0)

        rospy.loginfo("Starting forward motion.")
        # Move forward for TARGET_DISTANCE using FORWARD_SPEED.
        self.move(TARGET_DISTANCE, FORWARD_SPEED)

        rospy.loginfo("Starting backward motion.")
        # Move backward for TARGET_DISTANCE using BACKWARD_SPEED.
        self.move(TARGET_DISTANCE, BACKWARD_SPEED)

        rospy.loginfo("Straight-line movement complete.")

    def on_shutdown(self):
        rospy.loginfo("Shutting down StraightLineNode, stopping robot.")
        self.stop_robot()


if __name__ == "__main__":
    # create the node
    node = StraightLineNode(node_name="straight_line_node")
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
