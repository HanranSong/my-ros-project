#!/usr/bin/env python3
import os
import sys
# Add the directory where composite_node.py resides to sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from duckietown.dtros import DTROS, NodeType
from motions_node import Motions
import math
from led_controller.srv import SetLEDColor, SetLEDColorResponse

import numpy as np

class CompositeMotionNode(DTROS):
    def __init__(self, node_name):
        # Initialize this node using DTROS.
        super(CompositeMotionNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC
        )
        vehicle_name = os.environ["VEHICLE_NAME"]
        # Instantiate the helper motion controller.
        self.controller = Motions(vehicle_name)

        # Wait for LED service
        rospy.loginfo("[INIT] Waiting for LED service to become available...")
        try:
            rospy.wait_for_service("/led/set_color", timeout=5)  # Timeout to avoid blocking forever
            self.set_led_color = rospy.ServiceProxy("/led/set_color", SetLEDColor)
            rospy.loginfo("[INIT] LED service connected successfully!")
        except rospy.ROSException as e:
            rospy.logerr("[ERROR] LED service not available: %s", e)
            self.set_led_color = None  # Prevent crashes if LED service is not found

    def call_led(self, color):
        """ Helper function to change LED color using the LED service """
        if self.set_led_color is None:
            rospy.logwarn("[LED] Skipping LED change to '%s' because the LED service is unavailable!", color)
            return
        
        rospy.loginfo("[LED] Attempting to change LED color to: %s", color)
        try:
            response = self.set_led_color(color)
            rospy.loginfo("[LED] Successfully changed LED color: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr("[LED ERROR] LED service call failed: %s", e)

    def run(self):
        """ Main control loop """
        rospy.loginfo("[RUN] Waiting for encoder messages before starting any motion...")
        self.controller.wait_for_encoders()
        rospy.loginfo("[RUN] Encoder messages received, starting motion sequence.")

        # --- State 1: Stop ---
        rospy.loginfo("[STATE 1] Stopping robot for 5 seconds...")
        self.call_led("blue")  
        rospy.sleep(5)

        # --- State 2: Tracing the 'D' Path ---
        rospy.loginfo("[STATE 2] Moving forward 1.2 meters...")
        self.call_led("green")
        self.controller.move_straight(target_distance=0.9 * 1.2, speed=0.35)

        self.controller.rotate_robot(target_angle=-0.9 * (math.pi / 2), speed=0.4)

        self.controller.move_straight(target_distance=0.9 * 0.92, speed=0.35)

        self.controller.drive_curve(
            radius=0.08, velocity=0.4, angle_span=0.9 * np.pi / 2
        )

        self.controller.move_straight(target_distance=1 * 0.61, speed=0.35)

        self.controller.drive_curve(
            radius=0.08, velocity=0.4, angle_span=0.8 * np.pi / 2
        )

        self.controller.move_straight(target_distance=0.9 * 0.92, speed=0.35)

        self.controller.rotate_robot(target_angle=-math.pi / 2, speed=0.4)

        rospy.loginfo("[RUN] Composite motion sequence complete.")

        # --- State 3: Return to Starting Position ---
        rospy.loginfo("[STATE 3] Returning to start position, waiting 5 seconds...")
        self.call_led("blue")  
        rospy.sleep(5)

    def on_shutdown(self):
        rospy.loginfo("[SHUTDOWN] Stopping robot...")
        self.controller.stop_robot()

if __name__ == "__main__":
    rospy.loginfo("[MAIN] Starting Composite Motion Node...")
    node = CompositeMotionNode("composite_motion_node")
    node.run()
    rospy.spin()  # Keep node alive
