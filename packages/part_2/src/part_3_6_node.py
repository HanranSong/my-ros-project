#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from motions_node import Motions

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

    def run(self):
        # Wait for encoder messages before starting any motion.
        self.controller.wait_for_encoders()

        # rospy.loginfo("Driving straight for 1.25 meter...")
        # self.controller.move_straight(target_distance=1.25, speed=0.3)

        # rospy.loginfo("Driving straight for -1.25 meter (backward)...")
        # self.controller.move_straight(target_distance=1.25, speed=-0.3)

        rospy.loginfo("Driving curve...")
        self.controller.drive_curve(radius=0.1, velocity=0.4, angle_span=np.pi / 2)

        rospy.loginfo("Composite motion sequence complete.")
        # rospy.spin()

    def on_shutdown(self):
        rospy.loginfo("Shutting down CompositeMotionNode, stopping robot.")
        self.controller.stop_robot()


if __name__ == "__main__":
    node = CompositeMotionNode("composite_motion_node")
    node.run()
