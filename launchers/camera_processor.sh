#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun part_1 camera_processor_node.py

# wait for app to end
dt-launchfile-join