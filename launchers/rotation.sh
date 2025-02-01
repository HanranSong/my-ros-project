#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun part_2 rotation_node.py

# wait for app to end
dt-launchfile-join