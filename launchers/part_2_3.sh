#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun part_2 part_2_3_node.py

# wait for app to end
dt-launchfile-join