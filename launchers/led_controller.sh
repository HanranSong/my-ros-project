#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun led_controller led_controller_node.py

# wait for app to end
dt-launchfile-join