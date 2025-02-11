#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# Launch LED controller node in the background
rosrun led_controller led_controller_node.py &

# Launch composite node
rosrun part_2 composite_node.py &

# wait for app to end
dt-launchfile-join