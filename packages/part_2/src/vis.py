#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
import math
import sys


def main():
    # bag_file = "./packages/part_2/src/straight.bag"
    bag_file = "./packages/part_2/src/rotation.bag"

    # Differential drive parameters (same as your motion code)
    WHEEL_RADIUS = 0.0318  # meters
    WHEEL_BASE = 0.1  # meters

    # Encoder topics (make sure these match your published topics)
    left_topic = "/csc22919/left_wheel_encoder_node/tick"
    right_topic = "/csc22919/right_wheel_encoder_node/tick"

    # Read encoder messages from the bag file
    left_msgs = []
    right_msgs = []

    bag = rosbag.Bag(bag_file, "r")
    for topic, msg, t in bag.read_messages(topics=[left_topic, right_topic]):
        if topic == left_topic:
            left_msgs.append((t.to_sec(), msg.data, msg.resolution))
        elif topic == right_topic:
            right_msgs.append((t.to_sec(), msg.data, msg.resolution))
    bag.close()

    # Use the smaller number of messages for pairing.
    n = min(len(left_msgs), len(right_msgs))
    if n < 2:
        print("Not enough encoder data to compute trajectory.")
        sys.exit(1)

    # Initialize pose: starting at (0,0) with heading 0
    x, y, theta = 0.0, 0.0, 0.0
    xs = [x]
    ys = [y]

    for i in range(1, n):
        # Compute dt from the left encoder timestamps (or average left/right if desired)
        t_prev, _, _ = left_msgs[i - 1]
        t_curr, _, _ = left_msgs[i]
        dt = t_curr - t_prev

        # Compute left wheel displacement:
        t_left_prev, tick_left_prev, res_left = left_msgs[i - 1]
        t_left, tick_left, _ = left_msgs[i]
        delta_tick_left = tick_left - tick_left_prev
        d_left = 2 * math.pi * WHEEL_RADIUS * (delta_tick_left / res_left)
        v_left = d_left / dt

        # Compute right wheel displacement:
        t_right_prev, tick_right_prev, res_right = right_msgs[i - 1]
        t_right, tick_right, _ = right_msgs[i]
        delta_tick_right = tick_right - tick_right_prev
        d_right = 2 * math.pi * WHEEL_RADIUS * (delta_tick_right / res_right)
        v_right = d_right / dt

        # Differential drive kinematics
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / WHEEL_BASE

        x += v * math.cos(theta + (omega * dt / 2.0)) * dt
        y += v * math.sin(theta + (omega * dt / 2.0)) * dt
        theta += omega * dt

        xs.append(x)
        ys.append(y)

    print(f"Final computed pose: x = {x:.3f} m, y = {y:.3f} m, theta = {theta:.3f} rad")

    # Plot the computed trajectory
    plt.figure()
    plt.plot(xs, ys, marker="o", linestyle="-", label="Trajectory")
    plt.xlabel("x (meters)")
    plt.ylabel("y (meters)")
    # plt.title("Straight")
    plt.title("Rotation")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
