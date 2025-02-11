#!/usr/bin/env python3

import math
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        # Avoid division by zero.
        if dt <= 0:
            dt = 1e-3
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0


class Motions:
    def __init__(self, vehicle_name):
        self.vehicle_name = vehicle_name

        # Topics for wheel commands and encoders.
        self.wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.encoder_left_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self.encoder_right_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # Publisher for wheel commands.
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)

        # Encoder tick variables and resolution.
        self.ticks_left = None
        self.ticks_right = None
        self.res_left = None
        self.res_right = None

        # Subscribers for wheel encoders.
        self.sub_left = rospy.Subscriber(
            self.encoder_left_topic, WheelEncoderStamped, self.callback_left
        )
        self.sub_right = rospy.Subscriber(
            self.encoder_right_topic, WheelEncoderStamped, self.callback_right
        )

        # Wheel parameters.
        self.WHEEL_RADIUS = 0.0318  # meters
        self.WHEEL_BASE = 0.1  # meters

        # PID Controller for drive straight.
        self.pid_straight = PIDController(Kp=1, Ki=0, Kd=0)

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
        rate = rospy.Rate(30)
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

    #######################################################################
    # Move Straight (using PID control for lateral error)
    #######################################################################
    def move_straight(self, target_distance, speed):
        self.wait_for_encoders()

        # Reset PID controller before starting a new motion.
        self.pid_straight.reset()

        init_left = self.ticks_left
        init_right = self.ticks_right

        # Variables for incremental integration.
        prev_distance_left = 0.0
        prev_distance_right = 0.0

        # Pose variables (we use y for lateral error; x and theta are integrated but not controlled here).
        x, y, theta = 0.0, 0.0, 0.0

        rate = rospy.Rate(30)
        last_time = rospy.Time.now()
        cumulative_distance = 0.0

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            # Get the current tick differences.
            delta_left_ticks = self.ticks_left - init_left
            delta_right_ticks = self.ticks_right - init_right

            # Compute cumulative distances for each wheel.
            current_distance_left = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_left_ticks / self.res_left
            )
            current_distance_right = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_right_ticks / self.res_right
            )

            # Compute incremental distances.
            d_left = current_distance_left - prev_distance_left
            d_right = current_distance_right - prev_distance_right

            prev_distance_left = current_distance_left
            prev_distance_right = current_distance_right

            # Compute the center displacement and incremental rotation.
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.WHEEL_BASE

            # Update the pose.
            x += d_center * math.cos(theta)
            y += d_center * math.sin(theta)
            theta += d_theta

            cumulative_distance += abs(d_center)

            # Lateral error (desired y is zero).
            error_y = 0.0 - y

            # Compute PID correction.
            correction = self.pid_straight.compute(error_y, dt)
            # When driving backward, invert the correction.
            if speed < 0:
                correction = -correction

            # Set wheel speeds with lateral correction.
            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = speed - correction
            cmd_msg.vel_right = speed + correction
            self.pub.publish(cmd_msg)

            rospy.loginfo(
                f"Distance: {cumulative_distance:.3f} m, y: {y:.3f}, Error: {error_y:.3f}, Correction: {correction:.3f}"
            )

            if cumulative_distance >= abs(target_distance):
                rospy.loginfo("Target distance reached.")
                break

            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)

    #######################################################################
    # Rotate Robot (using normal open-loop control)
    #######################################################################
    def rotate_robot(self, target_angle, speed):
        self.wait_for_encoders()

        init_left = self.ticks_left
        init_right = self.ticks_right
        rate = rospy.Rate(30)
        direction = 1 if target_angle >= 0 else -1
        cmd_vel_left = -direction * speed
        cmd_vel_right = direction * speed

        while not rospy.is_shutdown():
            # Compute tick differences (absolute values).
            delta_left = abs(self.ticks_left - init_left)
            delta_right = abs(self.ticks_right - init_right)
            dist_per_tick_left = (2 * math.pi * self.WHEEL_RADIUS) / self.res_left
            dist_per_tick_right = (2 * math.pi * self.WHEEL_RADIUS) / self.res_right

            # Estimate the distance traveled by each wheel.
            distance_left = (
                delta_left * dist_per_tick_left * (1 if cmd_vel_left >= 0 else -1)
            )
            distance_right = (
                delta_right * dist_per_tick_right * (1 if cmd_vel_right >= 0 else -1)
            )

            # Compute the change in orientation.
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

    #######################################################################
    # Drive Curve (using normal open-loop control)
    #######################################################################
    def drive_curve(self, radius, velocity, angle_span):
        self.wait_for_encoders()

        # Calculate wheel speeds using differential-drive kinematics.
        vel_left = velocity * (radius + self.WHEEL_BASE / 2.0) / radius
        vel_right = velocity * (radius - self.WHEEL_BASE / 2.0) / radius

        # For a clockwise curve, the integrated rotation will be negative.
        target_angle = -abs(angle_span)

        init_left = self.ticks_left
        init_right = self.ticks_right

        # Variables for incremental integration.
        prev_distance_left = 0.0
        prev_distance_right = 0.0
        cumulative_angle = 0.0
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            delta_left_ticks = self.ticks_left - init_left
            delta_right_ticks = self.ticks_right - init_right

            current_distance_left = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_left_ticks / self.res_left
            )
            current_distance_right = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_right_ticks / self.res_right
            )

            d_left = current_distance_left - prev_distance_left
            d_right = current_distance_right - prev_distance_right

            prev_distance_left = current_distance_left
            prev_distance_right = current_distance_right

            # Incremental rotation.
            d_theta = (d_right - d_left) / self.WHEEL_BASE
            cumulative_angle += d_theta

            rospy.loginfo(
                "Cumulative angle: {:.3f} rad, Target: {:.3f} rad".format(
                    cumulative_angle, target_angle
                )
            )

            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = vel_left
            cmd_msg.vel_right = vel_right
            self.pub.publish(cmd_msg)

            if abs(cumulative_angle) >= abs(target_angle):
                rospy.loginfo("Curve target reached.")
                break

            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("motions_combined_node")
    # Get the vehicle name from ROS parameters or default to 'duckiebot'
    vehicle_name = rospy.get_param("~vehicle_name", "duckiebot")
    motions = Motions(vehicle_name)

    # Example usage:
    try:
        # Drive straight for 1 meter using PID control.
        motions.move_straight(target_distance=1.0, speed=0.5)
        rospy.sleep(1.0)

        # Rotate 90 degrees (pi/2 radians) with open-loop control.
        motions.rotate_robot(target_angle=math.pi / 2, speed=0.3)
        rospy.sleep(1.0)

        # Drive a curve with radius 0.5 m, velocity 0.4 m/s, spanning 90 degrees.
        motions.drive_curve(radius=0.5, velocity=0.4, angle_span=math.pi / 2)
    except rospy.ROSInterruptException:
        pass
