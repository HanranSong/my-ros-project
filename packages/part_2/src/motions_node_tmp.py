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
        """Reset the PID controller state."""
        self.prev_error = 0.0
        self.integral = 0.0


class Motions:
    def __init__(self, vehicle_name):
        self.vehicle_name = vehicle_name

        # Topics for wheel commands and encoders.
        self.wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.encoder_left_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self.encoder_right_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # Publisher and Subscribers.
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)
        self.sub_left = rospy.Subscriber(
            self.encoder_left_topic, WheelEncoderStamped, self.callback_left
        )
        self.sub_right = rospy.Subscriber(
            self.encoder_right_topic, WheelEncoderStamped, self.callback_right
        )

        # Encoder tick variables and resolution.
        self.ticks_left = None
        self.ticks_right = None
        self.res_left = None
        self.res_right = None

        # Wheel parameters.
        self.WHEEL_RADIUS = 0.0318  # meters
        self.WHEEL_BASE = 0.1  # meters

        # PID Controllers.
        # For straight driving: Using tuning values derived from Ku and Pu.
        self.pid_straight = PIDController(Kp=2, Ki=0, Kd=0)  # 1.75, 8
        self.pid_rotation = PIDController(Kp=1, Ki=0, Kd=0.3)  # Tune as needed.

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
        rate = rospy.Rate(10)
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

    def move_straight(self, target_distance, speed):
        """
        Drives the robot in a straight line using PID to correct lateral errors.
        When driving backward (speed < 0), the PID correction is inverted.
        """
        self.wait_for_encoders()

        # Reset the PID controller state before starting a new motion.
        self.pid_straight.reset()

        init_left = self.ticks_left
        init_right = self.ticks_right

        # Initialize previous cumulative distances for incremental integration.
        prev_distance_left = 0.0
        prev_distance_right = 0.0

        # Pose variables (we are mainly interested in lateral error, y).
        x, y, theta = 0.0, 0.0, 0.0

        rate = rospy.Rate(10)
        last_time = rospy.Time.now()
        cumulative_distance = 0.0

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            # Get the signed tick differences.
            delta_left_ticks = self.ticks_left - init_left
            delta_right_ticks = self.ticks_right - init_right

            # Compute cumulative distances for each wheel.
            current_distance_left = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_left_ticks / self.res_left
            )
            current_distance_right = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_right_ticks / self.res_right
            )

            # Compute the incremental distances since the last iteration.
            d_left = current_distance_left - prev_distance_left
            d_right = current_distance_right - prev_distance_right

            # Update previous distances.
            prev_distance_left = current_distance_left
            prev_distance_right = current_distance_right

            # Compute the incremental center displacement and rotation.
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.WHEEL_BASE

            # Update robot pose.
            x += d_center * math.cos(theta)
            y += d_center * math.sin(theta)
            theta += d_theta

            # Accumulate the traveled distance.
            cumulative_distance += abs(d_center)

            # Lateral error: assume desired y = 0.
            error_y = 0.0 - y

            # Compute PID correction.
            correction = self.pid_straight.compute(error_y, dt)

            # When driving backward, invert the correction.
            if speed < 0:
                correction = -correction

            # Set wheel speeds with correction.
            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = speed - correction
            cmd_msg.vel_right = speed + correction
            self.pub.publish(cmd_msg)

            rospy.loginfo(
                f"Distance: {cumulative_distance:.3f} m, y: {y:.3f}, Error: {error_y:.3f}, Correction: {correction:.3f}"
            )

            # Check if the target distance has been reached.
            if cumulative_distance >= abs(target_distance):
                rospy.loginfo("Target distance reached.")
                break

            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)

    def rotate_robot(self, target_angle, speed):
        """
        Rotates the robot by a target angle (in radians) using PID control.
        Positive target_angle rotates counterclockwise.
        """
        self.wait_for_encoders()

        # Optionally reset rotation PID here if needed:
        self.pid_rotation.reset()

        init_left = self.ticks_left
        init_right = self.ticks_right

        # Initialize previous cumulative distances.
        prev_distance_left = 0.0
        prev_distance_right = 0.0

        current_angle = 0.0  # Integrated rotation angle.
        rate = rospy.Rate(10)
        last_time = rospy.Time.now()

        # Determine rotation direction.
        direction = 1 if target_angle >= 0 else -1

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            # Compute tick differences (signed).
            delta_left_ticks = self.ticks_left - init_left
            delta_right_ticks = self.ticks_right - init_right

            # Compute cumulative wheel distances.
            current_distance_left = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_left_ticks / self.res_left
            )
            current_distance_right = (2 * math.pi * self.WHEEL_RADIUS) * (
                delta_right_ticks / self.res_right
            )

            # Compute incremental distances.
            d_left = current_distance_left - prev_distance_left
            d_right = current_distance_right - prev_distance_right

            # Update previous distances.
            prev_distance_left = current_distance_left
            prev_distance_right = current_distance_right

            # Incrementally update the rotation.
            d_theta = (d_right - d_left) / self.WHEEL_BASE
            current_angle += d_theta

            # Compute the rotation error.
            error = target_angle - current_angle

            # Compute PID correction.
            correction = self.pid_rotation.compute(error, dt)

            # Adjust wheel speeds for rotation.
            cmd_msg = WheelsCmdStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.vel_left = (-direction * speed) - correction
            cmd_msg.vel_right = (direction * speed) + correction
            self.pub.publish(cmd_msg)

            rospy.loginfo(
                f"Angle: {current_angle:.3f} rad, Error: {error:.3f}, Correction: {correction:.3f}"
            )

            if abs(error) < 0.05:  # Tolerance in radians (~3 degrees).
                rospy.loginfo("Rotation target reached.")
                break

            rate.sleep()

        self.stop_robot()
        rospy.sleep(1.0)
