#!/usr/bin/env python3
"""Estimates the pose of the Duckiebot from the wheel encoders (dead reckoning).

Subscribers:
    left_wheel_encoder_driver_node/tick  (duckietown_msgs/WheelEncoderStamped)
    right_wheel_encoder_driver_node/tick (duckietown_msgs/WheelEncoderStamped)
Publishers:
    ~/pose (nav_msgs/Odometry): the estimated pose, frame ``map``
Services:
    ~/reset (std_srvs/Trigger): restart the estimate from the origin
Parameters:
    publish_rate (float, Hz): rate of ~/pose
"""
from threading import Lock
from typing import Optional

import numpy as np
import rclpy
from duckietown_msgs.msg import WheelEncoderStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from lx_ros2_tools.params import LiveParams
from lx_ros2_tools.robot import get_wheel_params

from encoder_pose.odometry import delta_phi, estimate_pose


class EncoderPoseNode(Node):
    """
    Computes an estimate of the Duckiebot pose using the wheel encoders.
    """

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float

    def __init__(self):
        super().__init__("encoder_pose_node")
        self.get_logger().info("Initializing...")
        # get the name of the robot
        self.veh = self.get_namespace().strip("/")

        self.right_wheel_mutex = Lock()
        self.left_wheel_mutex = Lock()

        # parameters (defaults also in config/encoder_pose_node/default.yaml)
        self.params = LiveParams(self, {"publish_rate": 2.0}, on_change=self._on_param)

        # Init the parameters
        self.reset_parameters()

        # wheel radius and baseline from the robot calibration (DB21 defaults otherwise)
        wheels = get_wheel_params(self.veh)
        self.R = wheels.radius  # meters, wheel radius
        self.baseline = wheels.baseline  # meters, distance between the wheels
        self.get_logger().info(f"wheel radius {self.R} m, baseline {self.baseline} m ({wheels.source})")

        # Wheel encoder subscribers:
        self.create_subscription(
            WheelEncoderStamped, "left_wheel_encoder_driver_node/tick", self.cb_left_encoder, qos_profile_sensor_data
        )
        self.create_subscription(
            WheelEncoderStamped, "right_wheel_encoder_driver_node/tick", self.cb_right_encoder, qos_profile_sensor_data
        )

        # reset service (the ROS 1 node reset on the `activity_name` / `PID_parameters` topics)
        self.create_service(Trigger, "~/reset", self.srv_reset)

        # Odometry publisher
        self.db_estimated_pose = self.create_publisher(Odometry, "~/pose", 10)
        self._timer = self.create_timer(1.0 / float(self.params.publish_rate), self.pose_publisher)
        self.get_logger().info("Initialized.")

    def _on_param(self, name, value):
        if name == "publish_rate" and value > 0:
            self._timer.cancel()
            self._timer = self.create_timer(1.0 / float(value), self.pose_publisher)

    def reset_parameters(self):
        self.get_logger().info("Encoder pose resetting")
        self.delta_phi_left = 0.0
        self.left_tick_prev = None

        self.delta_phi_right = 0.0
        self.right_tick_prev = None

        # Initializing the odometry
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0

    def srv_reset(self, _req, res):
        """Restart the pose estimate from the origin."""
        with self.left_wheel_mutex, self.right_wheel_mutex:
            self.reset_parameters()
        res.success, res.message = True, "pose reset to the origin"
        return res

    def cb_left_encoder(self, encoder_msg: WheelEncoderStamped):
        """
        Wheel encoder callback
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        with self.left_wheel_mutex:
            # initializing ticks to stored absolute value
            if self.left_tick_prev is None:
                self.left_tick_prev = encoder_msg.data
                return

            left_ticks_curr = encoder_msg.data
            # running the delta_phi() function copied from the notebooks to calculate rotations
            delta_phi_left = delta_phi(left_ticks_curr, self.left_tick_prev, encoder_msg.resolution)
            if delta_phi_left == 0:
                return
            self.left_tick_prev = left_ticks_curr
            self.delta_phi_left += delta_phi_left

    def cb_right_encoder(self, encoder_msg: WheelEncoderStamped):
        """
        Wheel encoder callback, the rotation of the wheel.
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        with self.right_wheel_mutex:
            if self.right_tick_prev is None:
                self.right_tick_prev = encoder_msg.data
                return

            right_ticks_curr = encoder_msg.data

            # calculate rotation of right wheel
            delta_phi_right = delta_phi(right_ticks_curr, self.right_tick_prev, encoder_msg.resolution)
            if delta_phi_right == 0:
                return
            self.right_tick_prev = right_ticks_curr
            self.delta_phi_right += delta_phi_right

    def pose_publisher(self):
        """
        Publish the pose of the Duckiebot given by the kinematic model
            using the encoders.
        Publish:
            ~/pose (:obj:`Odometry`): Duckiebot pose.
        """
        with self.left_wheel_mutex, self.right_wheel_mutex:
            x_curr, y_curr, theta_curr = estimate_pose(
                self.R,
                self.baseline,
                self.x_prev,
                self.y_prev,
                self.theta_prev,
                self.delta_phi_left,
                self.delta_phi_right,
            )

            theta_curr = self.angle_clamp(theta_curr)  # angle always between 0,2pi

            # Calculate new odometry only when new data from encoders arrives
            self.delta_phi_left = 0.0
            self.delta_phi_right = 0.0

            # Current estimate becomes previous estimate at next iteration
            self.x_prev = x_curr
            self.y_prev = y_curr
            self.theta_prev = theta_curr

        # Creating message to plot the pose (TrajectoryPlot in the notebooks)
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = self.get_clock().now().to_msg()

        odom.pose.pose.position.x = float(x_curr)  # x position - estimate
        odom.pose.pose.position.y = float(y_curr)  # y position - estimate
        odom.pose.pose.position.z = 0.0  # z position - no flying allowed in Duckietown

        # these are quaternions!
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(np.sin(theta_curr / 2))
        odom.pose.pose.orientation.w = float(np.cos(theta_curr / 2))

        self.db_estimated_pose.publish(odom)

    @staticmethod
    def angle_clamp(theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta


def main(args=None):
    rclpy.init(args=args)
    node = EncoderPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
