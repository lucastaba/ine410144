#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_control_msgs.msg import AckermannLateralCommand
from autoware_auto_control_msgs.msg import LongitudinalCommand
from nav_msgs.msg import Odometry
import math

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')

        # Publisher to control the vehicle
        self.publisher_ = self.create_publisher(
            AckermannControlCommand,
            '/simulation/actuation/control_command',
            10
        )

        # Subscriber to get vehicle position and speed
        self.subscription = self.create_subscription(
            Odometry,
            '/simulation/sensor/odometry',
            self.odometry_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.max_speed = 27.78  # 100 km/h in m/s
        self.acceleration_command = 1.0  # m/s^2 acceleration command (adjust as needed)
        self.state = 'accelerating'
        self.current_speed = 0.0
        self.current_position = (0.0, 0.0, 0.0)

        self.get_logger().info("Vehicle control node started.")

    def odometry_callback(self, msg):
        # Compute current speed from velocity components
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx ** 2 + vy ** 2)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.current_position = (x, y, z)

    def timer_callback(self):
        msg = AckermannControlCommand()
        msg.stamp = self.get_clock().now().to_msg()

        # Lateral control (keep straight)
        steering_tire_angle_degree = 0
        lateral = AckermannLateralCommand()
        lateral.steering_tire_angle = steering_tire_angle_degree*math.pi/180
        lateral.steering_tire_rotation_rate = 0.0

        # Longitudinal control
        longitudinal = LongitudinalCommand()

        if self.state == 'accelerating':
            if self.current_speed < self.max_speed:
                longitudinal.acceleration = self.acceleration_command
                self.get_logger().info(
                    f"Accelerating My test: speed = {self.current_speed:.2f} m/s"
                )
            else:
                longitudinal.acceleration = -2.0  # Deceleration to stop
                self.state = 'stopping'
                self.get_logger().info("Reached 100 km/h, stopping vehicle.")

        elif self.state == 'stopping':
            # If speed is close to zero, stop the node
            if self.current_speed <= 0.1:
                longitudinal.acceleration = self.acceleration_command
                self.get_logger().info("Vehicle stopped. My test")
                self.state = 'accelerating'
                # self.destroy_node()
                # rclpy.shutdown()
                # return
            else:
                longitudinal.acceleration = -2.0
                self.get_logger().info(f"Stopping: speed = {self.current_speed:.2f} m/s")

        longitudinal.speed = 0.0  # We control with acceleration only
        longitudinal.jerk = 0.0

        msg.lateral = lateral
        msg.longitudinal = longitudinal
        self.publisher_.publish(msg)

        x, y, z = self.current_position
        self.get_logger().info(f"Current position -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
