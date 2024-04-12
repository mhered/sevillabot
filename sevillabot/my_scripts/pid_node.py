#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sevillabot.msg import TimestampedData
from geometry_msgs.msg import Twist

class LineFollowerController(Node):
    def __init__(self):
        super().__init__('line_follower_controller')
        self.subscription = self.create_subscription(
            TimestampedData,
            '/line_follower_error',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_follower', 10)
        self.subscription  # prevent unused variable warning

        
        # Declare parameters
        self.declare_parameter('Kp', 20.0) # Proportional gain
        self.declare_parameter('Ki', 0.0) # Integral gain
        self.declare_parameter('Kd', 2.0) # Derivative gain
        self.declare_parameter('linear_vel', 0.1)  # linear velocity
        
        # Load PID constants and linear velocity from parameters
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.linear_vel = self.get_parameter('linear_vel').get_parameter_value().double_value


        # Log the parameters
        self.get_logger().info(
            f'Using Parameters:\n'
            f'  Kp = {self.Kp}\n'
            f'  Ki = {self.Ki}\n'
            f'  Kd = {self.Kd}\n'
            f'  Linear Velocity = {self.linear_vel}'
)
        # PID terms initialization
        self.integral = 0
        self.last_error = 0
        self.last_time = 0

 

    def listener_callback(self, msg):
        angular_z_vel = self.calculate_angular_z_vel(msg)
        self.publish_velocity(angular_z_vel)

    def calculate_angular_z_vel(self, msg):

        # Convert timestamp to seconds
        time_in_s = msg.timestamp / 1000

        # Current time in s and error
        dt = (time_in_s - self.last_time) if self.last_time else 0

        # Error in m
        error = (3500 - msg.data) * 9.525 / 1000000

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        D = self.Kd * ((error - self.last_error) / dt) if dt > 0 else 0

        # Compute output
        angular_z_vel = P + I + D

        # Update for next iteration
        self.last_error = error
        self.last_time = time_in_s

        return angular_z_vel

    def publish_velocity(self, angular_z_vel):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.angular.z = angular_z_vel
        self.publisher.publish(twist_msg)
        self.get_logger().info(f'Publishing Velocity: Linear={twist_msg.linear.x} Angular={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
