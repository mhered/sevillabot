#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sevillabot.msg import TimestampedData
import serial

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.publisher_ = self.create_publisher(
            TimestampedData, '/line_follower_error', 10)
        self.serial_port = serial.Serial(
            # PC config assuming this is the serial assigned 
            # '/dev/ttyUSB0',
            # Robot config assumes the Addon Arduino is plugged in the bottom-left USB port of the RPi
            # and the Mega4 is plugged in the bottom-right USB port of the RPi
            '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0',
            9600, 
            timeout=1)  # Adjust port and baud rate as needed

        # Configure timer to read and publish every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)  
        

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line.startswith('E:'):
                parts = line[2:].split('\t')
                if len(parts) == 2:
                    try:
                        timestamp = int(parts[0])
                        data = int(parts[1])
                        msg = TimestampedData()
                        msg.timestamp = timestamp
                        msg.data = data
                        self.publisher_.publish(msg)
                        self.get_logger().info(
                            f'Publishing: timestamp={msg.timestamp}, data={msg.data}')
                    except ValueError:
                        self.get_logger().error('Received non-integer data')
                else:
                    self.get_logger().error('Invalid data format')
            else:
                self.get_logger().error('Waiting for data')

def main(args=None):
    rclpy.init(args=args)
    line_follower_node = LineFollowerNode()
    rclpy.spin(line_follower_node)
    line_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

