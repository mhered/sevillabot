#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
import serial


class JoySubscriberNode:
    
    def __init__(self):
        self.node = rclpy.create_node('joy_subscriber_node')
        self.subscriber = self.node.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.X_button_prev_state = 0  # Previous state of X button (laser)
        self.Y_button_prev_state = 0  # Previous state of Y button (trigger)
        self.waiting_for_arduino = False  # Flag to indicate if arduino is busy
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)  # Adjust port and baud rate as needed
        print('Serial port opened')

    def send_message_arduino(self, command):
        try:
            # Send message to Arduino over serial
            self.serial_port.write((command+'\r').encode('utf-8'))
            
            # Wait for response from Arduino
            response_from_arduino = self.serial_port.readline().decode('utf-8').strip()
            
            # Check if response is as expected
            if response_from_arduino == command+'_SUCCESS':
                print('Arduino responded with Success')
                response = True  
            else:
                print('ERROR: Unexpected response from Arduino: %s' % response_from_arduino)
                response = False

        except serial.SerialException as e:
            print('ERROR: Serial communication error: %s' % str(e))
            response = False

        return response

    def joy_callback(self, msg):

        # to get mapping of gamepad
        # (T1):$ ros2 run joy joy_node
        # (T2):$ ros2 topic echo /joy
        #
        # For Logitech 710 in Mode X:
        # buttons[0] = A
        # buttons[1] = B
        # buttons[2] = X
        # buttons[3] = Y
        # buttons[4] = LB
        # buttons[5] = RB
        # buttons[6] = BACK
        # buttons[7] = START
        # buttons[8] = ? (MODE and VIBRATION do not trigger response in /joy topic)
        # buttons[9] = Left Stick Push on
        # buttons[10] = Right Stick Push on
        #
        # axes[0] = Cross Left/Right. Discrete values Left= +1. None = 0, Right = -1
        # axes[1] = Cross Up/Down. Discrete values Up = +1, None = 0, Down = -1
        # axes[2] = LT. Float values from +1 (Not pressed) to -1 (Fully pressed)
        # axes[3] = Right Stick Left/Right. Float values from +1 (Left) to -1 (Right)
        # axes[4] = Right Stick Up/Down. Float values from +1 (Up) to -1 (Down)
        # axes[5] = RT. Float values from +1 (Not pressed) to -1 (Fully pressed)
        # axes[6] = Left Stick Left/Right. Discrete values Left= +1. None = 0, Right = -1
        # axes[7] = Left Stick Up/Down. Discrete values Up = +1, None = 0, Down = -1

        # For Logitech 710 in Mode D:
        # buttons[0] = X
        # buttons[1] = A
        # buttons[2] = B
        # buttons[3] = Y
        # buttons[4] = LB
        # buttons[5] = RB
        # buttons[6] = LT
        # buttons[7] = RT
        # buttons[8] = BACK
        # buttons[9] = START
        # buttons[10] = Left Stick Push on
        # buttons[11] = Right Stick Push on
        #
        # axes[0] = Left Stick Left/Right. Float values from +1 (Left) to -1 (Right)
        # axes[1] = Left Stick Up/Down. Float values from +1 (Up) to -1 (Down)
        # axes[2] = Right Stick Left/Right. Float values from +1 (Left) to -1 (Right)
        # axes[3] = Right Stick Up/Down. Float values from +1 (Up) to -1 (Down)
        # axes[4] = Cross Left/Right. Discrete values Left= +1. None = 0, Right = -1
        # axes[5] = Cross Up/Down. Discrete values Up = +1, None = 0, Down = -1   
        
        # Example of Trigger behaviour
        # Check the state of the Y button
        Y_button_state = msg.buttons[3]  # Y button is at index 3

        # Send msg to Arduino if Y button is pressed (state changes from 0 to 1) and arduino is not already busy
        if Y_button_state == 1 and self.Y_button_prev_state == 0:
            self.node.get_logger().info("Y button pressed")
            if not self.waiting_for_arduino:
                self.send_message_arduino('FIRE')

        # Example of Toggle behaviour
        # Check the state of the X button
        X_button_state = msg.buttons[2]  # X button is at index 2

        # Send Arduino Trigger command if X button is pressed (state changes from 0 to 1) and arduino is not already busy 
        if X_button_state == 1 and self.X_button_prev_state == 0:
            self.node.get_logger().info("X button pressed")
            if not self.waiting_for_arduino:
                self.send_message_arduino('LASER')

        # Example of Hold-down behaviour
        # Check the state of the VERT CROSS axis
        cross_vertical_state = msg.axes[7]  # Vertical axis of CROSS is at index 7

        # Send msg to Arduino if VERT CROSS button is pressed and arduino is not already busy
        command =""
        if cross_vertical_state > 0:
            command = "UP"
        elif cross_vertical_state < 0:
            command = "DOWN"
        if not self.waiting_for_arduino and command != "":
            self.send_message_arduino(command)
            self.node.get_logger().info("Adjusting Vertical Aim " + command)

       # Check the state of the HORZ CROSS axis
        cross_horizontal_state = msg.axes[6]  # Horizontal axis of CROSS is at index 6

        # Send msg to Arduino if HORZ CROSS button is pressed and arduino is not already busy
        command =""
        if cross_horizontal_state > 0:
            command = "OPEN"
        elif cross_horizontal_state < 0:
            command = "CLOSE"
        if not self.waiting_for_arduino and command != "":
            self.send_message_arduino(command)
            self.node.get_logger().info("Adjusting Vertical Aim " + command)

        self.Y_button_prev_state = Y_button_state
        self.X_button_prev_state = X_button_state


def main(args=None):
    rclpy.init(args=args)
    joy_subscriber_node = JoySubscriberNode()
    rclpy.spin(joy_subscriber_node.node)
    joy_subscriber_node.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
