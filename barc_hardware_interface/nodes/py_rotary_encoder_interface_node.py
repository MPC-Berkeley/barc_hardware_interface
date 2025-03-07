#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data

import copy

from mpclab_common.pytypes import NodeParamTemplate
from mpclab_common.mpclab_base_nodes import MPClabNode
from mpclab_msgs.msg import RotaryEncoderMsg

from mpclab_msgs.msg import EncoderMsg

from serial import Serial
from serial.tools import list_ports

import numpy as np
import serial
import time
import re

class RotaryEncoderNode(MPClabNode):

    def __init__(self):
        super().__init__("py_joystick_interface_node")
        self.counter_ = 0
        self.serial_data = self.create_publisher(
            RotaryEncoderMsg,
            'serial',
            qos_profile_sensor_data
        )
        
        self.timer_ = self.create_timer(0.05, self.send_data)

        self.arduino = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)
        time.sleep(2)
        self.arduino.flush()

    def read_serial(self):
        try:
            read_msg = "A\n"
            self.arduino.flushOutput()
            self.arduino.flushInput()
            self.arduino.write(read_msg.encode('utf-8'))
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode('utf-8').strip()
                # self.get_logger().info(f"{line}")
                if line and line[0] == "s":
                    line = line.strip()
                    data = line.split(':')
                    # self.get_logger().info(f"length data: {len(data)}")
                    if len(data) == 8:
                        # self.get_logger().info(f"data: {data}")
                        steering = int(data[1])
                        throttle = int(data[3])
                        vx = int(data[5])
                        vy = int(data[7])

                        return steering, throttle, vx, vy

        except Exception as e:
            print(f"Error in reading: {e}")

    def send_data(self):
        msg = RotaryEncoderMsg()
        outputs = self.read_serial()
        if outputs is not None:
            msg.steering = outputs[0]
            msg.throttle = outputs[1]
            self.get_logger().info(f"Steering = {msg.steering}, Throttle = {msg.throttle}, Vx = {msg.vx}, Vy = {msg.vy}")
            self.serial_data.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = RotaryEncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
