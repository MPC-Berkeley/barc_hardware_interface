#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from mpclab_common.pytypes import VehicleState
from barc_hardware_interface.barc_interface import BarcArduinoInterface, BarcArduinoInterfaceConfig

from mpclab_common.pytypes import NodeParamTemplate, VehicleState, VehicleActuation
from mpclab_common.mpclab_base_nodes import MPClabNode

from mpclab_msgs.msg import VehicleActuationMsg

from collections import deque
import pickle

MSG_TIMEOUT_CTRL = 0.2

class InterfaceNodeParams(NodeParamTemplate):
    def __init__(self):
        self.dt                 = 0.01
        self.imu                = False
        self.interface_params   = BarcArduinoInterfaceConfig()

class ArduinoInterfaceNode(MPClabNode):

    def __init__(self):
        super().__init__('arduino_interface')
        namespace = self.get_namespace()

        param_template = InterfaceNodeParams()
        self.autodeclare_parameters(param_template, namespace, verbose=False)
        self.autoload_parameters(param_template, namespace, verbose=False)

        self.interface_params.dt = self.dt
        self.get_logger().info(str(self.interface_params))
        self.interface  = BarcArduinoInterface(self.interface_params,
                                                print_method=self.get_logger().info)
        
        # Get handle to ROS clock
        self.clock = self.get_clock()
        self.t_start = self.clock.now().nanoseconds/1E9
        
        self.barc_control_pub = self.create_publisher(
            VehicleActuationMsg,
            'barc_control',
            qos_profile_sensor_data)

        if self.imu:
            self.barc_imu_pub = self.create_publisher(
                Imu,
                'barc_imu',
                qos_profile_sensor_data
            )

        self.test_int = 1.25

        throttle_range = 175
        throttle_steps = 8
        self.throttle_seq = np.array([])
        # Forwards, stop, backwards, stop
        for th in np.linspace(0, throttle_range, throttle_steps)[1:]:
            self.throttle_seq = np.append(self.throttle_seq, np.array([th, 0, -th, 0]))
        self.get_logger().info('Throttle set points: ' + str(self.throttle_seq))
        
        steering_range = 300
        steering_steps = 21
        self.steering_seq = np.linspace(-steering_range, steering_range, steering_steps)
        self.get_logger().info('Steering set points: ' + str(self.steering_seq))

        self.update_timer = self.create_timer(self.dt, self.step)

        self.throttle_idx = 0
        self.steering_idx = 0

        self.first_test = True
        self.start_t = 0
        self.output_enabled = False
        self.enable_output()

        return

    def step(self):
        t = self.clock.now().nanoseconds/1E9
        stamp = self.get_clock().now().to_msg()
        
        if self.first_test:
            self.start_t = t
            self.first_test = False
        if t - self.start_t >= self.test_int:
            self.throttle_idx += 1
            if self.throttle_idx >= len(self.throttle_seq):
                self.throttle_idx = 0
                self.steering_idx += 1
                if self.steering_idx >= len(self.steering_seq):
                    self.get_logger().info(f'Finished')
                    self.disable_output()
            self.start_t = t

        throttle_cmd = int(self.interface_params.throttle_off + self.throttle_seq[self.throttle_idx])
        steering_cmd = int(self.interface_params.steering_off + self.steering_seq[self.steering_idx])
        self.get_logger().info(f'throttle: {throttle_cmd}, steering:{steering_cmd}')

        self.interface.write_raw_commands(throttle=throttle_cmd, steering=steering_cmd)

        control = VehicleActuation(t=t, u_a=throttle_cmd, u_steer=steering_cmd)
        control.t = t
        barc_control_msg = self.populate_msg(VehicleActuationMsg(), control)
        self.barc_control_pub.publish(barc_control_msg)

        if self.imu:
            imu_msg = Imu()
            imu_msg.header.stamp = stamp

            a, w = self.interface.read_accel(), None
            # w = self.interface.read_gyro()
            # a, w = self.interface.read_imu()

            if a is not None:
                ax, ay, az = a
                imu_msg.linear_acceleration.x = ay
                imu_msg.linear_acceleration.y = ax
                imu_msg.linear_acceleration.z = az
            if w is not None:
                wx, wy, wz = w
                imu_msg.angular_velocity.x = wy
                imu_msg.angular_velocity.y = wx
                imu_msg.angular_velocity.z = wz

            self.barc_imu_pub.publish(imu_msg)  

        return

    def disable_output(self):
        if self.output_enabled:
            self.interface.disable_output()
            self.output_enabled = False
            self.get_logger().info('Disabling HW output')

    def enable_output(self):
        if not self.output_enabled:
            self.interface.enable_output()
            self.output_enabled = True
            self.get_logger().info('Enabling HW output')


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
