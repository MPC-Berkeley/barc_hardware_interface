#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data

from barc_hardware_interface.barc_interface import BarcArduinoInterface, BarcArduinoInterfaceConfig

from mpclab_common.pytypes import VehicleState, VehicleActuation
from mpclab_common.msg import VehicleStateMsg, VehicleActuationMsg
from mpclab_common.mpclab_base_nodes import MPClabNode

MSG_TIMEOUT_CTRL = 0.1

class BarcInterface(MPClabNode):

    def __init__(self):
        super().__init__('barc3d_interface')
        namespace = self.get_namespace()

        # TODO
        #param_template = JoystickNodeParams()
        #self.autodeclare_parameters(param_template, namespace)
        #self.autoload_parameters(param_template, namespace)
        
        config = BarcArduinoInterfaceConfig(torque_control=True)
        self.barc  = BarcArduinoInterface(config)
        self.state = VehicleState()
        self.input = VehicleActuation(u_a=0, u_steer=0)

        self.clock = self.get_clock()

        self.update_timer = self.create_timer(config.dt, self.step)

        self.control_sub = self.create_subscription(VehicleActuationMsg,
                                                    'ecu',
                                                    self.control_callback,
                                                    qos_profile_sensor_data)
                                                    
        self.state_sub = self.create_subscription(VehicleStateMsg,
                                                  'est_state',
                                                  self.state_callback,
                                                  qos_profile_sensor_data)

        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9 - MSG_TIMEOUT_CTRL
        self.output_enabled = False
        # self.disable_output()
        self.enable_output()

        return

    def control_callback(self, msg):
        self.unpack_msg(msg, self.input)
        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9
        return
    
    def state_callback(self, msg):
        self.unpack_msg(msg, self.state)
        return

    def step(self):
        t = self.clock.now().nanoseconds/1E9
        self.state.u = self.input
        #self.get_logger().info(str(self.state))
        self.barc.step(self.state)

        return

    def disable_output(self):
        if self.output_enabled:
            self.barc.disable_output()
            self.output_enabled = False
            self.get_logger().info('Disabling HW output due to timeout')

    def enable_output(self):
        if not self.output_enabled:
            self.barc.enable_output()
            self.output_enabled = True
            self.get_logger().info('Enabling HW output')


def main(args=None):
    rclpy.init(args=args)
    node = BarcInterface()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
