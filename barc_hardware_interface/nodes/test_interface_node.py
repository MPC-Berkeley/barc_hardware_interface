#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data

from mpclab_common.pytypes import VehicleState
from barc_hardware_interface.barc_interface import BarcArduinoInterface

#from mpclab_common.msg import VehicleState     as StateMsg
#from mpclab_common.msg import BodyLinearAcceleration as AccelMsg
#from mpclab_common.msg import BodyAngularVelocity    as GyroMsg
#from mpclab_common.msg import VehicleActuation as ActuationMsg
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

        self.barc  = BarcArduinoInterface()
        self.state = VehicleState()
        #self.state_msg = StateMsg()
        self.clock = self.get_clock()

        self.update_timer = self.create_timer(0.01, self.step)

        # self.control_sub = self.create_subscription(ActuationMsg,
        #                                             'ecu',
        #                                             self.control_callback,
        #                                             qos_profile_sensor_data)
        #
        #
        # self.state_pub = self.create_publisher(StateMsg,
        #                                        'state',
        #                                        qos_profile_sensor_data)
        #
        # self.accel_sub  = self.create_subscription(AccelMsg,
        #                                         'accel',
        #                                         self.accel_callback,
        #                                         qos_profile_sensor_data)

        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9 - MSG_TIMEOUT_CTRL
        self.output_enabled = False
        # self.disable_output()
        self.enable_output()

        return

    def control_callback(self, msg):
        self.unpack_msg(msg, self.state.u)
        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9
        return

    def accel_callback(self, msg):
        self.unpack_msg(msg, self.state.a)
        return

    def step(self):
        t = self.clock.now().nanoseconds/1E9

        # if t - self.last_msg_timestamp > MSG_TIMEOUT_CTRL:
        #     self.disable_output()
        # else:
        #     self.enable_output()

        #steering = 0.1*np.sin(t/(2*np.pi))
        #throttle = 0.5*np.sin(t/(2*np.pi))
        
        #self.get_logger().info(f'steering: {steering}, throttle: {throttle}')
        
        #self.state.u.u_a = throttle
        #self.state.u.u_steer = steering
        #self.barc.step(self.state)
        
        self.barc.write_raw_commands(throttle=1550, steering=1500)
        
        v = self.barc.read_encoders()
        if v is not None:
            v_fl, v_fr, v_rl, v_rr = v
            v_avg = (v_fl + v_fr + v_rl + v_rr)/4.
            #self.get_logger().info(f'fl:{v_fl}, fr:{v_fr}, rl:{v_rl}, rr:{v_rr}')
            self.get_logger().info(f'v avg:{v_avg}')
        
        #self.state.t = t
        #self.populate_msg(self.state_msg, self.state)
        #self.state_pub.publish(self.state_msg)

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
