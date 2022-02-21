#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data

from mpclab_common.pytypes import VehicleState
from barc_hardware_interface.barc_interface import BarcArduinoInterface

from mpclab_common.pytypes import VehicleState, VehicleActuation
from mpclab_common.msg import VehicleStateMsg, VehicleActuationMsg
from mpclab_common.mpclab_base_nodes import MPClabNode

from collections import deque
import pickle

MSG_TIMEOUT_CTRL = 0.1

class BarcInterface(MPClabNode):

    def __init__(self):
        super().__init__('barc3d_interface')
        namespace = self.get_namespace()

        self.barc  = BarcArduinoInterface()
        
        self.state = VehicleState()
        self.input = VehicleActuation(u_a=0, u_steer=0)
        
        self.clock = self.get_clock()
        
        # self.throttle_max = self.barc.config.throttle_max
        # self.throttle_min = self.barc.config.throttle_min
        self.throttle_range = 175
        self.throttle_off = self.barc.config.throttle_off
        self.throttle_cmd = 0
        self.throttle_test_step = 25
        self.throttle_test_int = 1.25
        self.it_start = None
        self.data = []
        
        self.throttle_cmd_seq = []
        for i in range(int(self.throttle_range/self.throttle_test_step)):
            self.throttle_cmd_seq.extend([self.throttle_test_step*(i+1), 0, -self.throttle_test_step*(i+1), 0])
        
        self.steering_off = self.barc.config.steering_off
        
        self.finished = False
        
        self.n_hist = 100

        self.update_timer = self.create_timer(0.01, self.step)

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
        
        if not self.finished:
            if self.it_start is None:
                self.it_start = t
                self.throttle_cmd = self.throttle_cmd_seq.pop(0)
                self.v_long_hist = []
            elif t - self.it_start >= self.throttle_test_int:
                self.get_logger().info(f'throttle: {self.throttle_off+self.throttle_cmd}, v:{np.mean(self.v_long_hist)}')
                self.data.append(dict(throttle=self.throttle_off+self.throttle_cmd, v=self.v_long_hist))
                if len(self.throttle_cmd_seq) == 0:
                    self.finished = True
                    with open('/home/mpclab/barc_data/long_data.pkl', 'wb') as f:
                        pickle.dump(self.data, f)
                    return
                self.it_start = t
                self.throttle_cmd = self.throttle_cmd_seq.pop(0)
                self.v_long_hist = []
            
            self.barc.write_raw_commands(throttle=self.throttle_off+self.throttle_cmd, steering=self.steering_off)
            
            self.v_long_hist.append(self.state.v.v_long)
                
        else:
            self.barc.write_raw_commands(throttle=self.throttle_off, steering=self.steering_off)

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
