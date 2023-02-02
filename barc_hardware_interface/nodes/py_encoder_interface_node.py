#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data

import copy

from mpclab_common.msg import EncoderMsg
from mpclab_common.pytypes import NodeParamTemplate
from mpclab_common.mpclab_base_nodes import MPClabNode

from serial import Serial
from serial.tools import list_ports

import time
import multiprocess as mp

class InterfaceNodeParams(NodeParamTemplate):
    def __init__(self):
        self.dt                 = 0.01
        self.serial_port        = '/dev/ttyACM0'
        self.baud_rate          = 115200
        self.device_name        = 'Mega'
        
class EncoderInterfaceNode(MPClabNode):

    def __init__(self):
        super().__init__('encoder_interface')
        self.get_logger().info('Initializing encoder interface node')
        namespace = self.get_namespace()

        param_template = InterfaceNodeParams()
        self.autodeclare_parameters(param_template, namespace, verbose=False)
        self.autoload_parameters(param_template, namespace, verbose=False)
        
        # Get arduino port
        self.serial = Serial(port         = self.serial_port,
                             baudrate     = self.baud_rate,
                             timeout      = self.dt,
                             writeTimeout = self.dt)
        
        self.clock = self.get_clock()
        self.t_start = self.clock.now().nanoseconds/1E9

        self.update_timer = self.create_timer(self.dt, self.step)

        self.barc_enc_pub = self.create_publisher(
            EncoderMsg,
            'barc_enc',
            qos_profile_sensor_data
        )

        return
        
    def step(self):
        now= self.clock.now()
        t = now.nanoseconds/1E9
        stamp = now.to_msg()
        
        self.serial.flushOutput()
        self.serial.flushInput()
        
        try:
            msg = self.serial.read(size=50).decode('ascii')
        except:
            return

        start = msg.find('&')
        end = msg.find('\r\n', start)
        rl_start = msg.find('L', start)
        rr_start = msg.find('R', start)
        if start < 0 or end < 0:
            return
        else:
            try:
                v_rl = float(msg[rl_start+1:rr_start])
                v_rr = float(msg[rr_start+1:end])
            except:
                return

        enc_msg = EncoderMsg()
        enc_msg.header.stamp = stamp
        enc_msg.bl = v_rl
        enc_msg.br = v_rr
        self.barc_enc_pub.publish(enc_msg)

        return

def main(args=None):
    rclpy.init(args=args)
    node = EncoderInterfaceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()