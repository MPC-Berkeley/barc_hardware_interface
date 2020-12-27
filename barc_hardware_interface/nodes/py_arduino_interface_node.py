#!/usr/bin/env python3

import rclpy

from mpclab_common.msg import State, Actuation
from mpclab_common.mpclab_base_nodes import MPClabNode

SHOW_MSG_TRANSFER_WARNINGS = False

class ArduinoInterfaceNode(MPClabNode):

    def __init__(self):
        super().__init__('arduino_interface')

        namespace = self.get_namespace()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
