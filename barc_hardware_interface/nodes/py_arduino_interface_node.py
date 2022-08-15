#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from barc_hardware_interface.barc_interface import BarcArduinoInterface, BarcArduinoInterfaceConfig

from mpclab_common.msg import VehicleStateMsg, VehicleActuationMsg
from mpclab_common.pytypes import NodeParamTemplate, VehicleActuation, VehicleState, BodyLinearAcceleration
from mpclab_common.mpclab_base_nodes import MPClabNode

MSG_TIMEOUT_CTRL = 0.2

class InterfaceNodeParams(NodeParamTemplate):
    def __init__(self):
        self.dt                 = 0.01
        self.imu                = False
        self.interface_params   = BarcArduinoInterfaceConfig()
        
class ArduinoInterfaceNode(MPClabNode):

    def __init__(self):
        super().__init__('arduino_interface')
        self.get_logger().info('Initializing low level control node')
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

        self.update_timer = self.create_timer(self.dt, self.step)

        self.control_sub = self.create_subscription(VehicleActuationMsg,
                                                    'ecu',
                                                    self.control_callback,
                                                    qos_profile_sensor_data)
                                                    
        self.state_sub = self.create_subscription(VehicleStateMsg,
                                                  'est_state',
                                                  self.state_callback,
                                                  qos_profile_sensor_data)

        self.barc_state_pub = self.create_publisher(
            VehicleStateMsg,
            'barc_state',
            qos_profile_sensor_data)

        if self.imu:
            self.barc_imu_pub = self.create_publisher(
                Imu,
                'barc_imu',
                qos_profile_sensor_data
            )

        self.state = VehicleState()
        if self.interface_params.control_mode == 'direct':
            self.input = VehicleActuation(u_a=self.interface_params.throttle_off, u_steer=self.interface_params.steering_off)
        else:
            self.input = VehicleActuation(u_a=0, u_steer=0)

        self.control_msg_start = False
        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9 - MSG_TIMEOUT_CTRL
        self.output_enabled = False
        self.enable_output()

        self.control_alive = True

        return

    def control_callback(self, msg):
        self.unpack_msg(msg, self.input)
        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9
        if not self.control_msg_start:
            self.control_msg_start = True
        return

    def state_callback(self, msg):
        self.unpack_msg(msg, self.state)
        
    def step(self):
        t = self.clock.now().nanoseconds/1E9
        stamp = self.get_clock().now().to_msg()
        
        if t - self.last_msg_timestamp > MSG_TIMEOUT_CTRL and self.control_msg_start:
            self.control_alive = False

        if self.control_alive:
            self.state.u = self.input
            th, st = self.interface.step(self.state)
            self.state.hw.throttle = th
            self.state.hw.steering = st
        else:
            self.state.hw.throttle = self.interface_params.throttle_off
            self.state.hw.steering = self.interface_params.steering_off
            self.disable_output()

        self.state.t = t
        barc_state_msg = self.populate_msg(VehicleStateMsg(), self.state)
        self.barc_state_pub.publish(barc_state_msg)

        if self.imu:
            ax, ay, az = self.interface.read_accel()
            wx, wy, wz = self.interface.read_gyro()
            
            imu_msg = Imu()
            imu_msg.header.stamp = stamp

            imu_msg.linear_acceleration.x = ay
            imu_msg.linear_acceleration.y = ax
            imu_msg.linear_acceleration.z = az

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
