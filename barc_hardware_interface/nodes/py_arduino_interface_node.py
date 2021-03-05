#!/usr/bin/env python3

import rclpy

from serial import Serial
import numpy as np

from mpclab_common.msg import State, Actuation
from mpclab_common.pytypes import VehicleActuation
from mpclab_common.mpclab_base_nodes import MPClabNode

SHOW_MSG_TRANSFER_WARNINGS = False

class ArduinoInterfaceNode(MPClabNode):

    def __init__(self):
        super().__init__('arduino_interface')
        namespace = self.get_namespace()

        self.declare_parameters(
            namespace=namespace,
            parameters=[
                ('dt', 0.1),
                ('serial.port', '/dev/ttyACM0'),
                ('serial.baudrate', 115200),
                ('steering.pwm_max', 1800),
                ('steering.pwm_min', 1200),
                ('steering.pwm_neutral', 1500),
                ('steering.gain', 200/17),
                ('steering.deadband', 0.001),
                ('throttle.pwm_max', 2000),
                ('throttle.pwm_min', 1000),
                ('throttle.pwm_neutral', 1500),
                ('throttle.deadband', 0.001)
            ]
        )

        self.dt = self.get_parameter('.'.join((namespace,'dt'))).value

        self.port = self.get_parameter('.'.join((namespace,'serial.port'))).value
        self.baudrate = self.get_parameter('.'.join((namespace,'serial.baudrate'))).value

        self.steering_pwm_max = self.get_parameter('.'.join((namespace,'steering.pwm_max'))).value
        self.steering_pwm_min = self.get_parameter('.'.join((namespace,'steering.pwm_min'))).value
        self.steering_pwm_neutral = self.get_parameter('.'.join((namespace,'steering.pwm_neutral'))).value
        self.steering_gain = self.get_parameter('.'.join((namespace, 'steering.gain'))).value
        self.steering_deadband = self.get_parameter('.'.join((namespace,'steering.deadband'))).value
        self.steering_pwm_range_u = self.steering_pwm_max - self.steering_pwm_neutral
        self.steering_pwm_range_l = self.steering_pwm_neutral - self.steering_pwm_min

        self.throttle_pwm_max = self.get_parameter('.'.join((namespace,'throttle.pwm_max'))).value
        self.throttle_pwm_min = self.get_parameter('.'.join((namespace,'throttle.pwm_min'))).value
        self.throttle_pwm_neutral = self.get_parameter('.'.join((namespace,'throttle.pwm_neutral'))).value
        self.throttle_deadband = self.get_parameter('.'.join((namespace,'throttle.deadband'))).value
        self.throttle_pwm_range_u = self.throttle_pwm_max - self.throttle_pwm_neutral
        self.throttle_pwm_range_l = self.throttle_pwm_neutral - self.throttle_pwm_min

        # Make serial connection to Arduino
        try:
            self.serial = Serial(port=self.port, baudrate=self.baudrate, timeout=self.dt, writeTimeout=self.dt)
        except Exception as e:
            self.get_logger().info('===== Serial connection error: %s =====' % e)

        self.update_timer = self.create_timer(self.dt, self.step)

        self.control = VehicleActuation(u_a=0, u_steer=0)
        self.pwm = VehicleActuation(u_a=self.throttle_pwm_neutral, u_steer=self.steering_pwm_neutral)

        self.control_sub = self.create_subscription(
            Actuation,
            'ecu',
            self.control_callback,
            10)

        self.wait_time = 1.0
        self.node_counter = 0
        self.interface_mode = 'init'

    def control_callback(self, msg):
        self.unpack_msg(msg, self.control)

    def step(self):
        # Check for initialization and termination modes
        if self.interface_mode == 'init':
            self.pwm.u_a = self.throttle_pwm_neutral
            self.pwm.u_steer = self.steering_pwm_neutral
            if self.node_counter*self.dt >= self.wait_time:
                self.get_logger().info('===== Arduino Interface start =====')
                self.interface_mode = 'run'
        elif self.interface_mode == 'finished':
            # Apply braking
            self.pwm.u_a = self.throttle_pwm_min
            self.pwm.u_steer = self.steering_pwm_min
        else:
            throttle_accel, steer_rad = self.control.u_a, -self.control.u_steer

            # Map from desired steering angle to PWM
            if np.abs(steer_rad) <= self.steering_deadband:
                self.pwm.u_steer = self.steering_pwm_neutral
            elif steer_rad > self.steering_deadband:
                self.pwm.u_steer = steer_rad * (180/np.pi) * self.steering_gain * self.steering_pwm_range_u + self.steering_pwm_neutral
            elif steer_rad < -self.steering_deadband:
                self.pwm.u_steer = steer_rad * (180/np.pi) * self.steering_gain * self.steering_pwm_range_l + self.steering_pwm_neutral
            self.pwm.u_steer = self.saturate_pwm(self.pwm.u_steer, self.steering_pwm_max, self.steering_pwm_min)

            # Map from desired acceleration to PWM
            if np.abs(throttle_accel) <= self.throttle_deadband:
                self.pwm.u_a = self.throttle_pwm_neutral
            elif throttle_accel > self.throttle_deadband:
                self.pwm.u_a = (1.0 + 6.5*throttle_accel)*self.throttle_pwm_range_u/90.0 + self.throttle_pwm_neutral
            elif throttle_accel < -self.throttle_deadband:
                self.pwm.u_a = (3.5 + 6.73*throttle_accel)*self.throttle_pwm_range_l/90.0+ self.throttle_pwm_neutral
            self.pwm.u_a = self.saturate_pwm(self.pwm.u_a, self.throttle_pwm_max, self.throttle_pwm_min)

        try:
            self.send_serial(self.pwm)
            # self.get_logger().info('%g: %s' %(self.node_counter, str(self.pwm)))
        except Exception as e:
            self.get_logger().info('===== Serial comms error: %s =====' % e)
            # self.interface_mode = 'finished'

        self.node_counter += 1

    def send_serial(self, pwm):
        serial_msg = '& {} {}\r'.format(int(pwm.u_a), int(pwm.u_steer))
        self.get_logger().info(serial_msg)
        self.serial.write(serial_msg.encode('ascii'))

    def saturate_pwm(self, pwm, pwm_max, pwm_min):
        return np.around(max(min(pwm, pwm_max), pwm_min))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
