from serial import Serial
from serial.tools import list_ports
import numpy as np
from dataclasses import dataclass, field
import pigpio

from barc3d.pytypes import VehicleActuation, VehicleState, PythonMsg, DriveState

@dataclass
class BarcArduinoInterfaceConfig():
    device_name: str = field(default = 'Nano')
    port: str   = field(default = None) # default to autoscan
    baud: int   = field(default = 115200)
    dt:   float = field(default = 0.01)

    steering_max: int = field(default = 1999)
    steering_min: int = field(default = 1000)
    steering_off: int = field(default = 1500)

    throttle_max: int = field(default = 1999)
    throttle_min: int = field(default = 1000)
    throttle_off: int = field(default = 1500)



@dataclass
class BarcPiInterfaceConfig():
    steering_pin: int = field(default = 5)
    throttle_pin: int = field(default = 6)

    steering_max: int = field(default = 1999)
    steering_min: int = field(default = 1000)
    steering_off: int = field(default = 1500)

    throttle_max: int = field(default = 1999)
    throttle_min: int = field(default = 1000)
    throttle_off: int = field(default = 1500)

throttle_gain = 15/90
steering_gain = -1000 #200/17.0 * 180 / np.pi

class BarcArduinoInterface():

    def __init__(self, config: BarcArduinoInterfaceConfig = BarcArduinoInterfaceConfig()):
        self.config = config
        self.hw_state = DriveState()
        self.start()
        return

    def start(self):
        if self.config.port is None:
            self.config.port = self.scan_ports()

            if self.config.port is None:
                raise NotImplementedError('No Arduino Found')
                return

        self.serial = Serial(port         = self.config.port,
                             baudrate     = self.config.baud,
                             timeout      = self.config.dt,
                             writeTimeout = self.config.dt)

        return

    def scan_ports(self):
        ports = list(list_ports.comports())
        for p in ports:
            if self.config.device_name in p.description:
                return p.device

        return None

    def step(self, state:VehicleState):
        self.write_output(state.u)
        state.hw = self.hw_state.copy()
        return

    def write_output(self, u: VehicleActuation):

        throttle = self.config.throttle_off + throttle_gain * u.u_a * (self.config.throttle_max - self.config.throttle_min)
        throttle = max(min(throttle, self.config.throttle_max), self.config.throttle_min)

        steering = self.config.steering_off + steering_gain * u.u_steer
        steering = max(min(steering, self.config.steering_max), self.config.throttle_min)

        self.hw_state.throttle = throttle
        self.hw_state.steering = steering
        self.hw_state.brake = 0


        self.serial.write(b'A0%03d\n'%(throttle - 1000))
        self.serial.write(b'A1%03d\n'%(steering - 1000))

        return

    def reset_output(self):
        self.serial.write(b'A0%03d\n'%(self.config.throttle_off-1000))
        self.serial.write(b'A1%03d\n'%(self.concig.steering_off-1000))
        return

    def enable_output(self):
        self.serial.write(b'AA000\n')
        return

    def disable_output(self):
        self.serial.write(b'AB000\n')
        return


class BarcPiInterface():
    def __init__(self, config: BarcPiInterfaceConfig= BarcPiInterfaceConfig()):
        self.config = config
        self.hw_state = DriveState()
        self.start()

    def start(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise ConnectionRefusedError('Pigpio Daemon is not available')

        self.last_throttle_output = 0
        self.last_steering_output = 0

    def step(self, state:VehicleState):
        self.write_output(state.u)
        state.hw = self.hw_state.copy()
        return

    def write_output(self, u: VehicleActuation):
        throttle = self.config.throttle_off + throttle_gain * u.a * (self.config.throttle_max - self.config.throttle_min)
        throttle = max(min(throttle, self.config.throttle_max), self.config.throttle_min)

        steering = self.config.steering_off + steering_gain * u.y
        steering = max(min(steering, self.config.steering_max), self.config.throttle_min)

        self.hw_state.throttle = throttle
        self.hw_state.steering = steering
        self.hw_state.brake = 0

        self.pi.set_servo_pulsewidth(self.config.throttle_pin, throttle)
        self.pi.set_servo_pulsewidth(self.config.steering_pin, steering)

        self.last_throttle_output = throttle
        self.last_steering_output = steering

    def reset_output(self):
        self.pi.set_servo_pulsewidth(self.config.throttle_pin, self.config.throttle_off)
        self.pi.set_servo_pulsewidth(self.config.steering_pin, self.config.steering_off)
        self.last_throttle_output = self.config.throttle_off
        self.last_steering_output = self.config.steering_off

    def enable_output(self):
        self.pi.set_servo_pulsewidth(self.config.throttle_pin, self.last_throttle_output)
        self.pi.set_servo_pulsewidth(self.config.steering_pin, self.last_steering_output)

    def disable_output(self):
        self.pi.set_servo_pulsewidth(self.config.throttle_pin, 0)
        self.pi.set_servo_pulsewidth(self.config.steering_pin, 0)

if __name__ == '__main__':
    arduino = BarcArduinoInterface()
