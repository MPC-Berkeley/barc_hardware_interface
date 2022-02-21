from serial import Serial
from serial.tools import list_ports
import numpy as np
from dataclasses import dataclass, field

from mpclab_common.pytypes import VehicleActuation, VehicleState, PythonMsg

@dataclass
class BarcArduinoInterfaceConfig():
    device_name: str = field(default = 'Nano')
    port: str   = field(default = None) # default to autoscan
    baud: int   = field(default = 115200)
    dt:   float = field(default = 0.01)
    require_echo: bool = field(default = False)

    steering_max: int = field(default = 1900)
    steering_min: int = field(default = 1100)
    steering_off: int = field(default = 1580)

    throttle_max: int = field(default = 1900)
    throttle_min: int = field(default = 1100)
    throttle_off: int = field(default = 1500)

    torque_control: bool = field(default = False)

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
steering_gain = -1050 # -1000

class BarcArduinoInterface():

    def __init__(self, config: BarcArduinoInterfaceConfig = BarcArduinoInterfaceConfig()):
        self.config = config
        self.start()
        self.dt=0.01
        self.v = 0
        self.desired_a = 0
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
        self.write_output(state)
        return

    def write_output(self, x: VehicleState):
        
        if self.config.torque_control:
            if not self.v==0 or not x.u.u_a==0:
                if x.u.u_a == self.desired_a:
                    c = self.config.dt
                    if x.u.u_a < 0:
                        c = self.config.dt*10
                    self.v += c*x.u.u_a
                else:
                    c = self.config.dt
                    if x.u.u_a < 0:
                        c = self.config.dt*10
                    self.desired_a = x.u.u_a
                    self.v = x.v.v_long + c*x.u.u_a
            throttle = self.v_to_pwm(self.v)
        else:
            throttle = self.config.throttle_off + throttle_gain * x.u.u_a * (self.config.throttle_max - self.config.throttle_min)
        throttle = max(min(throttle, self.config.throttle_max), self.config.throttle_min)

        # steering = self.config.steering_off + steering_gain * x.u.u_steer
        steering = self.angle_to_pwm(x.u.u_steer)
        steering = max(min(steering, self.config.steering_max), self.config.steering_min)
           
        self.serial.flushOutput()
        self.serial.write(b'A0%03d\n'%(throttle - 1000))
        self.serial.write(b'A1%03d\n'%(steering - 1000))

        return
    
    def write_raw_commands(self, throttle: int, steering: int):
        throttle = max(min(throttle, self.config.throttle_max), self.config.throttle_min)
        steering = max(min(steering, self.config.steering_max), self.config.throttle_min)
        
        self.serial.flushOutput()
        self.serial.write(b'A0%03d\n'%(throttle - 1000))
        self.serial.write(b'A1%03d\n'%(steering - 1000))
    
    def write_raw_serial(self, msg: str):
        self.serial.write(msg.encode('ascii'))
        
    def reset_output(self):
        self.serial.flushOutput()
        self.serial.write(b'A0%03d\n'%(self.config.throttle_off-1000))
        self.serial.write(b'A1%03d\n'%(self.config.steering_off-1000))
        return

    def enable_output(self):
        self.serial.write(b'AA000\n')
        return

    def disable_output(self):
        self.serial.write(b'AB000\n')
        return
    
    def read_encoders(self):
        self.serial.flushOutput()
        self.serial.flushInput()
        self.serial.write(b'B3000\n')
        
        msg = self.serial.read_until(expected='\r\n'.encode('ascii'), size=50).decode('ascii')
        fl_start = msg.find('a')
        fr_start = msg.find('b')
        rl_start = msg.find('c')
        rr_start = msg.find('d')
        if fl_start < 0 or fr_start < 0 or rl_start < 0 or rr_start < 0:
            return None
        v_fl = float(msg[fl_start+1:fr_start])
        v_fr = float(msg[fr_start+1:rl_start])
        v_rl = float(msg[rl_start+1:rr_start])
        v_rr = float(msg[rr_start+1:])
        return v_fl, v_fr, v_rl, v_rr

    def angle_to_pwm(self, steering_angle):
        Popt = [1.59587557e-01, 1.58019463e+03, -2.04235057e-03, 4.34184765e-01,
                3.74410204e-02, 2.18558980e-01]
        offset = Popt[1]
        gain = Popt[2]
        outer_gain = Popt[3]
        lr = Popt[4]
        lf = Popt[5]
        L = lr + lf
        u = np.tan(steering_angle / outer_gain) / gain + offset
        return u
    
    def v_to_pwm(self, v):
        pwm = self.config.throttle_off + 55.37439384702125*v
        return pwm

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
