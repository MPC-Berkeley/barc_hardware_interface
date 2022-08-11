from multiprocessing.sharedctypes import Value
from serial import Serial
from serial.tools import list_ports
import numpy as np
from dataclasses import dataclass, field

from mpclab_common.pytypes import VehicleState, PythonMsg

@dataclass
class BarcArduinoInterfaceConfig(PythonMsg):
    device_name: str = field(default = 'Nano')
    port: str   = field(default = None) # default to autoscan
    baud: int   = field(default = 115200)
    dt:   float = field(default = 0.01)
    require_echo: bool = field(default = False)

    steering_max: int = field(default = 1990)
    steering_min: int = field(default = 1010)
    steering_off: int = field(default = 1500)

    throttle_max: int = field(default = 1900)
    throttle_min: int = field(default = 1100)
    throttle_off: int = field(default = 1500)

    steering_map_mode: str      = field(default='affine')
    steering_map_params: list   = field(default=None)
    throttle_map_mode: str      = field(default='affine')
    throttle_map_params: list   = field(default=None)
    
    control_mode: str = field(default = 'torque')

class BarcArduinoInterface():

    def __init__(self, config: BarcArduinoInterfaceConfig = BarcArduinoInterfaceConfig(), 
                        print_method=print):
        self.config = config
        self.dt = config.dt
        self.v = 0

        self.print_method = print_method

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
        self.write_output(state)
        return

    def write_output(self, x: VehicleState):
        if self.config.control_mode == 'torque':
            if self.config.throttle_map_mode == 'integration':
                self.v += self.dt*x.u.u_a
                throttle = self.v_to_pwm(self.v)
            elif self.config.throttle_map_mode == 'affine':
                throttle = self.a_to_pwm(x.u.u_a)
            else:
                raise(ValueError("Throttle map mode must be 'affine', 'integration'"))
            steering = self.angle_to_pwm(x.u.u_steer)
        elif self.config.control_mode == 'velocity':
            throttle = self.v_to_pwm(x.u.u_a)
            steering = self.angle_to_pwm(x.u.u_steer)
        elif self.config.control_mode == 'direct':
            throttle = x.u.u_a
            steering = x.u.u_steer
        else:
            raise(ValueError("Control mode must be 'torque', 'velocity', or 'direct'"))
        
        if throttle > 1.1*self.config.throttle_max or throttle < 0.9*self.config.throttle_min:
            throttle = self.config.throttle_off
        if steering > 1.1*self.config.steering_max or steering < 0.9*self.config.steering_min:
            steering = self.config.steering_off
        throttle = int(max(min(throttle, self.config.throttle_max), self.config.throttle_min))
        steering = int(max(min(steering, self.config.steering_max), self.config.steering_min))
        
        self.serial.flushOutput()
        self.serial.write(b'A0%03d\n'%(throttle - 1000))
        self.serial.write(b'A1%03d\n'%(steering - 1000))

        return throttle, steering
    
    def write_raw_commands(self, throttle: int, steering: int):
        throttle = max(min(throttle, self.config.throttle_max), self.config.throttle_min)
        steering = max(min(steering, self.config.steering_max), self.config.steering_min)
        
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
    
    def read_accel(self):
        self.serial.flushOutput()
        self.serial.flushInput()
        self.serial.write(b'B2000\n')
        
        msg = self.serial.read_until(expected='\r\n'.encode('ascii'), size=50).decode('ascii')
        ax_start = msg.find('x')
        ay_start = msg.find('y')
        az_start = msg.find('z')
        if ax_start < 0 or ay_start < 0 or az_start < 0:
            return None
        ax = float(msg[ax_start+1:ay_start])*9.81
        ay = float(msg[ay_start+1:az_start])*9.81
        az = float(msg[az_start+1:])*9.81
        return ax, ay, az
        
    def angle_to_pwm(self, steering_angle):
        if self.config.steering_map_mode == 'affine':
            offset, gain = self.config.steering_map_params
            steer_pwm = steering_angle / gain + offset
        elif self.config.steering_map_mode == 'arctan':
            # Popt = [1.01471732e-01, 1.490e+03, -1.57788871e-03, 5.38431760e-01,
            #         1.18338718e-01, 1.37661282e-01]
            # Popt = self.config.steering_map_params
            offset = self.config.steering_map_params[1]
            gain = self.config.steering_map_params[2]
            outer_gain = self.config.steering_map_params[3]
            # lr = Popt[4]
            # lf = Popt[5]
            # L = lr + lf
            steer_pwm = np.tan(steering_angle / outer_gain) / gain + offset
        else:
            raise(ValueError("Steering map mode must be 'affine' or 'arctan'"))
        return steer_pwm
    
    def v_to_pwm(self, v):
        # K = 55.37439384702125
        K = self.config.throttle_map_params[0]
        throttle_pwm = self.config.throttle_off + K * v
        return throttle_pwm

    def a_to_pwm(self, a):
        gain = self.config.throttle_map_params[0]
        throttle_pwm = a / gain + self.config.throttle_off
        return throttle_pwm

if __name__ == '__main__':
    arduino = BarcArduinoInterface()
