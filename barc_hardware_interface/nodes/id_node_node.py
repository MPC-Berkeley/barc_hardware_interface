#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data

from mpclab_common.pytypes import VehicleState
from barc_hardware_interface.barc_interface import BarcArduinoInterface


from mpclab_common.mpclab_base_nodes import MPClabNode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
MSG_TIMEOUT_CTRL = 0.1


class Vec3d:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
        return
    
class BarcInterface(MPClabNode):

    def __init__(self):
        super().__init__('barc3d_interface')
        namespace = self.get_namespace()

        self.barc  = BarcArduinoInterface()
        self.state = VehicleState()
        self.clock = self.get_clock()

        self.update_timer = self.create_timer(0.01, self.step)

        self.imu_accel_sub = self.create_subscription(Imu,
                                                      '/imu/sample',
                                                      self.imu_callback,
                                                      qos_profile_sensor_data)
                                                      
                                                      
        self.camera_accel_sub = self.create_subscription(Imu,
                                                        '/camera/accel/sample',
                                                        self.camera_callback,
                                                        qos_profile_sensor_data)
                                                      
        self.vive_sub = self.create_subscription(Odometry, 
                                                 '/T_4/odom',
                                                 self.vive_callback,
                                                 qos_profile_sensor_data)
                                                 
        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9 - MSG_TIMEOUT_CTRL
        self.tstart = self.clock.now().nanoseconds/1E9
        self.output_enabled = False
        # self.disable_output()
        self.enable_output()
        
        #### data #####
        
        self.imu_a = Vec3d()
        self.cam_a = Vec3d()
        self.vive_vlinear = Vec3d()
        self.vive_vangular = Vec3d()

        ### data vectors ###
        self.t_vec = []
        self.imu_a_vec = []
        self.cam_a_vec = []
        self.vive_vlinear_vec = []
        self.vive_vangular_vec = []
        self.throttle_vec = []
        self.steering_vec = []
        self.v_fl_vec = []
        self.v_fr_vec = []
        self.v_rl_vec = []
        self.v_rr_vec = []
        self.v_avg_vec = []
        return
        
    def imu_callback(self, msg):
        self.imu_a = Vec3d(msg.linear_acceleration.x, 
                           msg.linear_acceleration.y, 
                           msg.linear_acceleration.z)
        return

    def camera_callback(self, msg):
        self.cam_a = Vec3d(msg.linear_acceleration.x, 
                           msg.linear_acceleration.y, 
                           msg.linear_acceleration.z)
        return

    def vive_callback(self, msg):                     
        self.vive_vlinear = Vec3d(msg.twist.twist.linear.x, 
                                  msg.twist.twist.linear.y, 
                                  msg.twist.twist.linear.z)

        self.vive_vangular = Vec3d(msg.twist.twist.angular.x, 
                                   msg.twist.twist.angular.y,
                                   msg.twist.twist.angular.z)
        return
        
    def step(self):
        t = self.clock.now().nanoseconds/1E9 - self.tstart
        
        self.t_vec.append(t)
        self.imu_a_vec.append(self.imu_a)
        self.cam_a_vec.append(self.cam_a)
        self.vive_vlinear_vec.append(self.vive_vlinear)
        self.vive_vangular_vec.append(self.vive_vangular)

        #################### writing commands #######################
       
        throttle = 1550 if t < 2 else 1550
        steering = 1500 if t < 2 else 1500
       
        self.throttle_vec.append(throttle)
        self.steering_vec.append(steering)
        self.barc.write_raw_commands(throttle=throttle, steering=steering)
        
        
        ###################### reading speed #########################
        v_fl, v_fr, v_rl, v_rr, v_avg = 0,0,0,0,0
       #v = self.barc.read_encoders()
        v = 0,0,0,0
        if v is not None:
            v_fl, v_fr, v_rl, v_rr = v
            v_avg = (v_fl + v_fr + v_rl + v_rr)/4.
            #self.get_logger().info(f'fl:{v_fl}, fr:{v_fr}, rl:{v_rl}, rr:{v_rr}')
            #self.get_logger().info(f'v avg:{v_avg}')
            
        self.v_fl_vec.append(v_fl)
        self.v_fr_vec.append(v_fr)
        self.v_rl_vec.append(v_rl)
        self.v_rr_vec.append(v_rr)
        self.v_avg_vec.append(v_avg)

        ##################### saving to file ###########################
        if t > 4: 
            import os
            import sys
            
            filename = 'id_data.npz'
            fileno = 1
            while os.path.exists(filename):
                filename = 'id_data (%d).npz'%fileno
                fileno += 1
            np.savez(filename, t = self.t_vec, u_a = self.throttle_vec, u_y = self.steering_vec,
                   imu_a = self.imu_a_vec, cam_a = self.cam_a_vec, vive_vlinear = self.vive_vlinear_vec,
                   vive_vangular = self.vive_vangular_vec, v_fl = self.v_fl_vec, 
                   v_fr = self.v_fr_vec, v_rl = self.v_rl_vec, v_rr = self.v_rr_vec, v_avg = self.v_avg_vec) 
            sys.exit(0)
            
      
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

    def angle_to_pwm(self, steering_angle):

        #u = tan(steering_angle / outer_gain) * gain + offset
        # beta = ca.arctan(lr/L*ca.tan(steering_angle))
        # Popt = [1.01469519e-01,  1.52277729e+03, -2.44954380e-03, 3.65220714e-01,
        # 5.42083502e-02,  2.01791650e-01]
	
	    # without v transformation 
        #Popt = [ 1.59587557e-01,  1.58019463e+03, -2.04235057e-03,  4.34184765e-01,
        #3.74410204e-02,  2.18558980e-01]

        # with v Transformation
	    #Popt = [1.59587557e-01,  1.57976089e+03, -2.03580401e-03,  4.35267745e-01,
        #1.23028808e-01,  1.32971192e-01]
	    
        Popt = [ 1.20106157e-01,  1.60183547e+03, -4.21558157e-03, 2.51962419e-01, 3.58509959e-02,  2.20149004e-01]
        
        offset = Popt[1]
        gain = Popt[2]
        outer_gain = Popt[3]
        lr = Popt[4]
        lf = Popt[5]

        L = lr + lf

        # original
        # steering_angle = outer_gain*(ca.arctan( (u(t-delay) - offset) * gain) )   
        # beta = ca.arctan(lr/L*ca.tan(steering_angle))

        # inverted
        # steering_angle = np.arctan(L/lr * np.tan(beta))
        u = np.tan(steering_angle / outer_gain) / gain + offset

        return u

def main(args=None):
    rclpy.init(args=args)
    node = BarcInterface()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
