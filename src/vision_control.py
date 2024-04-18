#    Copyright 2024 Hasan Osman

#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import mavros_interface as mav
import threading
from pynput import keyboard
import rospy
import time
import tf
import math
import copy
import numpy
from os.path import dirname,abspath
from copter_project.msg import VisionInfo

class VisionController():
    x_last_error=x_cum_error=0
    x_kp=0.0003
    x_kd=0.0004
    x_ki=0.00001
    y_last_error=y_cum_error=0
    y_kp=0.0003
    y_kd=0.0004
    y_ki=0.00001
    x_allowed_error=10
    y_allowed_error=10
    manual_speed=1
    pressed_key=None
    yaw=None
    next_pose=None
    prev_time=time.time()
    mav_inter=None
    pid_data=None
    start_save_data_time=None
    x0_object=-1
    y0_object=-1
    theta_aruco=500

    def __init__(self,mavros_interface):
        self.mav_inter=mavros_interface
        vision_info_sub=rospy.Subscriber('/vision_info',VisionInfo,self.vision_info_callback,queue_size=1)

    def start(self,aruco_id):
        self.keythread=threading.Thread(target=self.forKey)
        self.keythread.start()
        self.mav_inter.setGuidedMode()
        stop=False
        self.next_pose=copy.deepcopy(self.mav_inter.pose)
        self.yaw=self.mav_inter.euler[2]
        self.pid_data=None
        self.start_save_data_time=None
        while(not self.pressed_key=='q' and not stop):
            rospy.set_param("aruco_id",aruco_id)
            self.mav_inter.goto(self.get_next_pose())
            self.pressed_key=None
            if(self.mav_inter.pose.position.z<=0.8):
                stop=True
            rospy.sleep(0.1)
        numpy.savetxt(dirname(dirname(abspath(__file__)))+'/data/pid_data/pid_data.csv',self.pid_data,delimiter=',')
        self.mav_inter.land()
        self.key_listener.stop()
    
    def vision_info_callback(self,data):
        self.image_x0=data.image_x0
        self.image_y0=data.image_y0
        self.x0_object=data.x0
        self.y0_object=data.y0
        self.theta_aruco=data.theta_aruco
    def on_press(self,key):
        try:
            self.pressed_key=key.char
        except AttributeError:
            self.pressed_key=key

    def on_release(self,key):
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    # Collect events until released
    def forKey(self):
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as self.key_listener:
            self.key_listener.join()
            


    def get_next_pose(self):
            try:
                FLU_x_shift,FLU_y_shift,FLU_z_shift,yaw_shift=self.get_shifts()
                
                theta_aruco=self.theta_aruco
                x0_object=self.x0_object
                y0_object=self.y0_object
                quaternion=self.get_next_rotation(yaw_shift,theta_aruco)

                if(x0_object!=-1):# and theta_aruco>-3 and theta_aruco<3):                                                         
                    x_pid_output,y_pid_output,change_in_z=self.get_pid_output(self.image_x0,x0_object,self.image_y0,y0_object)                    
                    x_pid_output,y_pid_output,change_in_z=self.rotate_cordinate(x_pid_output,y_pid_output,change_in_z,-90) #convert camera frame to copter frame(FLU) 
                else:
                    x_pid_output=y_pid_output=change_in_z=0
                    self.prev_time=time.time()
                    self.x_cum_error=self.y_cum_error=self.x_last_error=self.y_last_error=0

                FLU_x_shift+=x_pid_output
                FLU_y_shift+=y_pid_output
                FLU_z_shift=FLU_z_shift+change_in_z

                ENU_x_shift,ENU_y_shift,ENU_z_shift=self.FLU2ENU(FLU_x_shift,FLU_y_shift,FLU_z_shift) #convert FLU(Forward Left Up) frame to ENU(East North Up) frame

                self.next_pose.position.x+=ENU_x_shift
                self.next_pose.position.y+=ENU_y_shift
                self.next_pose.position.z+=ENU_z_shift
                self.next_pose.orientation.x=quaternion[0]
                self.next_pose.orientation.y=quaternion[1]
                self.next_pose.orientation.z=quaternion[2]
                self.next_pose.orientation.w=quaternion[3]
                return self.next_pose
            except Exception as e:
                print(e)

    def get_next_rotation(self,yaw_shift,theta_aruco):
            euler=self.mav_inter.euler
            roll=euler[0]
            pitch=euler[1]
            yaw=euler[2]
            if(theta_aruco==500):
                self.yaw+=(yaw_shift/180)*math.pi
            else:
                self.yaw=yaw
                self.yaw=self.yaw-theta_aruco
            self.yaw=self.yaw%(math.pi*2)
            quaternion=tf.transformations.quaternion_from_euler(roll,pitch,self.yaw)
            return quaternion

    def get_shifts(self):
            FLU_x_shift=FLU_y_shift=FLU_z_shift=yaw_shift=0 #this is in body frame and we will convert body frame to world frame 
            if(self.pressed_key==keyboard.Key.up):FLU_x_shift=self.manual_speed
            if(self.pressed_key==keyboard.Key.down):FLU_x_shift=-self.manual_speed
            if(self.pressed_key==keyboard.Key.right):FLU_y_shift=-self.manual_speed
            if(self.pressed_key==keyboard.Key.left):FLU_y_shift=self.manual_speed
            if(self.pressed_key=='a'):FLU_z_shift=0.2
            if(self.pressed_key=='z'):FLU_z_shift=-0.2
            if(self.pressed_key=='s'):yaw_shift=2
            if(self.pressed_key=='x'):yaw_shift=-2
            return FLU_x_shift,FLU_y_shift,FLU_z_shift,yaw_shift
            
    def get_pid_output(self,x_set_point,x_point,y_set_point,y_point):
        curr_time=time.time()
        elpased_time=curr_time-self.prev_time

        ## save pid's performance data
        self.save_pid_data(curr_time,x_set_point,x_point,y_set_point,y_point)

        ##calculate x pid
        x_error=x_point-x_set_point
        x_rate_error=(x_error-self.x_last_error)/elpased_time
        self.x_cum_error+=x_error*elpased_time
        x_p=self.x_kp*x_error
        x_i=self.x_ki*self.x_cum_error
        x_d=self.x_kd*x_rate_error
        x_pid_output=x_p+x_i+x_d
        self.x_last_error=x_error

        ##calculate y pid
        y_error=y_set_point-y_point
        y_rate_error=(y_error-self.y_last_error)/elpased_time
        self.y_cum_error+=y_error*elpased_time
        y_p=self.y_kp*y_error
        y_i=self.y_ki*self.y_cum_error
        y_d=self.y_kd*y_rate_error
        y_pid_output=y_p+y_i+y_d
        self.y_last_error=y_error
        
        ##calculate change in z
        change_in_z=(-0.2 if(abs(x_error) in range(0,self.x_allowed_error+1) and abs(y_error) in range(0,self.y_allowed_error+1)) else 0)
        # print(x_error,'   ',y_error,'   ',change_in_z)
        self.prev_time=curr_time

        return x_pid_output,y_pid_output,change_in_z

    def FLU2ENU(self,x,y,z):
        yaw=self.mav_inter.euler[2]
        ENU_x = x * math.cos(yaw) - y * math.sin(yaw)
        ENU_y = x * math.sin(yaw) + y * math.cos(yaw)
        ENU_z = z

        return ENU_x, ENU_y, ENU_z

    def rotate_cordinate(self,x,y,z,theta):
        """
        this function is to rotate cordinate frame by specific angle ,
        note this function is same of FLU2ENU function that is to rotate cordinate frame from FLU(Forward Left Up) to ENU(East North Up)
        """
        new_x= x * math.cos(theta)- y * math.sin(theta)
        new_y= x * math.sin(theta) + y * math.cos(theta)
        new_z=z
        return new_x,new_y,new_z

    def save_pid_data(self,time,x0,x,y0,y):
        if(self.pid_data is None):
            self.pid_data=numpy.array([0,x0,x,y0,y])
            self.start_save_data_time=time
        else:
            self.pid_data=numpy.vstack([self.pid_data,numpy.array([time-self.start_save_data_time,x0,x,y0,y])])

