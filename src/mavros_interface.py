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
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import OverrideRCIn,State,RCIn,ParamValue
from mavros_msgs.srv import CommandBool,SetMode,CommandTOL,ParamSet,ParamGet
import math
import os


class MavInterface:
    """
    A simple object to help interface with mavros
    """
    yaw=None
    euler=(0,0,0)
    def __init__(self):
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)
        rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        rospy.set_param('theta_aruco',500)
        rospy.set_param('x0_object',-1)
        rospy.set_param('x0_image',-1)
        rospy.set_param('y0_object',-1)
        rospy.set_param('y0_image',-1)


        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose=self.next_pose=self.start_pose= Pose()
        
        self.timestamp = rospy.Time()
        self.armed=False
    
    def start(self):
        pass

    def mavros_state_callback(self,data):
         self.armed=data.armed

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w )
        self.euler = tf.transformations.euler_from_quaternion(quaternion) # eluer=(roll,pitch,yaw)

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + math.pi/2.0)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)

    def change_height(self,change_value):
        pose=self.pose
        pose.position.z+=change_value
        self.goto(pose)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)
        
    def check_reach(self,target):
        return abs(self.pose.position.x-target.position.x)<=1 and \
               abs(self.pose.position.y-target.position.y)<=1 and \
               abs(self.pose.position.z-target.position.z) <=1

    def get_height(self):
        return self.pose.position.z

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        rospy.wait_for_service('mavros/set_mode')
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        #mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp
        return mode_resp
        
    def setGuidedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            self.mode_service(custom_mode="4")
            rospy.loginfo("Entering Guided mode OK")
        except (rospy.ServiceException):
            rospy.logerror("Entering Guided failed: %s. Guided mode could not be set.")


    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        rospy.wait_for_service('mavros/set_mode')
        resp = self.mode_service(custom_mode="9")
        self.disarm()

    def loadMission(self,waypoints_path):
	    # Load mission from file (MP format). 
	    # To do: change to avoid using os.system + mavwp
                     # Handle errors pending!
        os.system("rosrun mavros mavwp load "+waypoints_path) # Load new mission
        rospy.loginfo("Mission WayPoints LOADED!")
    
    def setAutoMissionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            self.mode_service(custom_mode='AUTO')
            rospy.loginfo("Entering AUTO mode OK")
        except (rospy.ServiceException):
            rospy.logerror("Entering AUTO failed: %s. AUTO mode could not be set.")

    def get_parameter(self,param):
        try:	    
            get = rospy.ServiceProxy("/mavros/param/get", ParamGet)  
            value=get(param_id=param)
        except(rospy.ServiceException):
            rospy.logerror("Failsafe Status read failed")
        
        return value

    def set_parameter(self,param,val=ParamValue(integer=0, real=0.0)):
        try: 
            set = rospy.ServiceProxy("/mavros/param/set", ParamSet)
            new_val = set(param_id=param, value=val)  
            print(f" New {param} value is : {new_val.value.integer}")
        except (rospy.ServiceException):
            rospy.logerror("Failsafe Status change failed: %s")

