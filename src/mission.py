#!/usr/bin/python

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

from time import sleep
from os.path import abspath,dirname
import mavros_interface as mav
import vision_control
import rospy
from mavros_msgs.msg import WaypointReached,WaypointList


class Mission:
    start_aruco=25
    target_aruco=9
    sub_mission_done=False
    def __init__(self):
        rospy.init_node("mission_node")
        self.mav_inter=mav.MavInterface()
        self.waypoints_path= dirname(dirname(abspath(__file__)))+"/data/my_wp.txt"
        rospy.Subscriber('mavros/mission/reached', WaypointReached, self.wp_callback)
        rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.wp_list)

    def wp_list(self,wps):
        self.num_wps=len(wps.waypoints)
        
    # WP reached Callback function. Controls spam by publishing once per WP!
    def wp_callback(self,msg): 
        try: 
            self.sub_mission_started
        except: # SUB-Mission begins
            rospy.loginfo("Starting SUB-MISSION: Waypoint #0 reached")
            self.starting_time = msg.header.stamp.secs
            self.sub_mission_started = True
        else: # SUB-Mission ongoing
            if msg.wp_seq == self.num_wps-1 : #and msg.wp_seq != last_wp: add this for closed mission,when last point is start point
                elapsed_time = msg.header.stamp.secs-self.starting_time	
                rospy.loginfo("Ending SUB-MISSION: Total time: %d s", elapsed_time)
                self.sub_mission_done=True
            elif msg.wp_seq != self.last_wp: # Intermediate WPs
                elapsed_time = msg.header.stamp.secs-self.starting_time	
                rospy.loginfo("MISSION Waypoint #%s reached. Elapsed time: %d s", msg.wp_seq, elapsed_time)
        # Update last WP reached
        self.last_wp=msg.wp_seq
    # Main Function
    def start(self):
        self.target_aruco=rospy.get_param("/mission_node/target_aruco",25)
        # Takeoff
        self.mav_inter.takeoff(20)
        while(self.mav_inter.get_height()<=19):
            print(f"takeoff : {self.mav_inter.get_height()}")
            sleep(1)
        
        self.start_pose=self.mav_inter.pose
        # AUTO MISSION for SUB-MISSION: set mode, read WPs and Arm!
        self.mav_inter.loadMission(waypoints_path=self.waypoints_path)
        self.mav_inter.setAutoMissionMode()
        self.sub_mission_done=False
        while(not self.sub_mission_done):
            sleep(0.1)

        # set plane mode to guided mode
        self.mav_inter.setGuidedMode()

        # get down to 8 meters
        print("I  have  reached target position, now I will land at a hight about of 8 meters..")
        while(self.mav_inter.get_height()>7):
            self.mav_inter.change_height(-0.1)
            sleep(0.01)
        print(f"landing copter at a height of: {self.mav_inter.get_height()}")
        # start vision control
        print('now it is time for Vision Control..')
        vis_cont=vision_control.VisionController(self.mav_inter)
        vis_cont.start(aruco_id=self.target_aruco)
        while(self.mav_inter.armed):
            print('still armed')
            sleep(1)

        # do somthing
        print('do something')
        sleep(5)

        # takeoff
        self.mav_inter.takeoff(20)
        while(self.mav_inter.get_height()<=19.5):
            print(f"takeoff : {self.mav_inter.get_height()}")
            sleep(1)

        # return to start position
        print('I am going to start position..')
        self.start_pose.orientation=self.mav_inter.pose.orientation
        self.mav_inter.goto(self.start_pose)
        while(not self.mav_inter.check_reach(self.start_pose)): # I don't like loop but is a temporary solution
            print("I haven't reached yet..")
            sleep(1)
        sleep(2)

        # get down to 8 meters
        print('I have reached start position, I will land at a height about of 8 meters..')
        while(self.mav_inter.get_height()>7):
            self.mav_inter.change_height(-0.1)
            sleep(0.01)
        print(f"landing copter at a height of: {self.mav_inter.get_height()}")
        # start vision control
        print("now it is time for Vision Control..")
        vis_cont.start(aruco_id=self.start_aruco)
        
        # end mission entirely
        print("THE MISSION IS DONE..")

    def end(self):
        """
        This Function Is Executed When The Mission is Finished
        """
        exit()

if __name__=='__main__':
    try:
        mission=Mission()
        mission.start()
        mission.end()
    except:
        pass

