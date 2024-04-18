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
import mavros_interface as mav
import vision_control
import rospy
from mavros_msgs.msg import WaypointReached,WaypointList

mav_inter=mav.MavInterface()

start_aruco=25
target_aruco=1

def wp_list(wps):
    """
    This Function To Get Waypoints Number
    """
    global num_wps,start_pose
    num_wps=len(wps.waypoints)
    
# WP reached Callback function. Controls spam by publishing once per WP!
def wp_callback(msg):
    """
    We Use This Function To Know We Reach Aruco Markers Position And Then Call 'reach_aruco_markers' Function
    """
    global last_wp,num_wps,starting_time,sub_mission_started
    try: 
        sub_mission_started
    except NameError: # SUB-Mission begins
        rospy.loginfo("Starting SUB-MISSION: Waypoint #0 reached")
        starting_time = msg.header.stamp.secs
        sub_mission_started = True
    else: # SUB-Mission ongoing
        if msg.wp_seq == num_wps-1 : # Last WP, that is aruco markers position
            elapsed_time = msg.header.stamp.secs-starting_time	
            rospy.loginfo("Ending SUB-MISSION: Total time: %d s", elapsed_time)
            sub_mission_started=last_wp=None
            reach_aruco_markers()   # Here we know the copter has reached aruco markers position
        elif msg.wp_seq != last_wp: # Intermediate WPs
            elapsed_time = msg.header.stamp.secs-starting_time	
            rospy.loginfo("MISSION Waypoint #%s reached. Elapsed time: %d s", msg.wp_seq, elapsed_time)
   # Update last WP reached
    last_wp=msg.wp_seq

def go_to_aruco_markers():
    """
    This Function To Go To Aruco Markers Position With Auto Mode And Using Waypoints list That Stored In A File
    """
    global start_pose

    # Takeoff
    mav_inter.takeoff(20)
    while(mav_inter.get_height()<=19.5):
        print(f" takeoff : {mav_inter.get_height()}")
        sleep(1)

    start_pose=mav_inter.pose

    # AUTO MISSION: set mode, read WPs and Arm! 
    mav_inter.loadMission()
    mav_inter.setAutoMissionMode()

def reach_aruco_markers():
    """
    This Function Is Executed When The Copter Reach aruco markers position
    """
    global start_pose,start_aruco,target_aruco
    
    # set plane mode to guided mode
    mav_inter.setGuidedMode()

    # get down to 8 meters
    print("I  have  reached target position, now I will get down to 8 meters..")
    while(mav_inter.get_height()>7):
        mav_inter.change_height(-0.1)
        print(f" get down copter : {mav_inter.get_height()}")
        sleep(0.01)

    # start vision control
    print('now I will run Vision Control..')
    vis_cont=vision_control.VisionController(mav_inter)
    vis_cont.start(aruco_id=target_aruco)
    while(mav_inter.armed):
        print('still armed')
        sleep(1)

    # do somthing
    print('do something')
    sleep(5)

    # takeoff
    mav_inter.takeoff(20)
    while(mav_inter.get_height()<=19.5):
        print(f" takeoff : {mav_inter.get_height()}")
        sleep(1)

    # return to start position
    print('I am going to start position..')
    start_pose.orientation=mav_inter.pose.orientation
    mav_inter.goto(start_pose)
    while(not mav_inter.check_reach(start_pose)): # I don't like loop but is a temporary solution
        print("I haven't reached yet..")
        sleep(1)
    sleep(2)

    # get down to 8 meters
    print('I have start position, I will get down to 8 meters..')
    while(mav_inter.get_height()>7):
        mav_inter.change_height(-0.1)
        print(f" get down copter : {mav_inter.get_height()}")
        sleep(0.01)

    # start vision control
    print("now I will run Vision Control..")
    vis_cont.start(aruco_id=start_aruco)
    
    while(mav_inter.armed):
        print('still armed')
        sleep(1)
    
    # rest for 2 second
    print("rest..")
    sleep(2)

    # go to next target aruco
    target_aruco+=1
    if(target_aruco<=9):
        go_to_aruco_markers()
    else:
        # end mission entirely
        print("THE MISSION IS ENDED..")

# Main Function
def main():
    rospy.Subscriber('mavros/mission/reached', WaypointReached, wp_callback)
    rospy.Subscriber('/mavros/mission/waypoints', WaypointList, wp_list)
    go_to_aruco_markers()
    # Keep main loop
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    
    except rospy.ROSInterruptException:
        pass

