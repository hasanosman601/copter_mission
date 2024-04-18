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

import math
import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from copter_project.msg import VisionInfo

bridge=CvBridge()
rospy.set_param('aruco_id',-1)
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
def image_callback(data):
    try:
        image=bridge.imgmsg_to_cv2(data,'bgr8')
        image_width=float(image.shape[1])
        image_height=float(image.shape[0])
    except CvBridgeError as e:
            print(e)
    (corners, ids, rejected) = detector.detectMarkers(image)
    cv2.aruco.drawDetectedMarkers(image,corners,ids,(0,255,0))
    vision_msg=VisionInfo()
    if(ids is not None):
        aruco_id=rospy.get_param('aruco_id')
        print(aruco_id)
        index=np.where(ids==[aruco_id])
        if(len(index[0])!=0):
            index=index[0][0]
            points=corners[index][0]
            y=points[1][1]-points[0][1]
            x=points[1][0]-points[0][0]
            theta=math.atan2(y,x)
            x0_object=float(points[0][0]+points[2][0])/2

            y0_object=float(points[0][1]+points[2][1])/2
            print(x0_object,'  ',y0_object)
            vision_msg.x0=int(x0_object)
            vision_msg.y0=int(y0_object)
            vision_msg.theta_aruco=theta
        else:
            vision_msg.x0=-1
            vision_msg.y0=-1
            vision_msg.theta_aruco=500        
    else:
        vision_msg.x0=-1
        vision_msg.y0=-1
        vision_msg.theta_aruco=500
    image_x0=int(image_width/2)
    image_y0=int(image_height/2)
    vision_msg.image_x0=image_x0
    vision_msg.image_y0=image_y0
    vision_info_pub.publish(vision_msg)
    cv2.imshow("image",image)
    cv2.waitKey(1)

rospy.init_node('cv',anonymous=True)
image_sub=rospy.Subscriber('/iris_demo/usb_cam/image_raw',Image,image_callback)
vision_info_pub=rospy.Publisher('/vision_info',VisionInfo,queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("shutting down")

rospy.set_param('theta_aruco',500)
cv2.destroyAllWindows()
