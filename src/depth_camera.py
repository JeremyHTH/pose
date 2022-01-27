#!/usr/bin/env python3

from __future__ import print_function

import cv2
import PoseModule as pm
import rospy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_msgs.msg import Int64
from cv_bridge import CvBridge, CvBridgeError

class depth_testing():
    def __init__(self):
        self.cameraInfo_subscriber = rospy.Subscriber('/camera/depth/camera_info',CameraInfo,self.Info_Subscriber_Callback)
        self.depth_Subscriber = rospy.Subscriber('/camera/depth/image_rect_raw',Image,self.depth_Subscriber_Callback)
        self.depthOfHuman_Publisher = rospy.Publisher('/human_depth',String,queue_size=10)
        self.locationX_Subscriber = rospy.Subscriber('body_position_x',String,self.posX_sub_callback)
        self.locationY_Subscriber = rospy.Subscriber('body_position_y',String,self.posY_sub_callback)

        self.cv_image = None
        self.image_length = None
        self.image_width = None
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0


    def Info_Subscriber_Callback(self,data):
        self.image_length = int(data.P[2]*2)
        self.image_width = int(data.P[6]*2)
        # print(self.image_length," and ",self.image_width)

    def depth_Subscriber_Callback(self,data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)
        
        print(depth_image[self.x//2][self.y//2])
        self.depthOfHuman_Publisher.publish(str(depth_image[self.x//2][self.y//2]))

        print(type(self.x))
        
        cv2.namedWindow("depth")
        cv2.imshow('depth',depth_image)
        cv2.waitKey(1)

    def posX_sub_callback(self,data):   
        self.x = int(data)
        print("x: ", self.x)

    def posY_sub_callback(self,data):
        self.y = int(data)
        print("y: ", self.y)


def main():
    rospy.init_node('depth_testing', anonymous=True)
    glo_RHD = depth_testing()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__== "__main__":
    main()