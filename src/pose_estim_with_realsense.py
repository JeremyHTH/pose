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

class RiseHandDetect:
    def __init__(self):
        self.cvImage_Subscriber = rospy.Subscriber('/camera/color/image_raw',Image,self.cvImage_Subscriber_Callback)
        self.depthImage_Subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,self.depthImage_Subscriber_Callback)
        self.cameraInfo_subscriber = rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.Info_Subscriber_Callback)
        self.depthInfo_Subscriber = rospy.Subscriber('/human_depth',String,self.depthInfo_Subscriber_Callback)
        self.poseInfo_Publisher = rospy.Publisher('/PoseInfo',String,queue_size=10)
        self.left_Angle_Publisher = rospy.Publisher('/left_Angle',Int64,queue_size=10)
        self.right_Angle_Publisher = rospy.Publisher('/right_Angle',Int64,queue_size=10)
        self.locationX_Publisher = rospy.Publisher('body_position_x',String,queue_size=10)
        self.locationY_Publisher = rospy.Publisher('body_position_y',String,queue_size=10)

        self.cv_image = None
        self.depth_image = None
        self.image_length = None
        self.image_width = None
        self.depth = None
        self.bridge = CvBridge()

        self.detector = pm.poseDetector()

    def cvImage_Subscriber_Callback(self,data):

        Left_hand_up = Right_hand_up= Left_hand_straight = Right_hand_straight  = None
        Left_hand_angle = Right_hand_angle = gesture = tempx = tempy = -200

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        
        
        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50,50), 10, 255)

        img = self.detector.findPose(cv_image,True)
        lmList = self.detector.findPosition(img,True)

        
        if (len(lmList) != 0):
            Left_hand_up,Right_hand_up = self.hand_up(lmList)
            Left_hand_straight,Right_hand_straight = self.hand_Straight(lmList)
            Left_hand_angle,Right_hand_angle,gesture = self.hand_angle(lmList)
            if(not Left_hand_straight and not Right_hand_straight):
                gesture = None
            tempx = int((lmList[11][1] + lmList[12][1] + lmList[23][1] + lmList[24][1])//4)
            tempy = int((lmList[11][2] + lmList[12][2] + lmList[23][2] + lmList[24][2])//4)
        
        self.left_Angle_Publisher.publish(int(Left_hand_angle))
        self.right_Angle_Publisher.publish(int(Right_hand_angle))
        
        dimension = self.depth_image.shape
        if (tempx in range(0,dimension[0]) and tempy in range(0,dimension[1])):
            self.depth = self.depth_image[tempx][tempy]
        else:
            self.depth = -1

        cv2.putText(img,"depth: " + str(self.depth),(self.image_length - 300 ,self.image_width - 220),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"gesture: " + str(gesture),(self.image_length - 300 ,self.image_width - 190),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"left: " + str(int(Left_hand_angle)),(self.image_length - 300 ,self.image_width - 160),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"right: " + str(int(Right_hand_angle)),(self.image_length - 300 ,self.image_width - 130),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"left straight: " + str(Left_hand_straight),(self.image_length - 400 ,self.image_width - 100),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"right straight: " + str(Right_hand_straight),(self.image_length - 400 ,self.image_width - 70),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"left: " + str(Left_hand_up),(self.image_length - 300 ,self.image_width - 40),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"right: " + str(Right_hand_up),(self.image_length - 300 ,self.image_width - 10),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.imshow('image',img)
        cv2.imshow('depth',self.depth_image)
        # poseInfo_Publisher.publish(gesture)
        cv2.waitKey(3)


    def hand_up(self,data):
        if(len(data)>30):
            left = True if data[15][2]-data[11][2] < -10 else False
            right = True if data[16][2]-data[12][2] < -10 else False
        else:
            left = False
            right = False
        return left,right
    
    def hand_Straight(self,data):
        e = 0.0000001
        if(len(data) > 30):
            right_wrist_slope = math.atan((data[15][2]-data[13][2])/((data[15][1]-data[13][1]) + e)) /math.pi * 180 
            right_elbow_slope = math.atan((data[13][2]-data[11][2])/((data[13][1]-data[11][1]) + e)) /math.pi * 180
            left_wrist_slope = math.atan((data[16][2]-data[14][2])/((data[16][1]-data[14][1]) + e)) /math.pi * 180
            left_elbow_slope = math.atan((data[14][2]-data[12][2])/((data[14][1]-data[12][1]) + e)) /math.pi * 180

            left = True if abs(abs(right_elbow_slope) - abs(right_wrist_slope)) < 15 else False
            right = True if abs(abs(left_elbow_slope) - abs(left_wrist_slope)) < 15 else False
        else:
            left = False
            right = False

        return left,right

    def hand_angle(self,data):
        e = 0.0000001
        if(len(data) > 30):
            right = math.atan((data[15][2]-data[11][2])/((data[15][1]-data[11][1]) + e)) /math.pi * 180 
            left = math.atan((data[16][2]-data[12][2])/((data[16][1]-data[12][1]) + e)) /math.pi * 180
            post = None
            if left > 40 and left < 90:
                if right > -90 and right < -40:
                    post = '1'
                elif right < 30:
                    post = '2'
                elif right < 90:
                    post = '3'     

            elif left > -30:
                if right >-90 and right < -40:
                    post = '4'
                elif right < 30:
                    post = '5'
                elif right < 90:
                    post = '6'   
            
            elif left > -90:
                if right >-90 and right < -40:
                    post = '7'
                elif right < 30:
                    post = '8'
                elif right < 90:
                    post = '9' 

            return left,right,post
        

    def Info_Subscriber_Callback(self,data):
        self.image_length = int(data.P[2]*2)
        self.image_width = int(data.P[6]*2)
        # rospy.loginfo(" length:%s width:%s",data.s[2],data.P[6])

    def depthInfo_Subscriber_Callback(self,data):
        self.depth = int(data)

    def depthImage_Subscriber_Callback(self,data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

def main():
    rospy.init_node('PostDetectV1', anonymous=True)
    glo_RHD = RiseHandDetect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__== "__main__":
    main()