#!/usr/bin/env python3
import cv2
import time  
import PoseModule as pm
import rospy
import math
from Filter_data import Filter_data
from std_msgs.msg import Int64


def main():
    rospy.init_node('Filter_data_testing', anonymous=True)
    cap = cv2.VideoCapture('/home/jeremy/catkin_ws/src/pose/src/video_camera_color_image_raw.mp4')
    success = True
    detector = pm.poseDetector()
    smooth_left = Filter_data()
    smooth_right = Filter_data()
    e = 0.0000001
    left_Angle_Publisher = rospy.Publisher('/left_Angle',Int64,queue_size=10)
    right_Angle_Publisher = rospy.Publisher('/right_Angle',Int64,queue_size=10)
    left_Angle_smooth_Publisher = rospy.Publisher('/left_Angle_smooth',Int64,queue_size=10)
    right_Angle_smooth_Publisher = rospy.Publisher('/right_Angle_smooth',Int64,queue_size=10)

    success, cv_image = cap.read()
    cv2.imshow('image',cv_image)
    cv2.waitKey(0)

    while success:
        success, cv_image = cap.read()
        cv_image = detector.findPose(cv_image,True)
        lmList = detector.findPosition(cv_image,True)
        
        right = math.atan((lmList[15][2]-lmList[11][2])/((lmList[15][1]-lmList[11][1]) + e)) /math.pi * 180 
        left = math.atan((lmList[16][2]-lmList[12][2])/((lmList[16][1]-lmList[12][1]) + e)) /math.pi * 180

        show_left = smooth_left.smoothed_data(left)
        show_right = smooth_right.smoothed_data(right)

        left_Angle_Publisher.publish(int(left))
        right_Angle_Publisher.publish(int(right))
        left_Angle_smooth_Publisher.publish(int(show_left))
        right_Angle_smooth_Publisher.publish(int(show_right))

        cv2.imshow('image',cv_image)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break




if __name__== "__main__":
    main()