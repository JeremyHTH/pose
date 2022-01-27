#!/usr/bin/env python3
import cv2

class photo_trim():

    def __init__(self):
        pass

 
def main():
    img = cv2.imread('/home/jeremy/catkin_ws/src/pose/src/trim_image.png')
    cv2.imshow('img', img)
    trim = img[189:618,368:883]
    cv2.imshow('trim',trim)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__== "__main__":
    main()