import cv2
import time  
import PoseModule as pm


def main():
    cap = cv2.VideoCapture('video_camera_color_image_raw.mp4')
    pTime = 0
    detector = pm.poseDetector()
    Left_hand_shake = False
    Right_hand_shake = False
    left_stage = 1
    right_stage = 1

    while True:
        success, img = cap.read()
        img = detector.findPose(img,True)
        lmList = detector.findPosition(img,True)
        # print(lmList)

        # if len(lmList) != 0:
        #     cv2.circle(img,(lmList[14][1],lmList[14][2]), 15,(0,0,255), cv2.FILLED)

        Left_hand_up,Right_hand_up = hand_shake(lmList)

        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
        text = 'Fps : ' + str((int)(fps))
        cv2.putText(img,text,(70,50), cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"left: " + str(Left_hand_up),(400,400),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.putText(img,"right: " + str(Right_hand_up),(400,430),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
        cv2.imshow("Image",img)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        

def hand_shake(data):
    if(len(data)>30):
        left = True if data[15][2]-data[11][2] <0 else False
        right = True if data[16][2]-data[12][2] <0 else False
    else:
        left = False
        right = False
    return left,right


if __name__== "__main__":
    main()