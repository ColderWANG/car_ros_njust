#!/usr/bin/python3

import cv2
import rospy


if __name__=='__main__':

    rospy.init_node("traffic_light_video")

    videoCapture = cv2.VideoCapture(0)

    fps=30

    timer=rospy.Rate(fps)
    
    size = (int(videoCapture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(videoCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("size:",size)
    videoWrite = cv2.VideoWriter('/home/w/njust_driver/traffi_light.avi', cv2.VideoWriter_fourcc('I', '4', '2', '0'), fps, size)

    try:
        while not rospy.is_shutdown():
            success, frame = videoCapture.read()
            if success is False:
                print("Capture Failed!")
                break
            else:
                videoWrite.write(frame)
                cv2.imshow('frame',frame)
                if cv2.waitKey(int(1000/fps))& 0xFF == ord('q'):
                    break
        videoCapture.release()
        videoWrite.release()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass



