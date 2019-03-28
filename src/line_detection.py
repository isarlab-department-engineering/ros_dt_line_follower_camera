#!/usr/bin/env python

import roslib,rospy,sys,cv2,time
import numpy as np
roslib.load_manifest('line_follower_camera')
# from __future__ import print_function
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pub = rospy.Publisher('line_detection', Float32, queue_size=1) #ros-line-detection

startY = 250
altezza = 10


def callback(data):

    global startY, altezza

    # convert image to cv2 standard format
    img = bridge.imgmsg_to_cv2(data)

    #cv2.imshow("B", img[:,:,0])
    #cv2.imshow("G", img[:,:,1])
    #cv2.imshow("R", img[:,:,2])

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    cv2.imshow("Orig", img)

    # start time
    start_time = cv2.getTickCount()

    # Gaussian Filter to remove noise
    img = cv2.medianBlur(img,5)

    #print(img.shape)

    Conv_hsv_Gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    #print(Conv_hsv_Gray.shape)

    ret, mask = cv2.threshold(Conv_hsv_Gray, 0, 255,cv2.THRESH_OTSU) #SFONDO NERO E LINEA BIANCA

    #ret, mask = cv2.threshold(Conv_hsv_Gray, 0, 255,cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU) #SFONDO BIANCO E LINEA NERA

    im_thresh_gray = cv2.bitwise_and(Conv_hsv_Gray, mask)

    mask_line = mask[:][startY:startY+altezza]

    mean = np.mean(mask_line, axis=0)

    length = float(len(mean))

    mx = float(np.argmax(mean))

    error = mx/length

    error = error - 0.5

    print(error)

    pub.publish(error)

    cv2.imshow("imgOriginal", im_thresh_gray)  # show windows

    cv2.imshow("output", mask_line)  # show windows

    cv2.imshow("Mask", mask)
    cv2.waitKey(2)



def line_detection_camera():
    rospy.init_node('line-detection-camera',anonymous=True)
    #rospy.Subscriber("usb_cam/image_raw",Image,callback,queue_size=1,buff_size=2**24)
    rospy.Subscriber("usb_camera/image_raw",Image,callback,queue_size=1,buff_size=2**24)
    try:
        rospy.loginfo("Enetering ROS Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        line_detection_camera()
    except rospy.ROSInterruptException:
        pass