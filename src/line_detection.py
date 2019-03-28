#!/usr/bin/env python

import roslib,rospy,sys,cv2,time
import numpy as np
roslib.load_manifest('line_follower_camera')
# from __future__ import print_function
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pub = rospy.Publisher('line_detection', Int32, queue_size=10) #ros-line-detection

startY = 50
altezza = 10


def callback(data):

    global startY, altezza

    # convert image to cv2 standard format
    img = bridge.imgmsg_to_cv2(data)

    # start time
    start_time = cv2.getTickCount()

    # Gaussian Filter to remove noise
    img = cv2.medianBlur(img,5)

    #img = img[:][startY:startY+altezza]

    Conv_hsv_Gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, mask = cv2.threshold(Conv_hsv_Gray, 0, 255,cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

    mask_line = mask[:][startY:startY+altezza]

    mean = np.mean(mask_line, axis=0)

    ind = np.argmax(mean)

    print(ind)

    pub.publish(ind)

    cv2.imshow("imgOriginal", Conv_hsv_Gray)  # show windows

    #cv2.imshow("output", res)  # show windows

    cv2.imshow("Test", mask)
    cv2.waitKey(2)



def line_detection_camera():
    rospy.init_node('line-detection-camera',anonymous=True)
    rospy.Subscriber("usb_cam/image_raw",Image,callback,queue_size=1,buff_size=2**24)
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