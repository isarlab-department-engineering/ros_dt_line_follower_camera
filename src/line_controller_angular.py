#!/usr/bin/env python
from __future__ import print_function
import rospy, sys, cv2, time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from master_node.msg import *
from master_node.srv import *

#impostare il nome del nodo coerente con quello del master
id_node = "line"

#impistare la risposta positiva
positive_answ = 1


I = 0
last_error = 0
twistMessage = Twist()
twistMessage.linear.x = 0
twistMessage.linear.y = 0
twistMessage.linear.z = 0
twistMessage.angular.x = 0
twistMessage.angular.y = 0
twistMessage.angular.z = 0

followmessage = Follow()

followmessage.id = id_node

lock = False


pub = rospy.Publisher("follow_topic",Follow,queue_size=1)
request_lock_service = rospy.ServiceProxy('request_lock', RequestLockService)


def calculatePID(error,Kp,Ki,Kd):
    global last_error, I
    
    P = error
    if P > 100:
        P = 100
    elif P < -100:
        P = -100

    I = I + error
    
    if I > 300:
        I = 300
    elif I < -300:
        I = -300

    if error < 10 and error > -10:
        I = I - I/2

    D = error - last_error

    PID = float(Kp*P + Ki*I + Kd*D)

    last_error = error
    
    return PID

def turnOffMotors():
    # DEBUG VERSION TO FIX
    twistMessage.linear.x = 0
    twistMessage.angular.z = 0
    followmessage.twist = twistMessage
    pub.publish(followmessage)
    
def setSpeed(x_speed,z_speed):
    if x_speed == 0 and z_speed == 0:
        turnOffMotors()
    else:
        twistMessage.linear.x = x_speed
        twistMessage.angular.z = z_speed
        followmessage.twist = twistMessage
        print(x_speed,z_speed)
        pub.publish(followmessage)

def callback(data):

    error = data.data
    x_speed = 0.2

    PID = calculatePID(error,1,0.0,0.0)
#   rospy.loginfo(error)

    setSpeed(x_speed,PID)

'''
    #Pezzi di Controllo con Vel speciali
    elif error == 152:
        setSpeed(speed1,60)
#       setSpeed(speed1,70)

    elif error == 153:
        setSpeed(60,speed2)
#       setSpeed(70,speed2)

    else:
        if error == 154:
            time.sleep(0.5)
        turnOffMotors()
'''


def lane_controller():
    rospy.init_node('line_controller_angular', anonymous=True)

    #Sottoscrizione al topic shared
    rospy.Subscriber("lock_shared",Lock,checkMessage)


    rospy.Subscriber('line_detection', Float32, requestLock)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #TODO RELEASE LOCK


def requestLock(data):
    global id_node, lock
    if lock:
        callback(data)
    else:
        resp = request_lock_service(id_node)
        print(resp.ack)
        if resp.ack:
            print("dentro if")
            lock = True
            callback(data)
        else:
            print("fuori if")
            rospy.loginfo("ACK FALSO ASPETTO")
            msg_shared = rospy.wait_for_message("/lock_shared",Lock)
            rospy.loginfo("MESSAGGIO ARRIVATO PROCEDO")
            checkMessage(msg_shared)

def checkMessage(data):
    global id_node, lock
    if data.id == id_node:
        if data.msg == positive_answ:
            lock = True
        else:
            lock = False
    else:
        msg_shared = rospy.wait_for_message("/lock_shared", Lock)
        checkMessage(msg_shared)


if __name__ == '__main__':
    lane_controller()
