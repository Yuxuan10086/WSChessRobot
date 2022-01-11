#!/usr/bin/env python3
import serial
import rospy
from geometry_msgs.msg import Point

bps = 9600
portx = rospy.get_param('portx')
ser=serial.Serial(portx, bps,timeout=None)

def aiCallback(point):
    pass
    

rospy.Subscriber('ai', Point, aiCallback)
rospy.spin()