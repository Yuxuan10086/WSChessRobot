#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point as rosPoint
from graphics import *
import threading

GRID_WIDTH = 40
COLUMN = 15
ROW = 15



win = GraphWin("this is a gobang game", GRID_WIDTH * COLUMN, GRID_WIDTH * ROW)
win.setBackground("yellow")

i1 = 0
while i1 <= GRID_WIDTH * COLUMN:
    l = Line(Point(i1, 0), Point(i1, GRID_WIDTH * COLUMN))
    l.draw(win)
    i1 = i1 + GRID_WIDTH

i2 = 0
while i2 <= GRID_WIDTH * ROW:
    l = Line(Point(0, i2), Point(GRID_WIDTH * ROW, i2))
    l.draw(win)
    i2 = i2 + GRID_WIDTH

rospy.init_node('game', anonymous = True)
rospy.Subscriber("human", rosPoint, lambda pos : print(pos.x))
rospy.Subscriber("ai", rosPoint, lambda pos : print(pos.x))
rospy.spin()