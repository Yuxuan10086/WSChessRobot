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

def drawCallback(pos, color, win):
    if pos.x == -1:
        Text(Point(100, 100), "black win.").draw(win)
        return
    if pos.y == -2:
        Text(Point(100, 100), "white win.").draw(win)
        return
    piece = Circle(Point(pos.x * GRID_WIDTH, pos.y * GRID_WIDTH), 16)
    piece.setFill(color)
    # piece.draw(win)

rospy.init_node('game', anonymous = True)
rospy.Subscriber("human", rosPoint, lambda pos : drawCallback(pos, 'black', win))
rospy.Subscriber("ai", rosPoint, lambda pos : drawCallback(pos, 'white', win))
rospy.spin()
