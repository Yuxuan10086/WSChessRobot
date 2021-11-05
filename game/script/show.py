#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point as rosPoint
from graphics import *
import time

rospy.init_node('show', anonymous = True)
column = rospy.get_param('column') - 1
row = rospy.get_param('row') - 1
gridWidth = rospy.get_param('showGridWidth')


win = GraphWin('gobang game', gridWidth * column, gridWidth * row)
win.setBackground('yellow')
i1 = 0
while i1 <= gridWidth * column:
    l = Line(Point(i1, 0), Point(i1, gridWidth * column))
    l.draw(win)
    i1 = i1 + gridWidth
i2 = 0
while i2 <= gridWidth * row:
    l = Line(Point(0, i2), Point(gridWidth * row, i2))
    l.draw(win)
    i2 = i2 + gridWidth

def drawPiece(pos, color, win):
    if pos[0] == -1:
        Text(Point(100, 100), 'black win.').draw(win)
        return
    if pos[0] == -2:
        Text(Point(100, 100), 'white win.').draw(win)
        return
    piece = Circle(Point(pos[0] * gridWidth, pos[1] * gridWidth), 16)
    piece.setFill(color)
    piece.draw(win)

while not rospy.is_shutdown():
    time.sleep(0.4)
    for y in range(column + 1):
        for x in range(row + 1):
            color = rospy.get_param('piece' + str(x) + '_' + str(y))
            if int(color) == 1:
                drawPiece([x, y], 'white', win)
            if int(color) == -1:
                drawPiece([x, y], 'black', win)