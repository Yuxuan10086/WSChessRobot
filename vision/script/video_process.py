#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import cv2
import cv_bridge

rospy.init_node('video_process', anonymous=True)
before = rospy.Publisher('map_image/before', Image, queue_size=1)
after = rospy.Publisher('map_image/after', Image, queue_size=1)

cam_id = rospy.get_param('camera_id')
try:
    cap = cv2.VideoCapture(cam_id)
    cap.release()
except:
    raise Exception('摄像头编号错误')
converter = cv_bridge.CvBridge()

def pubImgCallback(publisher):
    cap = cv2.VideoCapture(cam_id)
    sucess, img = cap.read()
    img = converter.cv2_to_imgmsg(img, encoding='bgr8')
    publisher.publish(img)
    cap.release()
    

# while not rospy.is_shutdown():
#     cap = cv2.VideoCapture(cam_id)
#     sucess, img = cap.read()
#     img = converter.cv2_to_imgmsg(img, encoding='bgr8')
#     if beforeImg != 0:
#         before.publish(beforeImg)
#     after.publish(img)
#     beforeImg = img
#     cap.release()
#     rate.sleep()

rospy.Subscriber("ai", Point, lambda data : pubImgCallback(before))
rospy.Subscriber("human_signal", Bool, lambda data : pubImgCallback(after))
rospy.spin()

# 机器落子后发送标志信号 此时记录beforeimg 检测到画面静止且出现新的棋子后记录afterimg