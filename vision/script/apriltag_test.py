import cv2
import time
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# rospy.init_node('test')
# before = rospy.Publisher('/map_image/before', Image, queue_size=10)

cam_id = 0
converter = cv_bridge.CvBridge()

cap = cv2.VideoCapture(cam_id)
sucess, img = cap.read()
cv2.imshow('test', img)
cv2.waitKey(0)
img = converter.cv2_to_imgmsg(img, encoding='bgr8')
# time.sleep(5)
# before.publish(img)
cap.release()