import rospy
from sensor_msgs.msg import Image
import time

beforeImg = 0
timelast = time.time()
rospy.init_node('video_process', anonymous=True)
def imgCallback(data):
    global beforeImg
    global timelast
    before = rospy.Publisher('map_image/before', Image, queue_size=10)
    after = rospy.Publisher('map_image/after', Image, queue_size=10)
    if beforeImg != 0:
        before.publish(beforeImg)
    after.publish(data)
    beforeImg = data
    while time.time() - timelast < 1:
        pass
    timelast = time.time()
    for i in range(timelast):
        print(timelast)

rospy.Subscriber('/usb_cam/image_raw', Image, imgCallback)
rospy.spin()