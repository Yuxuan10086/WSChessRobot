import apriltag
import cv2
import time
import os
ccap = cv2.VideoCapture(0)
while 1:
    success, frame = ccap.read()
    # cv2.imshow('f', frame)
    # cv2.waitKey(0)
    cv2.imwrite(os.path.split(os.path.realpath(__file__))[0] + '/test7.jpg', frame)
    break
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
    tags = at_detector.detect(frame)
    for tag in tags:
        print(tag.tag_id)
    time.sleep(0.1)