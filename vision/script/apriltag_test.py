import cv2
import time
import os
ccap = cv2.VideoCapture(0)
i = 0
# while 1:
#     success, img = ccap.read()
#     # cv2.imshow('f', frame)
#     # cv2.waitKey(0)
#     cv2.imwrite(os.path.split(os.path.realpath(__file__))[0] + '/img/test' + str(i) + '.jpg', img)
#     i += 1
#     time.sleep(2)

success, img = ccap.read()
time.sleep(1)
cv2.imwrite(os.path.split(os.path.realpath(__file__))[0] + '/test1.jpg', img)