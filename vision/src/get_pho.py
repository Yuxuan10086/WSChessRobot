import cv2
import numpy as np
cap_video=cv2.VideoCapture(1)
k=0
while(cap_video.isOpened()):
    ret,frame=cap_video.read()
    if ret == True:
        cv2.imshow('frame',frame)
    cv2.waitKey(1)
    k=k+1
    if k>1000:
        break
    else:
        if k%1000:
            s=str(k)
    cv2.imwrite('video_image'+ s +".jpg",frame)
    cap_video.release()
    cv2.waitKey(0)
# cv2.destroyAllWindow()
