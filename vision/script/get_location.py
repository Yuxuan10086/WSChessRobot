#!/usr/bin/env python3
from ctypes import sizeof
import numpy as np
import cv2
import os
import apriltag
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# 手机直拍图像过大，需压缩后方可使用，照片注意拍摄方向

def compare_corner(corner, direction): # 传入四个角点列表与方向 从左上顺时针0123
    sum = [-1]
    order_corner = []
    for point in corner:
        index = 0
        while sum[index] > point[0] + point[1]:
            index += 1
        sum.insert(index, point[0] + point[1])
        order_corner.insert(index, point)
    del sum[len(sum) - 1]
    if direction == 0:
        return order_corner[3]
    if direction == 2:
        return order_corner[0]
    if order_corner[1][0] > order_corner[2][0]:
        if direction == 1:
            return order_corner[1]
        elif direction == 3:
            return order_corner[2]
    else:
        if direction == 1:
            return order_corner[2]
        elif direction == 3:
            return order_corner[1]
    print(sum, order_corner)
    rospy.loginfo('方向数字不合法')

def corner_detector(img):
    # 从左上顺时针ID为 0 1 2 3
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11') )
    tags = at_detector.detect(gray)
    tags.sort(key = lambda tag : tag.tag_id)
    try:
        return [compare_corner(tags[0].corners, 2),
            compare_corner(tags[1].corners, 3),
            compare_corner(tags[2].corners, 0),
            compare_corner(tags[3].corners, 1)]
    except(IndexError):
        rospy.INFO('tag无法识别')

def order_points(pts): # 传入四点列表 从左上起顺时针
    pts = np.asarray(pts)
    rect = np.zeros((4, 2), dtype = 'float32')
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def four_point_transform(image, pts):
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = 'float32')
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped

def cut(img1, img2):
    size_y, size_x = img2.shape[:2]
    img1 = cv2.resize(img1, (size_x, size_y), interpolation=cv2.INTER_CUBIC)
    return img1, img2, size_x, size_y

def preprocess(img1, img2):
    # 通过作差,通道转换,二值化,腐蚀,膨胀操作得到黑底白圆的图片
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    thre, img1 = cv2.threshold(img1, 50, 255, cv2.THRESH_BINARY)
    thre, img2 = cv2.threshold(img2, 50, 255, cv2.THRESH_BINARY)
    res = (img2 - img1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    kerne2 = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    res = cv2.erode(res, kernel)  # 腐蚀
    res = cv2.dilate(res, kerne2)
    return res

def draw_contour(img):
    # 传入黑底白圆图片,保留圆的轮廓将其填充为黑色,返回处理后的图像与轮廓坐标列表
    contours, hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    res = np.ones((img.shape[0], img.shape[1]), dtype=np.uint8)
    # cv2.imshow('hh', img)
    # cv2.waitKey(0)
    if len(contours) != 1:
        # 暂时用比较面积并选取面积最大的轮廓代替形状大小判断
        area_max = 0
        area_max_indxes = 0
        for j in range(len(contours)):
            area = cv2.contourArea(contours[j])
            if area > area_max:
                area_max = area
                area_max_indxes = j
        contours = [contours[area_max_indxes]]
        # print(contours)
    for i in contours[0]:
        try:
            res[i[0][1], i[0][0]] = 255
        except:
            pass
    return res, contours[0]

def find(contours, img):
    # 传入轮廓坐标数组与图像,遍历图像找到与轮廓距离最大的点,即为几何中心
    center = [0, 0]
    M = cv2.moments(contours)
    center[0] = int(M['m10'] / M['m00']) # 横坐标
    center[1] = int(M['m01'] / M['m00'])
    return center

def get_location(img1, img2):
    # 1为落子后,2为落子前
    img1, img2, size_x, size_y = cut(img1, img2)
    res = preprocess(img1, img2)
    res, contours = draw_contour(res)
    center = find(contours, res)
    # 测试代码
    # print(center)
    # cv2.circle(res, center, 5, (255, 0, 0))
    # cv2.imshow('res', res)
    # cv2.waitKey(0)
    return center, size_x, size_y

def test():
    file_url = os.path.split(os.path.realpath(__file__))[0]
    after = cv2.imread(file_url + '/test16.jpg')
    before = cv2.imread(file_url + '/test12.jpg')
    before = four_point_transform(before, corner_detector(before))
    after = four_point_transform(after, corner_detector(after))
    center, size_x, size_y = get_location(after, before)
    center[1] = round(center[1] / (size_x / 6))
    center[0] = round(center[0] / (size_y / 9))
    print(center, size_x, size_y)
    cv2.imshow('after', before)
    cv2.waitKey(0)

def test2():
    i = 1
    id = 0
    cap = cv2.VideoCapture(id)
    sucess, img = cap.read()
    cv2.imshow('test', img)
    cv2.waitKey(0)
    cap.release()
    cv2.destroyWindow('test')
    while 1:
        input('wait')
        cap = cv2.VideoCapture(id)
        sucess, img = cap.read()
        cap.release()
        if i % 2 == 1:
            before = img 
        else:
            before = four_point_transform(before, corner_detector(before))
            after = four_point_transform(img, corner_detector(img))
            center, size_x, size_y = get_location(after, before)
            print(center, size_x, size_y)
            center[0] = round(center[0] / (size_x / 7))
            center[1] = round(center[1] / (size_y / 7))
            print(center)
        i += 1


def main():
    rospy.init_node('get_location')
    beforeImg = 0
    transformer = cv_bridge.CvBridge()
    pubCenter = rospy.Publisher('human', Point, queue_size = 10)
    column = rospy.get_param('column') - 1
    row = rospy.get_param('row') - 1
    def beforeCallback(img):
        global before
        before = transformer.imgmsg_to_cv2(img)
    def afterCallback(img):
        global before
        after = transformer.imgmsg_to_cv2(img)
        before = four_point_transform(before, corner_detector(before))
        after = four_point_transform(after, corner_detector(after))
        center, size_x, size_y = get_location(after, before)
        center[0] = round(center[0] / (size_x / column))
        center[1] = round(center[1] / (size_y / row))
        if center[0] >= row or center[0] <= 0 or center[1] >= column or center[1] <= 0:
            rospy.loginfo('非法落子位置')
        pointCenter = Point()
        pointCenter.x = center[0]
        pointCenter.y = center[1]
        pubCenter.publish(pointCenter)

    rospy.Subscriber('/map_image/before', Image, beforeCallback)
    rospy.Subscriber('/map_image/after', Image, afterCallback)
    rospy.spin()

# 棋子位置识别模块需在棋盘内同时增加一枚黑子和一枚白子时识别黑子的位置

if __name__ == '__main__':
    main()
    # test2()