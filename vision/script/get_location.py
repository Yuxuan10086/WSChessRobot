import numpy as np
import cv2
import os
import apriltag

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
    raise(Exception('方向数字不合法'))

def corner_detector(img):
    # 从左上顺时针ID为 0 1 2 3
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11') )
    tags = at_detector.detect(gray)
    tags.sort(key = lambda tag : tag.tag_id)
    print(tags)

def order_points(pts): # 传入四点列表 从左上起顺时针
    rect = np.zeros((4, 2), dtype = "float32")
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
        [0, maxHeight - 1]], dtype = "float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped

def cut(img1, img2):
    # 取两图片中最小的尺寸,将较大的图片从左上裁剪至此尺寸,返回裁剪后的两图
    size_x = img1.shape[0]
    size_y = img1.shape[1]
    if img2.shape[0] < size_x:
        size_x = img2.shape[0]
    if img2.shape[1] < size_y:
        size_y = img2.shape[1]
    img1 = img1[0 : size_x, 0 : size_y]
    img2 = img2[0 : size_x, 0 : size_y]
    return img1, img2

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
    # cv2.imshow("hh", img)
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
    center[1] = int(M['m10'] / M['m00'])
    center[0] = int(M['m01'] / M['m00'])
    return center

def get_location(img1, img2):
    # 1为落子后,2为落子前
    img1, img2 = cut(img1, img2)
    res = preprocess(img1, img2)
    res, contours = draw_contour(res)
    center = find(contours, res)
    # 测试代码
    print(center)
    res[center[0], center[1]] = 255
    cv2.imshow("res", res)
    cv2.waitKey(0)
    return center

file_url = os.path.split(os.path.realpath(__file__))[0] + '/test.png'
test = cv2.imread(file_url)
corner_detector(test)
# test = four_point_transform(test, np.array([(61, 251), (309, 114), (376, 259), (81, 408)]))
# cv2.imshow('test', test)
# cv2.waitKey(0)

# print(compare_corner([(0, 0), (1, 0), (1, 1), (0, 1)], 4))
