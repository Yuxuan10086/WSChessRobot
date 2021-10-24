import cv2
import numpy as np
# 只能识别黑子

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
    ''''莫名出bug且过于耗时，改用其他方法
       distance = -1000
       for i in range(img.shape[0]):
           for j in range(img.shape[1]):
               if cv2.pointPolygonTest(contours, (i, j), 0) > 0:
                   distance_this = cv2.pointPolygonTest(contours, (i, j), 1)
                   # print(distance_this)
                   if distance_this > distance:
                       distance = distance_this
                       center[1] = i
                       center[0] = j
       '''
    M = cv2.moments(contours)
    center[1] = int(M['m10'] / M['m00'])
    center[0] = int(M['m01'] / M['m00'])
    return center

def get_location(url1, url2):
    # 1为落子后,2为落子前
    img1 = cv2.imread(url1)
    img2 = cv2.imread(url2)
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

# for i in range(4):
#     get_location('test (' + str(i + 3) + ').jpg', 'test (' + str(i + 2) + ').jpg')