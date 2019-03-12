#!/usr/bin/python3
import rospy
import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

print("Awel")
print(cv2.__version__)

#help(cv2.aruco)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
for i in range(4):
    img = aruco.drawMarker(aruco_dict, i, 500)

    title = "Marker " + str(i)
    cv2.imshow(title, img)
    cv2.waitKey(0)
    filename = "markers/marker" + str(i) + ".png"
    cv2.imwrite(filename, img)



params = aruco.DetectorParameters_create()
img = cv2.imread("/home/wout/Pictures/arucotest.jpg")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=params)
#img_marker = aruco.drawDetectedMarkers(img.copy(), rejectedImgPoints)
img_marker = aruco.drawDetectedMarkers(img.copy(), corners, ids)

title = "Markers read"
print(ids)
cv2.namedWindow(title, cv2.WINDOW_NORMAL)
cv2.resizeWindow(title, 1200,900)
cv2.imshow(title, img_marker)
cv2.waitKey(0)

cv2.destroyAllWindows()