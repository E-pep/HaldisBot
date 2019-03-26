#!/usr/bin/python2.7
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import cv2
from cv2 import aruco
from cv_bridge import CvBridge

# print(cv2.__version__)

bridge = CvBridge()

def callback(data):
    cv_img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    params = aruco.DetectorParameters_create()

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_img_pts = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    # img_marker = aruco.drawDetectedMarkers(img.copy(), rejected_img_pts)
    img_marker = aruco.drawDetectedMarkers(cv_img.copy(), corners, ids)

    title = "Markers read"
    # print(ids)
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1200, 900)
    cv2.imshow(title, img_marker)
    cv2.waitKey(1)

    width, height = img_marker.shape[:2]
    print(width, height)

    markers = []

    i = 0
    for marker in corners:
        print("marker", i)
        # The two opposing corner points
        pt1 = marker[0][0]
        pt2 = marker[0][2]
        print("point1", pt1[0], pt1[1])
        print("point2", pt2[0], pt2[1])
        center_x = (pt1[0] + pt2[0])/2
        center_y = (pt1[1] + pt2[1])/2
        center = [center_x, center_y]
        print(center)
        markers.append(center)

        i += 1

    print(markers)

    # pub.publish(ids[0])


# help(cv2.aruco)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
'''for i in range(4):
    img = aruco.drawMarker(aruco_dict, i, 500)

    title = "Marker " + str(i)
    cv2.imshow(title, img)
    cv2.waitKey(0)
    filename = "markers/marker" + str(i) + ".png"
    cv2.imwrite(filename, img)'''


pub = rospy.Subscriber("/usb_cam/image_raw", Image, callback)
pub = rospy.Publisher("/id_pub", Int32, queue_size=4)
rospy.init_node("aruco_det", anonymous=True)

rospy.spin()

cv2.destroyAllWindows()

