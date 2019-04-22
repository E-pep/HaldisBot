#!/usr/bin/python2.7
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import cv2
from cv2 import aruco
from cv_bridge import CvBridge

# print(cv2.__version__)

bridge = CvBridge()


def callback(data):
    # Convert to cv image
    cv_img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    params = aruco.DetectorParameters_create()

    # Convert the image to gray scale and detect and draw ArUco markers
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_img_pts = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    img_marker = aruco.drawDetectedMarkers(cv_img.copy(), corners, ids)

    # Get the width and height of the image
    height, width = img_marker.shape[:2]
    # print(width, height)

    # Compute the centers of the detected markers
    markers = []
    if ids is not None:
        for marker, id in zip(corners, ids):
            # The two opposing corner points: top left and bottom right
            pt1 = marker[0][0]
            pt2 = marker[0][2]
            # print("point1", pt1[0], pt1[1])
            # print("point2", pt2[0], pt2[1])

            # Average those two points to get the center of the marker
            center_x = (pt1[0] + pt2[0])/2
            center_y = (pt1[1] + pt2[1])/2
            # Convert them relative to the image size
            center_x_rel = round(center_x/width, 3)
            center_y_rel = round(center_y/height, 3)

            # Save every marker's center and draw it
            markers.append([id[0], center_x_rel, center_y_rel])
            radius = int(abs(pt1[0] - pt2[0])/10)
            cv2.circle(img_marker, (int(center_x), int(center_y)), radius, (0, 0, 255))

        # Select the marker that's closest to the center
        most_centered = [200, 1, 1]
        for marker in markers:
            # print(marker)
            if abs(marker[1] - 0.5) < abs(most_centered[1] - 0.5):
                most_centered = marker

        # print("Most centered", most_centered)
        # Publish the most centered marker: id, rel_x and rel_y
        to_publish = Float32MultiArray(data=most_centered)
        pub.publish(to_publish)

    # Show the image with the detected ArUco markers
    title = "Markers read"
    # print(ids)
    #cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    #cv2.resizeWindow(title, width*2, height*2)
    #cv2.imshow(title, img_marker)
    cv2.waitKey(1)


# help(cv2.aruco)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
# Create the markers
'''for i in range(4):
    img = aruco.drawMarker(aruco_dict, i, 500)

    title = "Marker " + str(i)
    cv2.imshow(title, img)
    cv2.waitKey(0)
    filename = "markers/marker" + str(i) + ".png"
    cv2.imwrite(filename, img)'''


pub = rospy.Subscriber("/usb_cam/image_raw", Image, callback)
pub = rospy.Publisher("/id_pub", Float32MultiArray, queue_size=4)
rospy.init_node("aruco_det", anonymous=True)

rospy.spin()

cv2.destroyAllWindows()

