#!usr/bin/python3 

import rclpy 
from rclpy.node import Node 

import cv2 
import numpy as np 

lower_red = np.array([0, 90, 128])
upper_red = np.array([180, 255, 255])

def object_detect(image):
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv_img, lower_red, upper_red)

    contours, hierarchy = \
        cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        if cnt.shape[0] < 150:
            continue 

        (x, y, w, h) = cv2.boundingRect(cnt)
        cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)

    cv2.imshow("object", image)
    cv2.waitKey(0)
    cv2.destoryAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = Node("node_object")
    node.get_logger().info("ROS node, to monitor the apple")

    image = cv2.imread("/home/mike/ros2_cool/01-ros-node/dev_ws/src/learning_node/learning_node/img.png")
    object_detect(image)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()