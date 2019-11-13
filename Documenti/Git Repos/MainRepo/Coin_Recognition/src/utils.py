#! /usr/bin/env python

import cv2
import numpy as np
import os
import rospy

def resize_image(image, w=800, h=None):
    if w is not None and h is not None:
        return cv2.resize(image, (w, h))
    elif h is None:
        ratio = w / float(image.shape[1])
        return cv2.resize(image, (w, int(ratio * image.shape[0])))
    elif w is None:
        ratio = h / float(image.shape[0])
        return cv2.resize(image, (int(ratio * image.shape[1], h)))
    else:
        print("[ERROR] Insert either preferred height, preferred width or both.")

def nothing(x):
    pass

def transform_color_space(image, color_space):
    #change image's color space and return image
    if color_space == "HSV":
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    elif color_space == "LAB":
        image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    #OpenCV imread loads in BGR so no need for that conversion
    return image

def trackbar_mask_ranges(image):
    image = resize_image(image, w=800)
    imageHeight, imageWidth = image.shape[:2]
    color_space = "HSV"

    window_name = "%s image"%(color_space)
    cv2.namedWindow(window_name)

    #extract color components of color space
    (range1, range2, range3) = ([0, 179], [0, 255], [0, 255])
    component1, component2, component3 = list(color_space)

    u1 = "Upper %s"%(component1)
    l1 = "Lower %s"%(component1)
    u2 = "Upper %s"%(component2)
    l2 = "Lower %s"%(component2)
    u3 = "Upper %s"%(component3)
    l3 = "Lower %s"%(component3)

    #create trackbars for color components/channels
    cv2.createTrackbar(l1, window_name, range1[0], range1[1], nothing)
    cv2.createTrackbar(u1, window_name, range1[1], range1[1], nothing)
    cv2.createTrackbar(l2, window_name, range2[0], range2[1], nothing)
    cv2.createTrackbar(u2, window_name, range2[1], range2[1], nothing)
    cv2.createTrackbar(l3, window_name, range3[0], range3[1], nothing)
    cv2.createTrackbar(u3, window_name, range3[1], range3[1], nothing)

    #resize image while keeping same ratio and convert to HSV color space 
    image = cv2.resize(image, (imageWidth, imageHeight))
    new_color_space = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    while True:
        pos_l1 = cv2.getTrackbarPos(l1, window_name)
        pos_u1 = cv2.getTrackbarPos(u1, window_name)
        pos_l2 = cv2.getTrackbarPos(l2, window_name)
        pos_u2 = cv2.getTrackbarPos(u2, window_name)
        pos_l3 = cv2.getTrackbarPos(l3, window_name)
        pos_u3 = cv2.getTrackbarPos(u3, window_name)

        #array for final values
        final_lower = np.array([pos_l1, pos_l2, pos_l3])
        final_upper = np.array([pos_u1, pos_u2, pos_u3])

        #create mask
        mask = cv2.inRange(new_color_space, final_lower, final_upper)

        output = cv2.bitwise_and(image, image, mask=mask)

        cv2.imshow(window_name, output)
        key = cv2.waitKey(500) & 0xFF
        if key == ord('q'):
            break

    print("Lower Boundaries for %s Color Space: %s"%(color_space,
                                        final_lower))
    print("Upper Boundaries for %s Color Space: %s"%(color_space,
                                        final_upper))
    cv2.destroyAllWindows()
    return (final_lower, final_upper)

#initialize node and image file path 
rospy.init_node("coin_detection_node")
current_path = "/home/valeriofranchi/catkin_ws/src/assignment3"
image_file_path =  current_path + rospy.get_param('~image')

#load image
img = cv2.imread(image_file_path)

#tests
trackbar_mask_ranges(img)



