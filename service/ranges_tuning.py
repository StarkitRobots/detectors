#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
import sys
#from image_processing import to_three
import cv2
import numpy as np

#import rospy
#from sensor_msgs.msg import Image, CompressedImage
#from std_msgs.msg import String
#from geometry_msgs.msg import Point, Polygon
#from cv_bridge import CvBridge, CvBridgeError

sys.path.append("../modules/")

from input_output import Source
import detectors

def nothing (x):
    pass

#def callback(image_msg):
    #frame = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    #frame = cv2.cvtColor(frame, cv2.COLOR_YCrCb2RGB)
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


cv2.namedWindow ('Colorbars')

#source = Source ("../images/2019_08_11_08h00m33s/00014.png")
#source = Source ("../images/00014.png")
#source = Source ("../images/obst_bottom.png")
source = Source ("1")

#detector = detectors.Detector ('../configs/multiple_objects1.json')
#detector = detectors.Detector ('../configs/closest_obstacle.json')

low_th  = (140, 70, 40)
high_th = (220, 130, 100)
detector = detectors.Detector ()

detector.add_filter (detectors.colorspace_to_colorspace ("RGB", "HSV"), "a", "colorspace")
detector.add_filter (detectors.inrange (low_th, high_th), "a", "inrange")

#detector.add_filter (detectors.filter_connected_components (10), "a", "filter")

detector.add_filter (detectors.leave_max_area_cc (), "a", "max cc extraction")
detector.add_filter (detectors.bottom_cc_point (), "a", "bottom point extraction")

#detector.add_filter (detectors.bottom_bbox_point (), "a", "desired point extraction")

cv2.createTrackbar ("l1", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h1", "Colorbars", 255, 255, nothing)
cv2.createTrackbar ("l2", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h2", "Colorbars", 255, 255, nothing)
cv2.createTrackbar ("l3", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h3", "Colorbars", 255, 255, nothing)

#sub = rospy.Subscriber('/camera/image_raw_rhoban', Image, callback, queue_size=1)

low_th  = (57, 150, 110)
high_th = (67, 160, 120)

#inrange_filter = inrange (low_th, high_th)
#rospy.init_node('ranges')
#rospy.spin()
#cv2.destroyAllWindows()

while (True):    
    _, frame_from_source = source.get_frame ()

    #print ("a")

    detector.detect (frame_from_source, "a")
    stages = detector.get_stages_picts ("a")
    #detector.detect (frame_from_source, "ball detector")
    #stages = detector.get_stages_picts ("ball detector")

    l1 = cv2.getTrackbarPos ("l1", "Colorbars")
    h1 = cv2.getTrackbarPos ("h1", "Colorbars")
    l2 = cv2.getTrackbarPos ("l2", "Colorbars")
    h2 = cv2.getTrackbarPos ("h2", "Colorbars")
    l3 = cv2.getTrackbarPos ("l3", "Colorbars")
    h3 = cv2.getTrackbarPos ("h3", "Colorbars")

    low_th  = (l1, l2, l3)
    high_th = (h1, h2, h3)

    detector.detectors ["a"] [1] [0].set_ths (low_th, high_th)

    for i in range (len (stages) - 1):
        cv2.imshow (str (i), stages [i])

    cv2.waitKey(2)
