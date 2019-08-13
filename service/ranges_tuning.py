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
source = Source ("../images/obst_bottom.png")

#detector = detectors.Detector ('../configs/multiple_objects1.json')
detector = detectors.Detector ('../configs/closest_obstacle.json')

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
    frame_from_source = source.get_frame ()

    #print ("a")

    detector.detect (frame_from_source, "obstacle detector")
    stages = detector.get_stages_picts ("obstacle detector")
    #detector.detect (frame_from_source, "ball detector")
    #stages = detector.get_stages_picts ("ball detector")

    for i in range (len (stages)):
        cv2.imshow (str (i), stages [i])

    cv2.waitKey(2)
