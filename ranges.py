import cv2
import numpy as np
from matplotlib import pyplot as plt
from detectors import inrange
import os
import sys
from image_processing import to_three

def nothing (x):
    pass


def callback(image_msg):
    frame = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    frame = cv2.cvtColor(frame, cv2.COLOR_YCrCb2RGB)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l1 = cv2.getTrackbarPos ("l1", "Colorbars")
    h1 = cv2.getTrackbarPos ("h1", "Colorbars")
    l2 = cv2.getTrackbarPos ("l2", "Colorbars")
    h2 = cv2.getTrackbarPos ("h2", "Colorbars")
    l3 = cv2.getTrackbarPos ("l3", "Colorbars")
    h3 = cv2.getTrackbarPos ("h3", "Colorbars")

    low_th  = (l1, l2, l3)
    high_th = (h1, h2, h3)

    inrange_filter.set_ths (low_th, high_th)

    mask = inrange_filter.apply (frame)

    enlighted = frame.copy ()
    enlighted [:, :, 0] = np.array (np.add (enlighted [:, :, 0], np.multiply (mask, 0.5)), dtype = np.uint8)

    #cv2.imshow ("enlighted", enlighted)
    #cv2.imshow ("mask", mask)

    result = np.concatenate ((enlighted, to_three (mask)), axis = 1)
    cv2.imshow ("result", result)

    os.system ('clear')    
    print (low_th, high_th)

    if (cv2.waitKey(1) & 0xFF == 27):
        break


cv2.namedWindow ('Colorbars')

cv2.createTrackbar ("l1", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h1", "Colorbars", 255, 255, nothing)
cv2.createTrackbar ("l2", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h2", "Colorbars", 255, 255, nothing)
cv2.createTrackbar ("l3", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h3", "Colorbars", 255, 255, nothing)

sub = rospy.Subscriber('/camera/image_raw_rhoban', Image, callback, queue_size=1)


low_th  = (57, 150, 110)
high_th = (67, 160, 120)

inrange_filter = inrange (low_th, high_th)
rospy.init_node('ranges')
rospy.spin()
cv2.destroyAllWindows()