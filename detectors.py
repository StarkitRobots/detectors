#!/usr/bin/env python
with_ros = True
basketball = True
obstacles = True
import image_processing
import cv2
import json

if with_ros:
    import rospy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String
    from geometry_msgs.msg import Point, Polygon
    from cv_bridge import CvBridge, CvBridgeError
    import cv2
    import numpy as np


#Filter is an img-to-img transformation; generally from any shape to any shape

class Filter:
    def __init__(self, name_):
        self.name = name_
    
    def apply (self, img):
        return img

#class tograyscale (Filter):
#    def __init__ (self):
#        pass
#
#    def apply (self, img):
#        return cv2.cvtColor (img, cv2.COLOR_BGR2GRAY)

class inrange (Filter):
    def __init__ (self, low_th_, high_th_):
        self.low_th  = low_th_
        self.high_th = high_th_

    def apply (self, img):
        return cv2.inRange (img, self.low_th, self.high_th)

#find bbox of the connected component with maximal area
class max_area_cc_bbox (Filter):
    def __init__ (self):
        pass

    def apply (self, img):
        return image_processing.find_max_bounding_box (img)

#returns bottom point of the bbox, middle by x axis
class bottom_bbox_point (Filter):
    def __init__ (self):
        pass

    def apply (self, img):
        tl, br = img

        x = int ((tl [0] + br [0]) / 2)
        y = br [1]

        return (x, y)

class find_obstacles_distances (Filter):
    def __init__ (self, ranges_):
        Filter.__init__ (self, "find_obstacles_distances")
        self.ranges = ranges_
        self.inrange_filter = inrange ((0, 0, 0), (255, 255, 255))
        self.cc_filter = filter_connected_components ()

    #def get_obstacles(img):
    #    smart_gray = 0.5 * img[:,:,2] + 0.5 * img[:,:,1]
    #    converted = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    #
    #    # white color mask
    #    lower = np.uint8([100, 100, 100])
    #    upper = np.uint8([120, 200, 200])
    #    binarized = cv2.inRange(converted, lower, upper)
    #
    #    op_ker = 12
    #    cl_ker = 12
    #    morph = binarized.astype('uint8')
    #    morph = cv2.morphologyEx(binarized, cv2.MORPH_OPEN, np.ones((op_ker, op_ker),np.uint8))
    #    morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, np.ones((cl_ker,cl_ker),np.uint8))
    #
    #    return morph

    def _get_obstacles_dists (self, obstacles):
        obstacles_flipped = cv2.flip (obstacles, 0)
        distances = np.argmax (obstacles_flipped, axis=0)

        #print ("fuck")
        #print (distances)

        return distances

    def apply (self, img):
        result = []
        labels = []

        sh = img.shape

        for i in range (sh [1]):
            labels.append (0)

        filled = False

        for range_num in range (len (self.ranges)):
            range_ = self.ranges [range_num]

            self.inrange_filter.set_ths (range_ [0], range_ [1])
            mask = self.inrange_filter.apply (img)
            mask = self.cc_filter.apply (mask, 10)

            cv2.imshow ("blyad", mask)

            op_ker = 12
            cl_ker = 12

            morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((op_ker, op_ker),np.uint8))
            morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, np.ones((cl_ker,cl_ker),np.uint8))

            temp_result = self._get_obstacles_dists (morph)

            if (filled == False):
                filled = True
                result = temp_result.copy ()

            for i in range (len (temp_result)):
                if (temp_result [i] != 0):
                    temp_result [i] = sh [0] - temp_result [i]

            for i in range (len (temp_result)):
                if (temp_result [i] <= result [i] and temp_result [i] != 0):
                    result [i] = temp_result [i]
                    labels [i] = range_num + 1

        #for i in range (sh [1]):
        #    result.append ((i, 200, i))

        #print ("ll")
        #print (labels)

        return result, labels


#should simply incapsulate basic processing function
#class filter_connected_components

#------------------------------------------------------

#Detector incapsulates the whole detection process, which practically means image processing
#to certain stage and consequent extraction of the required object

#Any detector (color-based, NN, template-based) is supposed to
#be set as a sequence of filters. The idea is obviously taken from NNs

class Detector:
    filters = []
    
    #processing stages (for debugging purposes)
    stages  = []

    def __init__(self):
        pass

    def __init__(self, detector_filename):
        with open (detector_filename) as f:
            data = json.load(f)
        competition = data["competition"]
        if with_ros:
            self._cv_bridge = CvBridge()
            if competition == "basketball":
                self._sub = rospy.Subscriber('/camera/image_raw_rhoban', Image, self.callback_basketball, queue_size=1)
                self.basket_top = rospy.Publisher('detectors/basket_top', Point, queue_size=1)
                self.basket_bottom = rospy.Publisher('detectors/basket_bottom', Point, queue_size=1)
                self.tennis_ball = rospy.Publisher('detectors/tennis_ball', Point, queue_size=1)
                self.basketball_img = rospy.Publisher('detectors/resulted_img', CompressedImage, queue_size=1)
            elif competition == "obstacles":
                self._sub = rospy.Subscriber('/camera/image_raw_rhoban', Image, self.callback_obstacles, queue_size=1)
                self.obstacles = rospy.Publisher('detectors/obstacles', Polygon, queue_size=1)
                self.obstacle_img = rospy.Publisher('detectors/resulted_img', CompressedImage, queue_size=1)



           
            
	
        with open (detector_filename) as f:
            data = json.load(f)
        
        for filter in data ["filters"]:
            filter_name = filter ["name"]

            if (filter_name == "inrange"):
                low_th   = (int (filter ["l1"]), int (filter ["l2"]), int (filter ["l3"]))
                high_th  = (int (filter ["h1"]), int (filter ["h2"]), int (filter ["h3"]))

                #print (low_th)

                new_filter = inrange (low_th, high_th)

            if (filter_name == "max_area_cc_bbox"):
                new_filter = max_area_cc_bbox ()

            if (filter_name == "bottom_bbox_point"):
                new_filter = bottom_bbox_point ()

            if (filter_name == "find_obstacles_distances"):
                types_num = int (filter ["types_num"])
                
                ranges = []

                for i in range (types_num):
                    type_num = str (i + 1)

                    low_th   = (int (filter [type_num + "l1"]),
                                int (filter [type_num + "l2"]),
                                int (filter [type_num + "l3"]))

                    high_th  = (int (filter [type_num + "h1"]),
                                int (filter [type_num + "h2"]),
                                int (filter [type_num + "h3"]))

                    ranges.append ((low_th, high_th))
                
                new_filter = find_obstacles_distances (ranges)

            self.add_filter (new_filter, filter ["name"])
    

            self.add_filter (new_filter, filter ["name"])
    
    def add_filter (self, new_filter, filter_name):
        self.filters.append ((new_filter, filter_name))
    
    def get_stages (self):
        return self.stages

    def detect(self, image):
        self.stages = []
        self.stages.append (image)
	
        for filter, name in self.filters:
            curr_state = filter.apply (self.stages [-1])
            self.stages.append (curr_state)

        return self.stages [-1]
    if with_ros:
        def callback_basketball(self, image_msg):
            try:
                frame = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
                frame = cv2.cvtColor(frame, cv2.COLOR_YCrCb2RGB)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            except CvBridgeError as e:
                print(e)
	        #print(frame.shape)
	        #cv2.imshow ("frame", frame)
            #top left, bottom right
            bbox_tl, bbox_br = detector.detect(frame)
            #print(bbox_tl, bbox_br)
            #calc basket top and bottom
            x_b = (bbox_br[0] + bbox_tl[0])/2
            y_b = bbox_br[1]
            x_t = (bbox_br[0] + bbox_tl[0])/2
            y_t = bbox_tl[1]

            #x, y = detector.detect (frame)

            #draw circle on the frame
            result = cv2.circle (frame.copy (), (int(x_b), int(y_b)), 5, (120, 150, 190), thickness = -1)
            print(frame.shape)
            cv2.waitKey(2)

            cv2.imshow ("frame", result)
            #print (x, y)

            img_msg = CompressedImage()
            img_msg.header.stamp = rospy.Time.now()
            img_msg.format = "jpeg"
            img_msg.data = np.array(cv2.imencode('.jpg', result)[1]).tostring()
         #  # Publish new image
            self.basketball_img.publish(img_msg)

            basketT_msg = Point(float(x_t), float(y_t), float(0))
            basketB_msg = Point(float(x_b), float(y_b), float(0))
            self.basket_top.publish(basketT_msg)
            self.basket_bottom.publish(basketB_msg)
            #stages = detector.get_stages ()

            #for i in range (2):
            #    cv2.imshow (str (i), stages[i])

        def callback_obstacles(self, image_msg):
                    # Now we can tune json parametrs while it running
                    #conf_file = rospy.get_param('~conf_file')
	                #detector = Detector(conf_file)
                    try:
                        frame = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
                        frame = cv2.cvtColor(frame, cv2.COLOR_YCrCb2RGB)
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                        
                    except CvBridgeError as e:
                        print(e)
                    #print(frame.shape)
                    #cv2.imshow ("frame", frame)
                    #top left, bottom right
                    bbox_tl, bbox_br = detector.detect (frame)
                    #print(bbox_tl, bbox_br)
                    #calc basket top and bottom
                    x_b = (bbox_br[0] + bbox_tl[0])/2
                    y_b = bbox_br[1]
                    x_t = (bbox_br[0] + bbox_tl[0])/2
                    y_t = bbox_tl[1]
                #draw bbox on the frame
                    #result = cv2.rectangle (frame.copy (), bbox_tl, bbox_br, (255, 0, 0), 5)
                    #frame = cv2.cvtColor(frame, cv2.COLOR_YCR_CB2HSV)
                    #frame  = cv2.cvtColor(frame, cv2.COLOR_YCrCb2RGB)
                    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    #bottom point coordinates
                    #x, y = detector.detect (frame)

                    #draw circle on the frame
                    result = cv2.circle (frame.copy (), (int(x_b), int(y_b)), 5, (120, 150, 190), thickness = -1)
                    print(frame.shape)
                    cv2.waitKey(2)

                    cv2.imshow ("frame", result)
                    #print (x, y)

                    img_msg = CompressedImage()
                    img_msg.header.stamp = rospy.Time.now()
                    img_msg.format = "jpeg"
                    img_msg.data = np.array(cv2.imencode('.jpg', result)[1]).tostring()
                #  # Publish new image
                    self.obstacle_img.publish(img_msg)
                    basketT_msg = Point(float(x_t), float(y_t), float(0))
                    basketB_msg = Point(float(x_b), float(y_b), float(0))
                    self.basket_top.publish(basketT_msg)
                    self.basket_bottom.publish(basketB_msg)
                    self.obstacles.publish(tuple([basketB_msg, basketT_msg]))

                    #stages = detector.get_stages ()

                    #for i in range (2):
                    #    cv2.imshow (str (i), stages[i])
	
if __name__ == "__main__":
    rospy.init_node('detectors')
    conf_file = rospy.get_param('~conf_file')
    detector = Detector(conf_file)
    rospy.spin()
