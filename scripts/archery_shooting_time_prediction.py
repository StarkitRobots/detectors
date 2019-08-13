import cv2
import time
import os
import math
import sys

sys.path.append("/Users/elijah/Dropbox/Programming/detectors/modules/")

from input_output import Source
import detectors
import tracker

photo_path    = "/Users/elijah/Dropbox/Programming/detectors/images/"
photo_file    = "basket/2.jpg"
obstacle_file = "obstacle_only.jpg"

def main ():    
    detector   = detectors.Detector ('/Users/elijah/Dropbox/Programming/detectors/configs/object_tracking.json')
    
    img_source      = Source (photo_path + photo_file, "", True)
    obstacle_source = Source (photo_path + obstacle_file)
    
    target_tracker = tracker.Tracker ()

    img_sh  = img_source.shape ()
    obst_sh = obstacle_source.shape ()

    print (img_sh, obst_sh)

    x_obs_rot = int (img_sh [1] / 2)
    y_obs_rot = int (img_sh [0] / 2)
    obs_radius = int (x_obs_rot / 3)

    #print (x_obs_rot, y_obs_rot, radius)

    angle = 0
    angular_speed = 0.231415234567654

    turn_num = 0

    while (True):
        frame    = img_source.get_frame      ()
        obstacle = obstacle_source.get_frame ()

        x_obs = x_obs_rot + int (obs_radius * math.cos (angle))
        y_obs = y_obs_rot + int (obs_radius * math.sin (angle))

        angle += angular_speed

        frame [y_obs : y_obs + obst_sh [0],
               x_obs : x_obs + obst_sh [1], :] = obstacle

        #frame = frame_
                
        cv2.waitKey (1)    
        os.system ('clear')
        
        (x, y), success  = detector.detect (frame, "obstacle detector")
        measurement_time = time.time ()

        #print (x, y, measurement_time)

        target_tracker.add_measurement ((x, y), measurement_time)

        result = frame.copy ()

        if (turn_num > 10):
            target_tracker.calc_cycle_parameters ()

            (center_x, center_y) = target_tracker.cycle_parameters ["circle center"]
            radius = target_tracker.cycle_parameters ["radius"]

            lowest_x = int (center_x)
            lowest_y = int (center_y + radius)

            result = cv2.circle (result, (lowest_x, lowest_y), 9, (20, 150, 190), thickness = -1)

            time_to_approach = target_tracker.predict_time ((lowest_x, lowest_y))

            if (time_to_approach < 0.2):
                result = cv2.circle (result, (lowest_x, lowest_y), 15, (220, 15, 90), thickness = -1)

            print (time_to_approach)

        turn_num += 1

        #draw circle on the frame
        if (success == True):
            #print ("detected")
            
            result = cv2.circle (result, (x, y), 9, (120, 15, 190), thickness = -1)
            
        else:
            print ("not detected")

        stages = detector.get_stages_picts ("obstacle detector")
	
        for i in range (2):
            cv2.imshow (str (i), stages [i])

        #processing_stages = detector.stages ()
	
	#resultant_frame = form_images (processing_stages)
        
        cv2.imshow ("frame", result)

        time.sleep (0.02)

        #clear_output (wait=True)
        
        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    cam.release ()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()