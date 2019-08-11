import cv2
import time
import os
import math
import sys

sys.path.append("/Users/elijah/Dropbox/Programming/detectors/modules/")

from input_output import Source
import detectors

photo_path    = "/Users/elijah/Dropbox/Programming/detectors/images/"
photo_file    = "basket/2.jpg"
obstacle_file = "obstacle_only.jpg"

def main ():    
    detector   = detectors.Detector ('/Users/elijah/Dropbox/Programming/detectors/configs/object_tracking.json')
    
    img_source      = Source (photo_path + photo_file, "", True)
    obstacle_source = Source (photo_path + obstacle_file)
    
    img_sh  = img_source.shape ()
    obst_sh = obstacle_source.shape ()

    x_obs_rot = img_sh [1] // 2
    y_obs_rot = img_sh [0] // 2
    radius = x_obs_rot // 2
    angle = 0

    while (True):
        frame    = img_source.get_frame      ()
        obstacle = obstacle_source.get_frame ()

        x_obs = x_obs_rot + int (radius * math.cos (angle))
        y_obs = y_obs_rot + int (radius * math.sin (angle))

        angle += 0.1

        print (obst_sh)

        print (frame [x_obs : x_obs + obst_sh [1],
               y_obs : y_obs + obst_sh [0], :].shape)

        frame [x_obs : x_obs + obst_sh [1],
               y_obs : y_obs + obst_sh [0], :] = obstacle

        #frame = frame_
                
        cv2.waitKey (1)    
        os.system ('clear')
        
        (x, y), success = detector.detect (frame, "obstacle detector")

        #draw circle on the frame
        if (success == True):
            print ("detected")
            
            result = cv2.circle (result, (x, y), 9, (120, 15, 190), thickness = -1)
            
        else:
            print ("not detected")

        stages = detector.get_stages ()
	
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