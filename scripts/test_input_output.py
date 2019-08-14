import cv2
import time
import os
import math
import sys

sys.path.append("/Users/elijah/Dropbox/Programming/detectors/modules/")

from input_output import Source
import detectors
import tracker

def main ():    
    detector = detectors.Detector ('/Users/elijah/Dropbox/Programming/detectors/configs/basketball.json')
    
    series_source = Source ("/Users/elijah/Dropbox/Programming/detectors/images/2019_08_11_08h11m07s/", "", True)

    while (True):
        frame    = series_source.get_frame      ()
        
        (x, y), success  = detector.detect (frame, "ball detector")

        result = frame.copy ()

        #draw circle on the frame
        if (success == True):
            #print ("detected")
            
            result = cv2.circle (result, (x, y), 9, (120, 15, 190), thickness = -1)
            
        else:
            print ("not detected")

        stages = detector.get_stages_picts ("ball detector")
	
        for i in range (len (stages)):
            cv2.imshow (str (i), stages [i])

	#resultant_frame = form_images (processing_stages)
        
        cv2.imshow ("frame", result)

        time.sleep (0.02)

        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    cam.release ()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()