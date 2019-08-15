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
    
    #source = Source ("/Users/elijah/Dropbox/Programming/detectors/images/2019_08_11_08h11m07s/")
    source  = Source ("/Users/elijah/Dropbox/Programming/detectors/data/output.avi")

    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
    a = 0

    while (True):
        frame = source.get_frame ()
        
        #out.write(frame)
        #print (a)
        #a += 1
        #if (a > 49):
        #    break

        (x, y), success  = detector.detect (frame, "ball detector")

        #result = frame.copy ()

        #draw circle on the frame
        if (success == True):
            print ("detected")
            
            #result = cv2.circle (result, (x, y), 9, (120, 15, 190), thickness = -1)
            
        else:
            print ("not detected")

        stages = detector.get_stages_picts ("ball detector")
	
        for i in range (len (stages)):
            cv2.imshow (str (i), stages [i])

	#resultant_frame = form_images (processing_stages)
        
        #cv2.imshow ("frame", result)

        time.sleep (0.02)

        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    out.release ()

    #cam.release ()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()