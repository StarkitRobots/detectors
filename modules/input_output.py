import cv2

def get_available_cameras (upper_bound = 10, lower_bound = 0):
    available = []
    
    for i in range (lower_bound, upper_bound):
        cap = cv2.VideoCapture (i)
    
        if (cap.isOpened ()):
            available.append (i)
    
        cap.release ()
    
    return available

#read dir

#CAMERA = 0
#VIDEO  = 1
#PHOTO  = 2

#Incapsulates reading frames from the following types of sources:
#photo (single image), photos series, video, camera, ROS node (?)

#If an input file or camera number is given, desired input type
#can be omitted. Numeric value given in initialization is considered
#as a number of camera desired to be opened. Value "-1" opens camera
#with minimal available id, "-2" - with maximal.

#TODO: semi-automatic source type detection
#TODO: implementation of reading from all the sources (except for ROS) [later]
#TODO: output [later]

#"photo series" : self.read_frame_photo (),
#"video"        : self.read_frame_photo (),
#"camera"       : self.read_frame_photo (),
#"ros flex"     : self.read_frame_photo (),
# }

class Source:
    #type = ""
    #path = ""

    #Taking sample image requires reading, in case of non-constant
    #sources like camera or video it can lead to a loss of a single
    #frame. These is the fix :)
    sample_image_obtained          = False
    sample_image_incoherency_fixed = False

    def __init__ (self, path_, type_ = "", instant_init = True):
        self.path = path_
        
        if (type_ == ""):
            if (self.path.endswith ("jpg") or
                self.path.endswith ("png") or
                self.path.endswith ("bmp")):
                self.type = "photo"

            #if (ends with mp4, webm) video
            #    self.type = "video"

            #if (ends with /)         photo_series
            #    self.type = "photo series"

            #if (is number)           camera
            #    self.type = "camera"

            #    num = str (path_)
            #    if (num < 0):
            #        cameras = get_available_cameras ()

            #        if (num == -1):
            #            cam_num = min (cameras)

            #        if (num == -1):
            #            cam_num = min (cameras)

        else:
            self.type = type_

        if (instant_init == True):
            self.init_source ()

    def shape (self):
        return self.sample_image ().shape

    def sample_image (self):
        if (self.sample_image_obtained == False):
            self.sample_image = self.get_frame ()

            self.sample_image_obtained = True

        return self.sample_image

    def init_source (self):
        self.sources = {}
        self.sources.update ({"photo" : (self.init_photo, self.get_frame_photo)})

        self.sources [self.type] [0] ()

    def init_photo (self):
        self.img = cv2.imread (self.path)

    def get_frame (self):
        if (self.sample_image_obtained          == True and
            self.sample_image_incoherency_fixed == False):
            self.sample_image_incoherency_fixed = True
            
            return self.sample_image

        return self.sources [self.type] [1] ()

    def get_frame_photo (self):
        return self.img.copy ()

#output (stream to video file)