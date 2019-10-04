import math
import numpy as np
from scipy import optimize
import cv2

def detect_circle_params(x_coords, y_coords):
    '''
    Calculates geom params of point list by solving the nonlinear equasion.
    https://scipy-cookbook.readthedocs.io/items/Least_Squares_Circle.html
   
    Method: leastsq with jacobian
   
    return: X, Y, R, error
    '''
    def calc_R(xc, yc):
        return np.sqrt((x_coords-xc)**2 + (y_coords-yc)**2)
 
    def f_2b(c):
        Ri = calc_R(*c)
        return Ri - Ri.mean()
 
    def Df_2b(c):
        xc, yc     = c
        df2b_dc    = np.empty((len(c), len(x_coords)))
 
        Ri = calc_R(xc, yc)
        df2b_dc[0] = (xc - x_coords)/Ri                   # dR/dxc
        df2b_dc[1] = (yc - y_coords)/Ri                   # dR/dyc
        df2b_dc    = df2b_dc - df2b_dc.mean(axis=1)[:, np.newaxis]
 
        return df2b_dc
 
    x_m = np.mean(x_coords)
    y_m = np.mean(y_coords)
    center_estimate = x_m, y_m
    center_2b, ier = optimize.leastsq(f_2b, center_estimate, Dfun=Df_2b, col_deriv=True)
 
    xc_2b, yc_2b = center_2b
    Ri_2b        = calc_R(*center_2b)
    R_2b         = Ri_2b.mean()
    residu_2b    = sum((Ri_2b - R_2b)**2)
 
    return xc_2b, yc_2b, R_2b, residu_2b
 
def calculate_period(positions, timestamps): #measurments):
    def dotproduct(v1, v2):
        return sum((a*b) for a, b in zip(v1, v2))
 
    def length(v):
        return math.sqrt(dotproduct(v, v))
 
    def angle_b2v(v1, v2):
        cos = dotproduct(v1, v2) / (length(v1) * length(v2))
        angle = 0 if cos > 1 else math.pi if cos < -1 else math.acos(cos)
        return angle if angle > 0 else angle + 2 * math.pi

    def compute_angle(x1, y1, x2, y2, x_circle, y_circle):
        x1_d = x1 - x_circle
        y1_d = y1 - y_circle
        x2_d = x2 - x_circle
        y2_d = y2 - y_circle
        return angle_b2v((x1_d, y1_d), (x2_d, y2_d))
 
    #sort by time
    #measurments = sorted(measurments, key=lambda x: x[2])
    #x_coords, y_coords, time_stamps = zip(*measurements)

    time_stamps = timestamps
    x_coords, y_coords = zip (*positions)

    measurements = list (zip (x_coords, y_coords, timestamps))
    #print (measurements)
 
    circle_x, circle_y, circle_r, circle_error = detect_circle_params(x_coords, y_coords)
#    circle_x, circle_y, circle_r = (np.max(x_coords) + np.min(x_coords))/2, \
#                                   (np.max(y_coords) + np.min(y_coords))/2, \
#                                   ((np.max(x_coords) - np.min(x_coords) + \
#                                     np.max(y_coords) - np.min(y_coords))/4)
 
    angle = 0
    for st1, st2 in zip(measurements[:-1], measurements[1:]):
        angle += compute_angle(st2[0], st2[1], st1[0], st1[1], circle_x, circle_y)
 
    all_time = measurements[-1][2] - measurements[0][2]
    period = all_time / angle * 2 * math.pi
    return period, circle_x, circle_y, circle_r, circle_error

#Storing and processing of a set of 2D points
class Tracker:
    cycle_parameters = {}
    
    positions  = []
    timestamps = []

    def __init__(self):
        self.were_params_once_calculated = False
    
    def add_measurement (self, position, timestamp):
        self.positions.append  (position)
        self.timestamps.append (timestamp)

    #distance
    def _d (self, a, b):
        return math.sqrt ((a [0] - b [0])**2 + (a [1] - b [1])**2)

    def calc_cycle_parameters (self):
        #average point
        avg_x = 0
        avg_y = 0

        if (len (self.positions) != 0):
            avg_x = sum (self.positions [:] [0]) / len (self.positions)
            avg_y = sum (self.positions [:] [1]) / len (self.positions)

        self.cycle_parameters.update ({"average point" : (avg_x, avg_y)})

        left   = min (self.positions [:] [0])
        right  = max (self.positions [:] [0])
        bottom = min (self.positions [:] [1])
        top    = max (self.positions [:] [1])

        middle_x = (left + right) / 2
        middle_y = (bottom + top) / 2

        self.cycle_parameters.update ({"middle point" : (middle_x, middle_y)})
        self.cycle_parameters.update ({"height"       : bottom - top})
        self.cycle_parameters.update ({"width"        : right - left})

        #period

        period, circle_x, circle_y, circle_r, circle_error = \
            calculate_period (self.positions, self.timestamps)

        self.cycle_parameters.update ({"period" : period})
        self.cycle_parameters.update ({"circle center" : (circle_x, circle_y)})
        self.cycle_parameters.update ({"radius" : circle_r})

        #angular speed
        self.cycle_parameters.update ({"angular speed radians" : 2 * 3.14159265 / period})
        self.cycle_parameters.update ({"angular speed degrees" : 360 / period})

        #rotation direction

    def find_cycle_type (self):
        if (self.were_params_once_calculated == False):
            self.calc_cycle_parameters ()
            self.were_params_once_calculated = True

        height = self.cycle_parameters ["height"]
        width  = self.cycle_parameters ["width"]

        if (height >= 5 * width):
            cycle_type = "vertical"

        elif (width >= 5 * height):
            cycle_type = "horizontal"

        else:
            cycle_type = "circular"

        self.cycle_parameters.update ({"cycle type" : cycle_type})

    #def predict_position (self, time):
    #def predict_time (self, position):

    def get_norm_vector(self, point):
        return np.subtract (point, self.cycle_parameters['circle center'])
 
    def get_current_phase(self):
        '''
        Phase zero is at 3 o\'clock.
        Phase increments clockwise from 0 to 2Pi
        '''
        norm = self.get_norm_vector(self.positions[-1])
        orthodox_angle = math.atan2(norm[1], norm[0])
        our_angle = orthodox_angle if orthodox_angle > 0 else orthodox_angle + 2 * math.pi
        return our_angle
 
    def predict_time(self, position):
        def dotproduct(v1, v2):
            return sum((a*b) for a, b in zip(v1, v2))
 
        def length(v):
            return math.sqrt(dotproduct(v, v))
 
        def angle_b2v(v1, v2):
            cos = dotproduct(v1, v2) / (length(v1) * length(v2))
            angle = 0 if cos > 1 else math.pi if cos < -1 else math.acos(cos)
            return angle if angle > 0 else angle + 2 * math.pi
 
        current_vec = self.get_norm_vector(self.positions [-1])
        point_norm_vec = self.get_norm_vector(position)
        angle_diff = angle_b2v(current_vec, point_norm_vec)
        angle_diff = angle_diff if angle_diff > 0 else angle_diff + 2 * math.pi
 
        angular_speed = self.cycle_parameters ["angular speed radians"]

        predicted_time = angle_diff / angular_speed

        #print (current_vec, angle_diff, angular_speed)

        return predicted_time   

#Storing and processing 2d points on a virtual keyboard
class Target_handler:
    positions  = []
    timestamps = []

    def __init__(self, window_sz):
        self.window_sz = window_sz_
    
    def add_measurement (self, position, timestamp):
        self.positions.append  (position)
        self.timestamps.append (timestamp)

    def pressed (self):
        if (len (positions) < self.window_sz):
            return False, (0, 0)

        grads = []

        for i in range (0, self.window_sz):
            grads.append (self.positions [-self.window_sz + i - 1] - self.positions [-self.window_sz + i])

        if (grads [0:int (self.window_sz * 2 / 3)].sum () > 0 and
            sum (grads [int (self.window_sz * 2 / 3):]) < 0):
            max_x = 0
            max_y = 0

            for pos in self.positions:
                if (pos [1] > max_y):
                    (max_x, max_y) = pos 

            return True, (max_x, max_y)

        return False, (0, 0)

    def draw_y_by_t (self):
        width  = 700
        height = 700

        canvas = np.array ((300, 300), np.uint8)

        i = 0
        for pos in positions [-self.window_sz:]:
            x = int (width * (i + 1) / (self.window_sz + 1))
            y = pos [1]

            cv2.Circle (canvas, pos, (120, 150, 200), 6)

        return canvas

