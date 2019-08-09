import math

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

        if (len (positions) != 0):
            avg_x = positions [:] [0] / len (positions)
            avg_y = positions [:] [1] / len (positions)

        self.cycle_parameters.update ({"average point" : (avg_x, avg_y)})

        left   = min (positions [:] [0])
        right  = max (positions [:] [0])
        bottom = min (positions [:] [1])
        top    = max (positions [:] [1])

        middle_x = (left + right) / 2
        middle_y = (bottom + top) / 2

        self.cycle_parameters.upate ({"middle point" : (middle_x, middle_y)})
        self.cycle_parameters.upate ({"height"       : bottom - top})
        self.cycle_parameters.upate ({"width"        : right - left})

        #period
        #PANDAS/-like TIME SERIES ANALYSIS WILL BE HERE)0))

        for i in range (len (positions)):
            #cycle thru the remaining points in order to find closest

            for j in range (i + 1, len (positions)):

                #check if was big enough distance
                #check if new min
                #append timestamps difference in list

        #find period out of the list with candidates
        #average, median?

        #angular velocity
        self.cycle_parameters.upate ({"angular velocity radians" : 2 * 3.14159265 / period})
        self.cycle_parameters.upate ({"angular velocity degrees" : 360 / period})

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

        self.cycle_parameters.update ("cycle type" : cycle_type)

    def predict_position (self, time):
        

    def predict_time (self, position):
        