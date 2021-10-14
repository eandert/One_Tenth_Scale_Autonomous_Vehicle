import math
import numpy as np

import local_fusion
import shared_math
import sensor


''' This class contains the parameters of a RC car platform like wheelbase, etc. as
well as a rudimentary physics estimation based on the ackerman steering model. We can feed the class
a route which consists of a series of x,y coordinates and group speed target integers. The planner
class can then be classed with the current position from the localizer and it will use
pure pursuit to select a target point and decide a PPM value for the steering servo. There is also
a built in PID controller that will select the PID control for the motor based on the target velocity
of the target point velocity group. This velocity group accounts for traffic lights. '''
class Planner:
    def __init__(self):
        # Static Vehicle Params
        self.width = .3
        self.length = .3
        self.wheelbaseLength = .2
        self.wheelbaseWidth = .2
        self.theta = 0

        self.localizationPositionX = 0
        self.localizationPositionY = 0

        self.id = None
        self.simCIS = True
        self.key = None

        self.cameraDetections = []
        self.fusionDetections = []
        self.fusionDetectionsCovariance = []
        self.groundTruth = []

        self.cameraSensor = sensor.Sensor("IMX160", 0.0, 160, 10.0,
                                               0, .025, .05, .15)
        self.lidarSensor = None
        
    def initialVehicleAtPosition(self, x_offset, y_offset, theta_offset, id_in, simCIS):
        # This holds the actual position of the vehicle
        self.positionX_offset = x_offset
        self.positionY_offset = y_offset
        self.theta_offset = math.radians(theta_offset)
        # This is the known localization position
        if simCIS:
            self.localizationPositionX = self.positionX_offset
            self.localizationPositionY = self.positionY_offset
            self.theta = self.theta_offset
            self.positionX_sim = self.localizationPositionX
            self.positionY_sim = self.localizationPositionY
        else:
            # Since this is a real world test we will start the vehicle somewhere random until it connects
            self.localizationPositionX = 5 + self.positionX_offset
            self.localizationPositionY = 5 + self.positionY_offset
            self.theta = self.theta_offset

    def updatePosition(self, timestep):
        pass

    def update_localization(self, localization = None):
        if self.simCIS:
            # Update the localization, we could inject errors here if we want
            pass
        else:
            # Update the localization from real data
            # Calculate velocity before we update, the localization positions are from last frame
            #  - .175 is to adjust for lidar position vs rear axle
            self.velocity = 0.0 #self.calc_velocity(localization[0], localization[1], self.localizationPositionX, self.localizationPositionY, localization[2])
            self.localizationPositionX = (((localization[0] - .175) * math.cos(self.theta_offset)) - (localization[1] * math.sin(self.theta_offset))) + self.positionX_offset
            self.localizationPositionY = ((localization[1] * math.cos(self.theta_offset)) + (localization[0] * math.sin(self.theta_offset))) + self.positionY_offset
            self.theta = localization[2] + self.theta_offset

    def calc_distance(self, point_x, point_y):
        dx = self.localizationPositionX - point_x
        dy = self.localizationPositionY - point_y
        return math.hypot(dx, dy)

    def get_location(self):
        return [self.localizationPositionX, self.localizationPositionY, self.theta, self.targetVelocity, .2, self.width, self.length]

    def check_if_point_in_rectangle(self, x1, y1, x2, y2, x, y):
        if x1 > x2:
            xtemp = x2
            x2 = x1
            x1 = xtemp
        if y1 > y2:
            ytemp = y2
            y2 = y1
            y1 = ytemp
        if (x > x1) and (x < x2):
            if (y > y1) and (y < y2):
                return True
        return False

