import math
import sys
import numpy as np
from simple_pid import PID
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry.linestring import LineString


# Global imports
sys.path.append("../../../")
from shared_library import sensor, shared_math


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
        self.length = .57
        self.wheelbaseLength = .35
        self.wheelbaseWidth = .245
        self.axleFromCenter = self.wheelbaseLength/2.0
        self.steeringAngleMax = math.radians(30.0)
        self.velocityMax = 1.0
        
        self.maxTurningRadius = self.wheelbaseLength / math.tan(self.steeringAngleMax)
        
        self.k = 1.0  # look forward gain
        self.Lfc = 0.5  # look-ahead distance
        
        # Updatable Params
        self.velocity = 0
        self.theta = 0
        self.seeringAngle = 0
        self.positionX_sim = 0
        self.positionY_sim = 0
        self.rearAxlePositionX = 0
        self.rearAxlePositionY = 0
        self.lastPointIndex = 0
        self.targetIndexX = 0
        self.targetIndexY = 0
        self.lookAheadIndex = 0
        self.targetVelocity = 0
        self.steeringAcceleration = 0
        self.motorAcceleration = 0
        self.pursuit_index = 0
        self.localizationPositionX = 0
        self.localizationPositionY = 0
        self.lastTargetWithinTL = 0
        self.distance_pid_control_en = False
        self.distance_pid_control_overide = False
        self.followDistance = self.Lfc
        self.followDistanceGain = .5
        self.targetFollowDistance = 1

        self.id = None
        self.simVehicle = True
        self.key = None

        # Buffer to be added around vehicles to do follow distance math
        self.arbitrary_buffer = 0.1

        self.cameraDetections = []
        self.lidarDetections = []
        self.fusionDetections = []
        self.rawLidarDetections = []
        self.groundTruth = []
        self.localizationCovariance = np.array([[1.0, 0.0],
                                                [0.0, 1.0]]).tolist()

        # Raw LIDAR for gui debug
        self.lidarPoints = []
        self.localizationError = None

        # Start sensors with standard error model
        self.localization = sensor.Localization(0.001, 0.0, 0.01, 0.0)
        self.lidarSensor = None
        self.cameraSensor = sensor.Sensor("IMX160", 0.0, 160, 10.0,
                                               0, .025, .05, .15)
        
    def initialSensorAtPosition(self, x_init, y_init, theta_init, xCoordinates, yCoordinates, vCoordinates, id_in, simVehicle):
        self.targetVelocityGeneral = 0
        self.id = id_in
        self.simVehicle = simVehicle
        self.seeringAngle = 0
        # This is the known localization position
        if simVehicle:
            self.rearAxlePositionX = x_init
            self.rearAxlePositionY = y_init
            self.localizationPositionX = x_init
            self.localizationPositionY = y_init
            self.positionX_sim = x_init
            self.positionY_sim = y_init
            self.velocity = 0
            self.theta = theta_init
            # For simulation we will not use the offset but still need to store them
            self.positionX_offset = x_init
            self.positionY_offset = y_init
            self.theta_offset = theta_init
        else:
            # A real world test
            # We need to calcualte the constant offset for the LIDAR to world coordinates here
            # Assume we are starting at 0,0
            self.localizationPositionX = x_init
            self.localizationPositionY = y_init
            self.theta = theta_init
            self.positionX_offset = x_init
            self.positionY_offset = y_init
            self.theta_offset = theta_init
            # Now set our rear axle position
            self.rearAxlePositionX = x_init
            self.rearAxlePositionY = y_init
            self.velocity = 0
        self.lastPointIndex = None
        self.xCoordinates = xCoordinates
        self.yCoordinates = yCoordinates
        self.vCoordinates = vCoordinates

    def update_localization(self, use_localization, localization = None):
        # Update the localization, we could inject errors here if we want
        self.rearAxlePositionX = self.positionX_sim
        self.rearAxlePositionY = self.positionY_sim
        self.localizationPositionX = self.positionX_sim
        self.localizationPositionY = self.positionY_sim