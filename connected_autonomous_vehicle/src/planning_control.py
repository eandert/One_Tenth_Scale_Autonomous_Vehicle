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
        self.localizationPositionX_actual = 0
        self.localizationPositionY_actual = 0
        self.lastTargetWithinTL = 0
        self.distance_pid_control_en = False
        self.distance_pid_control_overide = False
        self.followDistance = self.Lfc
        self.followDistanceGain = .5
        self.targetFollowDistance = 1
        self.tfl_mode = 0
        self.av_intersection_permission = 0
        self.tind = 0

        self.id = None
        self.simVehicle = True
        self.key = None
        self.coordinateGroupVelocities = [0,0,0,0,0]

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
        self.lidarDetectionsRaw = []

        # Start sensors with standard error model
        #print(math.hypot(math.sqrt(0.03733623589373042) + math.sqrt(0.0428) * 0.36282835297512617, math.sqrt(0.0841) + math.sqrt(0.0241) * 0.36282835297512617))
        # self.localization = sensor.Localization(math.sqrt(0.0782), math.sqrt(0.0428), math.sqrt(0.0841), math.sqrt(0.0241))
        # self.lidarSensor = sensor.Sensor("M1M1", 0.0, 360, 15.0,
        #                                        math.sqrt(0.0097), math.sqrt(0.0361), math.sqrt(0.0165), math.sqrt(0.0607))
        # self.cameraSensor = sensor.Sensor("IMX160", 0.0, 160, 10.0,
        #                                        math.sqrt(0.0117), math.sqrt(0.023), math.sqrt(0.0517), math.sqrt(0.0126))

        print(math.hypot(0.03733623589373042 + 0.02798284589109133 * 0.36282835297512617, 0.019538095885057276 + 0.02941907006456509 * 0.36282835297512617))
        self.localization = sensor.Localization(0.02798284589109133, 0.03733623589373042, 0.02941907006456509, 0.019538095885057276)
        self.lidarSensor = sensor.Sensor("M1M1", 0.0, 360, 15.0,
                                               0.0097, 0.0361, 0.0517, 0.0126)
        self.cameraSensor = sensor.Sensor("IMX160", 0.0, 160, 10.0,
                                               0.0117, 0.023, 0.0165, 0.0607)

        # self.localization = sensor.Localization(0.0782, 0.0428, 0.0841, 0.0241)
        # self.lidarSensor = sensor.Sensor("M1M1", 0.0, 360, 15.0,
        #                                        0.0097, 0.0361, 0.0165, 0.0607)
        # self.cameraSensor = sensor.Sensor("IMX160", 0.0, 160, 10.0,
        #                                        0.0117, 0.023, 0.0517, 0.0126)
        
    def initialVehicleAtPosition(self, x_init, y_init, theta_init, xCoordinates, yCoordinates, vCoordinates, id_in, simVehicle):
        self.targetVelocityGeneral = 0
        self.id = id_in
        self.simVehicle = simVehicle
        self.seeringAngle = 0
        # This is the known localization position
        if simVehicle:
            reverse_theta = theta_init-math.radians(180)
            self.rearAxlePositionX = x_init + (self.axleFromCenter * math.cos(reverse_theta))
            self.rearAxlePositionY = y_init + (self.axleFromCenter * math.sin(reverse_theta))
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
            self.localizationPositionX_actual = x_init
            self.localizationPositionY_actual = y_init
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
            reverse_theta = theta_init-math.radians(180)
            self.rearAxlePositionX = x_init + (self.axleFromCenter * math.cos(reverse_theta))
            self.rearAxlePositionY = y_init + (self.axleFromCenter * math.sin(reverse_theta))
            self.velocity = 0
        self.lastPointIndex = None
        self.xCoordinates = xCoordinates
        self.yCoordinates = yCoordinates
        self.vCoordinates = vCoordinates
        
        # Initialize the controllers\
        if self.simVehicle:
            self.v_pid = PID(3.0, 0.00, 0.0, setpoint=self.targetVelocity)
            self.d_pid = PID(2, 0.00, 0.0, setpoint=self.Lfc)
        else:
            self.v_pid = PID(2, 0.00, 0.0, setpoint=self.targetVelocity)
            self.d_pid = PID(2, 0.00, 0.0, setpoint=self.Lfc)

    def updatePosition(self, timestep):
        self.positionX_sim += self.velocity * math.cos(self.theta) * timestep
        self.positionY_sim += self.velocity * math.sin(self.theta) * timestep
        self.theta += ( self.velocity / self.wheelbaseLength ) * math.tan(self.steeringAcceleration) * timestep
        self.velocity += self.motorAcceleration * timestep

    def update_localization(self, use_localization, localization = None):
        if not use_localization:
            # Update the localization, we could inject errors here if we want
            reverse_theta = self.theta-math.radians(180)
            self.rearAxlePositionX = self.positionX_sim + (self.axleFromCenter * math.cos(reverse_theta))
            self.rearAxlePositionY = self.positionY_sim + (self.axleFromCenter * math.sin(reverse_theta))
            self.localizationPositionX = self.positionX_sim
            self.localizationPositionY = self.positionY_sim
        else:
            # Update the localization from real data
            # Calculate velocity before we update, the localization positions are from last frame
            # self.axleFromCenter is to adjust for lidar position vs rear axle
            # TODO: Check this!
            self.velocity = self.calc_velocity(localization[0] + self.positionX_offset, localization[1] + self.positionY_offset, self.localizationPositionX, self.localizationPositionY, localization[2])
            # Update the localization position correctly
            self.localizationPositionX = localization[0] + self.positionX_offset
            self.localizationPositionY = localization[1] + self.positionY_offset
            self.theta = localization[2] + self.theta_offset
            reverse_theta = self.theta - math.radians(180)
            self.rearAxlePositionX = self.localizationPositionX + (self.axleFromCenter * math.cos(reverse_theta))
            self.rearAxlePositionY = self.localizationPositionY + (self.axleFromCenter * math.sin(reverse_theta))

    def calc_velocity(self, x1, y1, x2, y2, theta):
        velocity = math.hypot(x2 - x1, y2 - y1) * (1/8)
        expected_theta = math.atan2(y2 - y1, x2 - x1)
        if not (theta < (expected_theta + math.radians(45)) and theta > (expected_theta - math.radians(45))):
            # We are traveling backwards according to the LIDAR so adjust the velocity accordingly
            velocity = -velocity
        return velocity

    def pure_pursuit_control(self):

        ind = self.search_target_index()

        tx = self.xCoordinates[ind]
        ty = self.yCoordinates[ind]
        self.tind = ind

        alpha = math.atan2(ty - self.rearAxlePositionY, tx - self.rearAxlePositionX) - self.theta

        Lf = self.k * self.velocity + self.Lfc + self.axleFromCenter

        delta = math.atan2(2.0 * self.wheelbaseLength * math.sin(alpha) / Lf, 1.0)

        if delta > self.steeringAngleMax:
            delta = self.steeringAngleMax
        elif delta < -self.steeringAngleMax:
            delta = -self.steeringAngleMax

        # Account for the fact that in reverse we should be turning the other way
        #if self.velocity < 0:
        #    delta = -delta
        self.steeringAcceleration = delta

        self.targetIndexX = tx
        self.targetIndexY = ty
        self.distance_pid_control_overide = False
        if self.vCoordinates[ind] == 0:
            self.targetVelocity = self.targetVelocityGeneral
            self.lastTargetWithinTL = 0
        else:
            # Traffic light calculation
            if self.tfl_mode == 0:
                if self.coordinateGroupVelocities[self.vCoordinates[ind]] == 1:
                    if self.lastTargetWithinTL == 1:
                        if self.targetVelocity == 0:
                            # We are already stopping so keep stopping
                            self.targetVelocity = 0
                            self.distance_pid_control_overide = True
                        else:
                            # We have already entered the light so keep going
                            self.targetVelocity = self.targetVelocityGeneral
                    else:
                        # This is the first point we have seen of this TFL, should have enought time to stop
                        self.targetVelocity = 0
                        self.distance_pid_control_overide = True
                    self.lastTargetWithinTL = 1
                elif self.coordinateGroupVelocities[self.vCoordinates[ind]] == 2:
                    self.targetVelocity = self.targetVelocityGeneral
                    self.lastTargetWithinTL = 1
                else:
                    self.targetVelocity = 0
                    self.lastTargetWithinTL = 1
                    self.distance_pid_control_overide = True
            # Autonomous intersection mode calculation
            else:
                if self.av_intersection_permission:
                    self.targetVelocity = self.targetVelocityGeneral
                else:
                    self.targetVelocity = 0
                    self.distance_pid_control_overide = True

        if alpha != 0:
            turningRadius = self.wheelbaseLength / math.tan(alpha)
            if turningRadius > 10:
                turningRadius = 10
            if turningRadius < -10:
                turningRadius = -10
        else:
            turningRadius = 10

        # ( "Target ", self.targetVelocity, self.targetIndexX, self.targetIndexY, self.coordinateGroupVelocities[self.vCoordinates[ind]])

    def update_pid(self):
        if self.distance_pid_control_en and not self.distance_pid_control_overide:
            #print("TD", self.targetFollowDistance, "FD", self.followDistance)
            self.d_pid.setpoint = self.targetFollowDistance + self.followDistanceGain * self.velocity
            self.motorAcceleration = self.d_pid(self.followDistance)
        else:
            # Default to velocity PID cotnrol
            #print("TV, TG ", self.targetVelocity, self.targetVelocityGeneral)
            self.v_pid.setpoint = self.targetVelocity
            self.motorAcceleration = self.v_pid(self.velocity)
            #print( "motorAcceleration", self.motorAcceleration)
        # Check for pause and we have no reverse
        if self.targetVelocityGeneral == 0.0 or (self.targetVelocity <= 0.0 and not self.simVehicle):
            self.motorAcceleration = 0.0
        #if self.simVehicle == False:
            #commands[self.id] = [-self.steeringAcceleration, self.motorAcceleration]

    def calc_distance(self, point_x, point_y):
        dx = self.rearAxlePositionX - point_x
        dy = self.rearAxlePositionY - point_y
        return math.hypot(dx, dy)

    def recieve_coordinate_group_commands(self, commands):
        self.coordinateGroupVelocities = commands

    def get_location(self):
        return [self.localizationPositionX, self.localizationPositionY, self.theta, self.targetVelocity, self.width, self.length, self.id]

    def get_route(self):
        messageString = ""
        for xzip, yzip in zip(self.xCoordinates, self.yCoordinates):
            if len(messageString) == 0:
                messageString = str(xzip) + ',' + str(yzip)
            else:
                messageString = messageString + ':' + str(xzip) + ',' + str(yzip)
        return messageString

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

    def check_in_range_and_fov(self, target_angle, distance, center_angle, horizontal_fov, max_range):
        angle_diff = ((center_angle - target_angle + math.pi + (2*math.pi)) % (2*math.pi)) - math.pi
        if abs(angle_diff) <= (horizontal_fov/2.0) and (distance <= max_range):
            return True
        return False

    def check_positions_of_other_vehicles_adjust_velocity(self, positions):
        self.followDistance = 99
        self.distance_pid_control_en = False
        point = Point(self.targetIndexX, self.targetIndexY)
        for idx, each in enumerate(positions):
            # Create a bounding box for each vehicle that is length + 2*buffer long and width + 2*buffer wide
            x1 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2]+math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2]-math.radians(180))))
            y1 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2]+math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2]-math.radians(180))))
            x2 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2]-math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2]-math.radians(180))))
            y2 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2]-math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2]-math.radians(180))))
            x3 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2]-math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2])))
            y3 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2]-math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2])))
            x4 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2]+math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2])))
            y4 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2]+math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2])))

            polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
            #print(polygon.contains(point))
            #if self.check_if_point_in_rectangle(x1, y1, x2, y2, self.targetIndexX, self.targetIndexY):
            if polygon.contains(point):
                #print ( "True" )
                # Within the safety buffer, adjust speed to be that of the front vehicle
                #if self.targetVelocity > each[3]:
                #    self.targetVelocity = each[3]
                # For distance control
                targetFollowDistance = self.Lfc + self.followDistanceGain * self.velocity
                #followDistance = math.hypot(each[0]+((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2]))-self.rearAxlePositionX, each[1]+((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2]))-self.rearAxlePositionY)
                followDistance = math.hypot(self.localizationPositionX - (x3+x4)/2.0, self.localizationPositionY - (y3+y4)/2.0)
                if self.followDistance > followDistance:
                    self.followDistance = followDistance
                    self.targetFollowDistance = targetFollowDistance
                    self.distance_pid_control_en = True

    def check_steering_angle_possible(self, x, y):
        # This equation is a quick check for if it is possible to get to the current point based on geometry
        # Essentually 2 circles that we cant go to
        dx = self.rearAxlePositionX + self.maxTurningRadius*math.cos(shared_math.angleDifference(self.theta + math.radians(90),x))
        dy = self.rearAxlePositionY + self.maxTurningRadius*math.cos(shared_math.angleDifference(self.theta + math.radians(90),y))
        d = math.hypot(dx, dy)

        dx2 = self.rearAxlePositionX + self.maxTurningRadius*math.cos(shared_math.angleDifference(self.theta - math.radians(90), x))
        dy2 = self.rearAxlePositionY + self.maxTurningRadius*math.cos(shared_math.angleDifference(self.theta - math.radians(90), y))
        d2 = math.hypot(dx2, dy2)

        a = shared_math.angleDifference(math.atan2(self.rearAxlePositionY - y, self.rearAxlePositionX - x), self.theta)
        
        # Check if the target point is within either no go circle
        if d < self.maxTurningRadius or d2 < self.maxTurningRadius and a > math.radians(-90) and a < math.radians(90):
            return False
        else:
            return True

    def search_target_index(self):
        Lf = self.k * self.velocity + self.Lfc

        if self.lastPointIndex is None:
            # Search for the initial point, not reverse
            dx = [self.rearAxlePositionX - icx for icx in self.xCoordinates]
            dy = [self.rearAxlePositionY - icy for icy in self.yCoordinates]
            d = np.hypot(dx, dy)
            for index in range(len(dx)):
                if not self.check_steering_angle_possible(dx[index],dy[index]):
                    d[index] = 1000.0
            ind = np.argmin(d)
            self.lastPointIndex = ind
        else:
            ind = self.lastPointIndex
            distance_this_index = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])
            while True:
                checkind = ind + 1
                if checkind >= len(self.xCoordinates):
                    checkind = 0
                distance_next_index = self.calc_distance(self.xCoordinates[checkind], self.yCoordinates[checkind])
                if distance_next_index > distance_this_index:
                    break
                distance_this_index = distance_next_index
                ind = checkind
            self.lastPointIndex = ind

        L = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])

        # search look ahead target point index
        while Lf > L:
            L = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])
            ind += 1
            if ind >= len(self.xCoordinates):
                ind = 0

        return ind

    def return_command_package(self):
        return -self.steeringAcceleration, self.motorAcceleration

