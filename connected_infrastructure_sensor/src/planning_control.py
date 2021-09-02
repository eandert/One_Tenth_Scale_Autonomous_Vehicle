import math
import numpy as np
from simple_pid import PID
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry.linestring import LineString
from connected_autonomous_vehicle.src import local_fusion


''' Helper function to calculate the difference between 2 angles in radians'''
def angleDifference( angle1, angle2 ):
    diff = ( angle2 - angle1 + math.pi ) % (2*math.pi) - math.pi
    if diff < -math.pi:
        return diff + (2*math.pi)
    else:
        return diff


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

        self.cameraSensor = local_fusion.Sensor("IMX160", 0.0, 160, 10.0,
                                               .025, .15, .10, .10)
        
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

    def check_in_range_and_fov(self, target_angle, distance, center_angle, horizontal_fov, max_range):
        angle_diff = ((center_angle - target_angle + math.pi + (2*math.pi)) % (2*math.pi)) - math.pi
        if abs(angle_diff) <= (horizontal_fov/2.0) and (distance <= max_range):
            return True
        return False

    def fake_camera(self, positions, objects, cam_range, cam_center_angle, cam_fov):

        # print ( "FAKING LIDAR" )
        camera_array = []
        camera_error_array = []

        # Get the points the Slamware M1M1 should generate
        lidar_freq = 7000 / 8
        angle_change = (2 * math.pi) / lidar_freq

        # Create all the vehicle polygons and combine them into one big list
        polygons = []
        for idx, vehicle in enumerate(positions):
            # Create a bounding box for vehicle vehicle that is length + 2*buffer long and width + 2*buffer wide
            x1 = vehicle[0] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.cos(vehicle[2] + math.radians(90)) + (
                        (vehicle[4]) * math.cos(vehicle[2] - math.radians(180))))
            y1 = vehicle[1] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.sin(vehicle[2] + math.radians(90)) + (
                        (vehicle[4]) * math.sin(vehicle[2] - math.radians(180))))
            x2 = vehicle[0] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.cos(vehicle[2] - math.radians(90)) + (
                        (vehicle[4]) * math.cos(vehicle[2] - math.radians(180))))
            y2 = vehicle[1] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.sin(vehicle[2] - math.radians(90)) + (
                        (vehicle[4]) * math.sin(vehicle[2] - math.radians(180))))
            x3 = vehicle[0] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.cos(vehicle[2] - math.radians(90)) + (
                        (self.wheelbaseLength + vehicle[4]) * math.cos(vehicle[2])))
            y3 = vehicle[1] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.sin(vehicle[2] - math.radians(90)) + (
                        (self.wheelbaseLength + vehicle[4]) * math.sin(vehicle[2])))
            x4 = vehicle[0] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.cos(vehicle[2] + math.radians(90)) + (
                        (self.wheelbaseLength + vehicle[4]) * math.cos(vehicle[2])))
            y4 = vehicle[1] + ((self.wheelbaseWidth / 2 + vehicle[4]) * math.sin(vehicle[2] + math.radians(90)) + (
                        (self.wheelbaseLength + vehicle[4]) * math.sin(vehicle[2])))
            polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
            polygons.append(polygon)

        # Generate fake lidar data from the points
        for angle_idx in range(int(lidar_freq)):
            intersections = []
            intersections_origin_point = []
            intersections_count = 0
            intersect_dist = 9999999999
            final_point = None
            final_polygon = None

            # Go through all the polygons that the line intersects with and add them
            for poly in polygons:
                line = [(self.localizationPositionX, self.localizationPositionY), (
                self.localizationPositionX + (cam_range * math.cos(angle_idx * angle_change)),
                self.localizationPositionY + (cam_range * math.sin(angle_idx * angle_change)))]
                shapely_line = LineString(line)
                intersections += list(poly.intersection(shapely_line).coords)
                for idx in range(len(intersections) - intersections_count):
                    intersections_origin_point.append(poly)
                intersections_count = len(intersections)

            # Don't forget the other objects as well (already should be a list of polygons)
            for poly in objects:
                line = [(self.localizationPositionX, self.localizationPositionY), (
                self.localizationPositionX + (cam_range * math.cos(angle_idx * angle_change)),
                self.localizationPositionY + (cam_range * math.sin(angle_idx * angle_change)))]
                shapely_line = LineString(line)
                print ( poly )
                intersections += list(poly.intersection(shapely_line).coords)
                for idx in range(len(intersections) - intersections_count):
                    intersections_origin_point.append(poly)
                intersections_count = len(intersections)

            # Get the closest intersection with a polygon as that will be where our lidar beam stops
            for point, polygon in zip(intersections, intersections_origin_point):
                dist = math.hypot(point[0] - self.localizationPositionX, point[1] - self.localizationPositionY)
                if dist < intersect_dist:
                    final_point = point
                    intersect_dist = dist
                    final_polygon = polygon

            # Make sure this worked and is not None
            if final_point != None:
                # See if we can add a camera point
                if self.check_in_range_and_fov(angle_idx * angle_change, intersect_dist, self.theta + cam_center_angle,
                                               math.radians(cam_fov), cam_range):
                    # Object checks out and is in range and not blocked
                    # TODO: Do a little better approxamation of percent seen and account for this
                    point = list(final_polygon.centroid.coords)[0]
                    if point not in camera_array:
                        # Create the error component of the camera detection
                        delta_x = point[0] - self.localizationPositionX
                        delta_y = point[1] - self.localizationPositionY
                        angle = math.atan2(delta_y, delta_x)
                        distance = math.hypot(delta_x, delta_y)
                        #print ( "a:", math.degrees(angle), " d:", distance )
                        success, expected_error_gaussian, actual_sim_error = self.cameraSensor.calculateErrorGaussian(angle, distance, True)
                        #print ( success, point, expected_error_gaussian, actual_sim_error )
                        #print ( self.localizationPositionX, self.localizationPositionY )
                        if success:
                            camera_error_array.append((point[0] + actual_sim_error[0], point[1] + actual_sim_error[1]))
                            camera_array.append((point[0], point[1]))

        return camera_array, camera_error_array

