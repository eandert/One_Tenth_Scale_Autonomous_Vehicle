import math
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry.linestring import LineString

from shared_library import shared_math
from shared_library.local_fusion import MAX_ID


class BivariateGaussian:
    # This class is for creating and storing error terms of a sensor using 
    # mu and covariance so they can be fed into kalman filters and printed
    def __init__(self, a, b, phi, mu = None, cov = None):
        if cov is None:
            # Create the bivariate gaussian matrix
            self.mu = np.array([0.0, 0.0])
            self.covariance = np.array([[a, 0], [0, b]])

            # RΣR^T to rotate the ellipse where Σ is the original covariance matrix
            rotate = np.array([[math.cos(phi), math.sin(phi)], [-math.sin(phi), math.cos(phi)]])
            self.covariance = np.matmul(rotate, self.covariance)
            self.covariance = np.matmul(self.covariance, rotate.transpose())
        else:
            # Create the bivariate gaussian matrix
            self.mu = mu
            self.covariance = cov

    def calcSelfRadiusAtAnlge(self, angle, num_std_deviations):
        a, b, phi = self.extractErrorElipseParamsFromBivariateGaussian(num_std_deviations)
        return shared_math.calculateRadiusAtAngle(a, b, phi, angle)

    def eigsorted(self):
        '''
        Eigenvalues and eigenvectors of the covariance matrix.
        '''
        vals, vecs = np.linalg.eigh(self.covariance)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:, order]


    def extractErrorElipseParamsFromBivariateGaussian(self, num_std_deviations):
        """
        Source: http://stackoverflow.com/a/12321306/1391441
        """

        if np.all((self.covariance == 0.0)):
            return 0, 0, 0.0

        vals, vecs = self.eigsorted()
        phi = np.arctan2(*vecs[:, 0][::-1])

        a = num_std_deviations * vals[0]
        b = num_std_deviations * vals[1]

        return a, b, phi

    def calcXYComponents(self):
        x_comp = self.calcSelfRadiusAtAnlge(math.radians(0), 1.0)
        y_comp = self.calcSelfRadiusAtAnlge(math.radians(90), 1.0)
        return x_comp, y_comp

    def unionBivariateGaussians(self, gaussianB):
        # Now we have both distributions, it is time to do the combination math
        # Source: http://www.geostatisticslessons.com/lessons/errorellipses
        self.covariance = np.add(self.covariance.transpose(), gaussianB.covariance.transpose()).transpose()


def unionBivariateGaussians(gaussianA, gaussianB):
    # Now we have both distributions, it is time to do the combination math
    # Source: http://www.geostatisticslessons.com/lessons/errorellipses
    covariance_new = np.add(gaussianA.transpose(), gaussianB.transpose()).transpose()

    return covariance_new


def intersectionBivariateGaussiansCovariance(covariance_gaussian_a, covariance_gaussian_b):
    # Now we have both distributions, it is time to do the combination math
    # Source: http://www.geostatisticslessons.com/lessons/errorellipses
    covariance_new = np.subtract(covariance_gaussian_a.transpose(), covariance_gaussian_b.transpose()).transpose()

    return covariance_new


class Localization:
    # This object is for storing and calculating expected localization accuracy for 
    # object. In full simulation we inject error and generate expected error 
    # gaussians. In real world we only generate expected error gaussians.
    def __init__(self, longitudinal_error_x, longitudinal_error_b, lateral_error_x, lateral_error_b):
        self.longitudinal_error_x = longitudinal_error_x
        self.longitudinal_error_b = longitudinal_error_b
        self.lateral_error_x = lateral_error_x
        self.lateral_error_b = lateral_error_b
        self.static_error_range_average = .36282835297512617
        self.static_error = (longitudinal_error_b + longitudinal_error_x * self.static_error_range_average + \
                                    lateral_error_b + lateral_error_x * self.static_error_range_average) / 2.0

    def recalc_static_error(self):
        self.static_error = (self.longitudinal_error_b + self.longitudinal_error_x * self.static_error_range_average + \
                                    self.lateral_error_b + self.lateral_error_x * self.static_error_range_average) / 2.0
    
    def getErrorParamsAtVelocity(self, velocity, theta):
        elipse_longitudinal_expected = self.longitudinal_error_x * velocity + self.longitudinal_error_b
        elipse_lateral_expected = self.lateral_error_x * velocity + self.lateral_error_b
        elipse_angle_expected = theta
        expected_error_gaussian = BivariateGaussian(elipse_longitudinal_expected**2,
                                    elipse_lateral_expected**2,
                                    elipse_angle_expected)
        loc_error_longitudinal_actual = np.random.normal(0, elipse_longitudinal_expected, 1)[0]
        loc_error_lateral_actual = np.random.normal(0, elipse_lateral_expected, 1)[0]
        actual_error_gaussian = BivariateGaussian(loc_error_longitudinal_actual**2,
                                    loc_error_lateral_actual**2,
                                    elipse_angle_expected)
        actual_sim_error = actual_error_gaussian.calcXYComponents()
        #print("loc: ", actual_sim_error)
        return expected_error_gaussian, actual_sim_error

    def getStaticErrorParams(self, velocity, theta):
        # We need the real error here so call the other function
        expected_error_gaussian, actual_sim_error = self.getErrorParamsAtVelocity(velocity, theta)
        # It's just a circle so both are the same
        expected_error_gaussian_2 = BivariateGaussian(self.static_error**2,
                                    self.static_error**2,
                                    0.0)
        return expected_error_gaussian_2, actual_sim_error


class Sensor:
    # This object is for storing and calculating expected sensor accuracy for 
    # object. In full simulation we inject error and generate expected error 
    # gaussians. In real world we only generate expected error gaussians.
    def __init__(self, sensor_name, center_angle, field_of_view, max_distance, radial_error_x, radial_error_b, distal_error_x, distal_error_b):
        self.sensor_name = sensor_name
        self.center_angle = center_angle
        self.field_of_view = field_of_view
        self.max_distance = max_distance
        self.radial_error_x = radial_error_x
        self.radial_error_b = radial_error_b
        self.distal_error_x = distal_error_x
        self.distal_error_b = distal_error_b
        self.static_error_range_average = 1.5
        self.static_error = (radial_error_b + radial_error_x * self.static_error_range_average + \
                                    distal_error_b + distal_error_x * self.static_error_range_average) / 2.0

    def checkInRangeAndFOV(self, object_angle, object_distance):
        anglediff = ((self.center_angle - object_angle + math.pi + (2*math.pi)) % (2*math.pi)) - math.pi
        if abs(anglediff) <= (self.field_of_view) and (object_distance <= self.max_distance):
            return True
        return False

    def recalc_static_error(self):
        self.static_error = (self.radial_error_b + self.radial_error_x * self.static_error_range_average + \
                                    self.distal_error_b + self.distal_error_x * self.static_error_range_average) / 2.0
        
    def getRadialErrorAtDistance(self, object_distance):
        # This version we are disregarding horizontal crosssection for now
        return self.radial_error_b + (object_distance * self.radial_error_x)
        
    def getDistanceErrorAtDistance(self, object_distance):
        return self.distal_error_b + (object_distance * self.distal_error_x)

    def calculateErrorGaussian(self, object_relative_angle_to_detector, target_line_angle, object_distance, simulation, parameterized_covariance):
        # TODO: check ATLAS code and makse sure this is the same
        if self.checkInRangeAndFOV(object_relative_angle_to_detector, object_distance):
            radial_error = self.getRadialErrorAtDistance(object_distance)
            distal_error = self.getDistanceErrorAtDistance(object_distance)
            # Calculate our expected elipse error bounds
            elipse_angle_expected = target_line_angle
            if parameterized_covariance:
                expected_error_gaussian = BivariateGaussian(distal_error**2,
                                        radial_error**2,
                                        elipse_angle_expected)
            else:
                expected_error_gaussian = BivariateGaussian(self.static_error**2,
                        self.static_error**2,
                        0.0)
            # Calculate the actual error if this is a simulation, otherwise just return
            if simulation:
                # Calculate our expected errors in x,y coordinates
                actualRadialError = np.random.normal(0, radial_error, 1)[0]
                actualDistanceError = np.random.normal(0, distal_error, 1)[0]
                actual_error_gaussian = BivariateGaussian(actualDistanceError**2,
                                      actualRadialError**2,
                                      elipse_angle_expected)
                actual_sim_error = actual_error_gaussian.calcXYComponents()
                return True, expected_error_gaussian, actual_sim_error
            else:
                actual_sim_error = [0.0, 0.0]
                return True, expected_error_gaussian, actual_sim_error
        else:
            # Fallthrough case just in case this is out of the FOV of the sensor
            return False, None, None


# In some cases our error is additive. This function adds 2 gaussians together in a rough
# approxamation of the error.
def addBivariateGaussians(gaussianA, gaussianB):
    # Now we have both distributions, it is time to do the combination math
    covariance_new = np.add(gaussianA.transpose(), gaussianB.transpose()).transpose()

    return covariance_new

# This function is parses the setting from the simulation and calls necessary functions to simulate
# the sensors and their respective errors
def simulate_sensors(planner, lidarRecognition, time, sim_values, vehicle_object_positions, fusion_error=False, fusion_error_rate=1.0):
    lidar_returned = [[], [], None]
    cam_returned = [[], None]
    if lidarRecognition != None:
        lidar_dist = planner.lidarSensor.max_distance
        lidar_center = planner.lidarSensor.center_angle
        lidar_fov = planner.lidarSensor.field_of_view
    else:
        lidar_dist = planner.cameraSensor.max_distance
        lidar_center = planner.cameraSensor.center_angle
        lidar_fov = planner.cameraSensor.field_of_view

    if sim_values["parameterized_covariance"]:
        localization_error_gaussian, localization_error = planner.localization.getErrorParamsAtVelocity(abs(planner.velocity), planner.theta)
    else:
        localization_error_gaussian, localization_error = planner.localization.getStaticErrorParams(abs(planner.velocity), planner.theta)

    point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = fake_lidar_and_camera(planner,
        vehicle_object_positions, [], lidar_dist, lidar_center, lidar_fov, planner.cameraSensor.max_distance, planner.cameraSensor.center_angle, planner.cameraSensor.field_of_view, sim_values['parameterized_covariance'])

    if sim_values["simulate_error"]:
        # Error injection
        cam_returned[0] = camera_error_array
        cam_returned[1] = time
        planner.localizationError = localization_error_gaussian
        lidar_returned[0] = [planner.localizationPositionX + localization_error[0], planner.localizationPositionY + localization_error[1],
                    planner.theta, planner.velocity, localization_error_gaussian.covariance.tolist()]
        if lidarRecognition != None:
            if sim_values["real_lidar"]:
                # TODO: check this seems wrong
                lidar_returned[1], lidar_returned[2] = lidarRecognition.processLidarFrame(point_cloud_error, time, 
                    lidar_returned[0][0], lidar_returned[0][1], lidar_returned[0][2], planner.lidarSensor)
                planner.rawLidarDetections = point_cloud_error
            else:
                lidar_returned[1] = lidar_detected_error
                lidar_returned[2] = time
                planner.rawLidarDetections = point_cloud_error
    else:
        # No error injection at all!
        cam_returned[0] = camera_array
        cam_returned[1] = time
        lidar_returned[0] = [planner.localizationPositionX, planner.localizationPositionY,
                    planner.theta, planner.velocity, np.array([[0.001,0.0],[0.0,0.001]]).tolist()]
        if lidarRecognition != None:
            lidar_returned[1] = lidar_detected_error
            lidar_returned[2] = time
            planner.rawLidarDetections = point_cloud
    planner.groundTruth = camera_array
    planner.lidarPoints = point_cloud

    return cam_returned, lidar_returned

# This function is for full simulation where we fake both LIDAR and camera
# TODO: modularize this more so we can consider other sensor locations and facing angles
def fake_lidar_and_camera(detector, positions, objects, lidar_range,
                            lidar_center_angle, lidar_fov, cam_range,
                            cam_center_angle, cam_fov, parameterized_covariance):

        # print ( "FAKING LIDAR" )
        lidar_point_cloud = []
        lidar_point_cloud_error = []
        camera_array_searcher = []
        camera_array = []
        camera_error_array = []
        lidar_array_searcher = []
        lidar_detected_error = []
        cam_offset = 5000
        MAX_ID = 10000

        # Get the points the Slamware M1M1 should generate
        lidar_freq = 7000 / 8
        angle_change = (2 * math.pi) / lidar_freq

        # Create all the vehicle polygons and combine them into one big list
        polygons = []
        for idx, vehicle in enumerate(positions):
            # Create a bounding box for vehicle vehicle that is length + 2*buffer long and width + 2*buffer wide
            x1 = vehicle[0] + ((vehicle[4]/2.0)*math.cos(vehicle[2]+math.radians(90)) + ((vehicle[5]/2.0)*math.cos(vehicle[2]-math.radians(180))))
            y1 = vehicle[1] + ((vehicle[4]/2.0)*math.sin(vehicle[2]+math.radians(90)) + ((vehicle[5]/2.0)*math.sin(vehicle[2]-math.radians(180))))
            x2 = vehicle[0] + ((vehicle[4]/2.0)*math.cos(vehicle[2]-math.radians(90)) + ((vehicle[5]/2.0)*math.cos(vehicle[2]-math.radians(180))))
            y2 = vehicle[1] + ((vehicle[4]/2.0)*math.sin(vehicle[2]-math.radians(90)) + ((vehicle[5]/2.0)*math.sin(vehicle[2]-math.radians(180))))
            x3 = vehicle[0] + ((vehicle[4]/2.0)*math.cos(vehicle[2]-math.radians(90)) + ((vehicle[5]/2.0)*math.cos(vehicle[2])))
            y3 = vehicle[1] + ((vehicle[4]/2.0)*math.sin(vehicle[2]-math.radians(90)) + ((vehicle[5]/2.0)*math.sin(vehicle[2])))
            x4 = vehicle[0] + ((vehicle[4]/2.0)*math.cos(vehicle[2]+math.radians(90)) + ((vehicle[5]/2.0)*math.cos(vehicle[2])))
            y4 = vehicle[1] + ((vehicle[4]/2.0)*math.sin(vehicle[2]+math.radians(90)) + ((vehicle[5]/2.0)*math.sin(vehicle[2])))
            polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
            polygons.append(polygon)

        # Generate fake lidar data from the points using ray tracing
        for angle_idx in range(int(lidar_freq)):
            intersections = []
            intersections_origin_point = []
            intersections_count = 0
            intersect_dist = 9999999999
            final_point = None
            final_polygon = None

            # Go through all the polygons that the line intersects with and add them
            for poly in polygons:
                line = [(detector.localizationPositionX, detector.localizationPositionY), (
                detector.localizationPositionX + (lidar_range * math.cos(angle_idx * angle_change)),
                detector.localizationPositionY + (lidar_range * math.sin(angle_idx * angle_change)))]
                shapely_line = LineString(line)
                intersections += list(poly.intersection(shapely_line).coords)
                for idx in range(len(intersections) - intersections_count):
                    intersections_origin_point.append(poly)
                intersections_count = len(intersections)

            # Don't forget the other objects as well (already should be a list of polygons)
            for poly in objects:
                line = [(detector.localizationPositionX, detector.localizationPositionY), (
                detector.localizationPositionX + (lidar_range * math.cos(angle_idx * angle_change)),
                detector.localizationPositionY + (lidar_range * math.sin(angle_idx * angle_change)))]
                shapely_line = LineString(line)
                intersections += list(poly.intersection(shapely_line).coords)
                for idx in range(len(intersections) - intersections_count):
                    intersections_origin_point.append(poly)
                intersections_count = len(intersections)

            # Get the closest intersection with a polygon as that will be where our lidar beam stops
            idx_counter = 0
            for point, polygon in zip(intersections, intersections_origin_point):
                dist = math.hypot(point[0] - detector.localizationPositionX, point[1] - detector.localizationPositionY)
                if dist < intersect_dist:
                    final_point = point
                    intersect_dist = dist
                    final_polygon = polygon
                    final_polygon_id = idx_counter
                idx_counter += 1

            # Make sure this worked and is not None
            if final_point != None:
                lidar_point_cloud.append(final_point)
                # Generate error for the individual points
                x_error = np.random.normal(0, 0.05, 1)[0]
                y_error = np.random.normal(0, 0.05, 1)[0]
                lidar_point_cloud_error.append((final_point[0] + x_error, final_point[1] + y_error))

                # See if we can add a camera point as well
                if shared_math.check_in_range_and_fov(angle_idx * angle_change, intersect_dist, detector.theta + cam_center_angle,
                                               cam_fov, cam_range):
                    # Object checks out and is in range and not blocked
                    # TODO: Do a little better approxamation of percent seen and account for this
                    point = list(final_polygon.centroid.coords)[0]
                    if point not in camera_array_searcher:
                        # Create the error component of the camera detection
                        relative_angle_to_detector, target_line_angle, relative_distance = shared_math.get_relative_detection_params(
                            detector.localizationPositionX, detector.localizationPositionY, detector.theta, point[0], point[1])
                        success, expected_error_gaussian, actual_sim_error = detector.cameraSensor.calculateErrorGaussian(
                            relative_angle_to_detector, target_line_angle, relative_distance, True, parameterized_covariance)
                        if success:
                            universal_id = detector.id * MAX_ID + final_polygon_id
                            camera_error_array.append((universal_id, point[0] + actual_sim_error[0], point[1] + actual_sim_error[1], expected_error_gaussian.covariance.tolist(), 0.0, 0.0, []))
                            camera_array.append((universal_id, point[0], point[1], expected_error_gaussian.covariance.tolist(), 0.0, 0.0, []))
                            camera_array_searcher.append((point[0], point[1]))
                
                # Fast lidar math to skip the points
                if shared_math.check_in_range_and_fov(angle_idx * angle_change, intersect_dist, detector.theta + lidar_center_angle,
                                               lidar_fov, lidar_range):
                    # Object checks out and is in range and not blocked
                    # TODO: Do a little better approxamation of percent seen and account for this
                    point = list(final_polygon.centroid.coords)[0]
                    if point not in lidar_array_searcher:
                        # Create the error component of the lidar detection
                        relative_angle_to_detector, target_line_angle, relative_distance = shared_math.get_relative_detection_params(
                            detector.localizationPositionX, detector.localizationPositionY, detector.theta, point[0], point[1])
                        success_lidar = False
                        if detector.lidarSensor != None:
                            success_lidar, expected_error_gaussian_lidar, actual_sim_error_lidar = detector.lidarSensor.calculateErrorGaussian(
                                relative_angle_to_detector, target_line_angle, relative_distance, True, parameterized_covariance)
                        else:
                            success_lidar = 0.0
                            expected_error_gaussian_lidar = 0.0
                            actual_sim_error_lidar = 0.0

                        if success_lidar:
                            universal_id = detector.id * MAX_ID + final_polygon_id + cam_offset
                            lidar_detected_error.append((universal_id, point[0] + actual_sim_error_lidar[0], point[1] + actual_sim_error_lidar[1], expected_error_gaussian_lidar.covariance.tolist(), 0.0, 0.0, []))
                            lidar_array_searcher.append((point[0], point[1]))

        return lidar_point_cloud, lidar_point_cloud_error, camera_array, camera_error_array, lidar_detected_error

# Use to check if a sensor should have detected an object or not
def check_visble_objects(sensor_position, sensor_center_angle, sensor_range, sensor_fov, object_polygons, vehicle_width = .5):
    # Get the points the Slamware M1M1 should generate
    lidar_freq = 7000 / 8
    angle_change = (2 * math.pi) / lidar_freq
    seen_point_set = {}
    available_point_set = {}
    sensor_angle = sensor_position[2] + sensor_center_angle
    final_set = []

    # Generate fake lidar data from the points using ray tracing
    for angle_idx in range(int(lidar_freq)):
        intersections = []
        intersection_id = []
        intersect_dist = 9999999999
        final_point = None

        #print(angle_idx * angle_change, sensor_angle, sensor_fov)
        if shared_math.check_in_fov(angle_idx * angle_change,
                                    sensor_angle, sensor_fov):
            #print("in")

            # Go through all the polygons that the line intersects with and add them
            for obj_idx, poly in enumerate(object_polygons):
                line = [(sensor_position[0], sensor_position[1]), (
                sensor_position[0] + (sensor_range * math.cos(angle_idx * angle_change)),
                sensor_position[1] + (sensor_range * math.sin(angle_idx * angle_change)))]
                shapely_line = LineString(line)
                temp_intersection_list = list(poly.intersection(shapely_line).coords)
                intersections += temp_intersection_list
                for count in range(len(temp_intersection_list)):
                    intersection_id.append(obj_idx)

            # Get the closest intersection with a polygon as that will be where our lidar beam stops
            for point, polygon_id in zip(intersections, intersection_id):
                dist = math.hypot(point[0] - sensor_position[0], point[1] - sensor_position[1])
                if sensor_range >= dist and dist >= vehicle_width:
                    if dist < intersect_dist:
                        final_point = point
                        intersect_dist = dist
                        final_polygon_id = polygon_id
                    
                    # Add a point to all polygons IDs in the path
                    if polygon_id in available_point_set:
                        available_point_set[polygon_id] += 1
                    else:
                        available_point_set[polygon_id] = 1

            # If we have a final point, add to the list
            if final_point != None:
                # Add this point to the set so we can figure out if we see enough of the object
                if final_polygon_id in seen_point_set:
                    seen_point_set[final_polygon_id] += 1
                else:
                    seen_point_set[final_polygon_id] = 1

    # Here we do the final check to see what percentage of each we can see
    for key in seen_point_set.keys():
        if seen_point_set[key] >= .5 * available_point_set[key]:
            # 25% or more is visible, add it to the should see it list
            final_set.append(key)

    return final_set