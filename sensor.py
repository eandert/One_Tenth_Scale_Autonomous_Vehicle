import math
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry.linestring import LineString

import shared_math


class BivariateGaussian:
    # This class is for creating and storing error terms of a sensor using 
    # mu and covariance so they can be fed into kalman filters and printed
    def __init__(self, a, b, phi, mu = None, cov = None):
        if cov is None:
            # Create the bivariate gaussian matrix
            self.mu = np.array([0.0,0.0])
            self.covariance = np.array([[a*a, 0], [0, b*b]])

            # RΣR^T to rotate the ellipse where Σ is the original covariance matrix
            rotate = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
            self.covariance = np.matmul(rotate, self.covariance)
            self.covariance = np.matmul(self.covariance, rotate.transpose())
        else:
            # Create the bivariate gaussian matrix
            self.mu = mu
            self.covariance = cov

    # def ellipsify(self, meters_to_print_scale, num = 50, multiplier = 3):
    #     # Default multiplier is 3 because that should cover 99.7% of errors
    #     a, b, phi = self.extractErrorElipseParamsFromBivariateGaussian()
    #     a = math.pow(a, 2)
    #     b = math.pow(b, 2)
    #     #print("a (ellipsify) ", str(a))
    #     #print("b (ellipsify)", str(b))
    #     #print("phi (ellipsify) ", str(math.degrees(phi)))
    #     pointEvery = math.radians(360/num)
    #     ellipse = QtGui.QPolygonF()
    #     for count in range(num + 1):
    #         cur_angle = pointEvery * count
    #         range_val = self.calculateRadiusAtAngle(a, b, phi, cur_angle) * multiplier
    #         x_val = self.mu[0] + range_val * math.cos(cur_angle)
    #         y_val = self.mu[1] + range_val * math.sin(cur_angle)
    #         ellipse.append((x_val * meters_to_print_scale, y_val * meters_to_print_scale))

    #     return ellipse

    def dot(self):

        ellipse = []

        ellipse.append([self.mu[0], self.mu[1]])
        ellipse.append([self.mu[0], self.mu[1]+.1])
        ellipse.append([self.mu[0], self.mu[1]-.1])
        ellipse.append([self.mu[0], self.mu[1]])
        ellipse.append([self.mu[0]+.1, self.mu[1]])
        ellipse.append([self.mu[0]-.1, self.mu[1]])
        ellipse.append([self.mu[0], self.mu[1]])

        return ellipse

    def calculateRadiusAtAngle(self, a, b, phi, measurementAngle):
        denominator = math.sqrt( a**2 * math.sin(phi-measurementAngle)**2 + b**2 * math.cos(phi-measurementAngle)**2 )
        if denominator == 0:
            print ( "Warning: calculateEllipseRadius denom 0! - check localizer definitions " )
            #print ( a, b, phi, measurementAngle )
            return 0
        else:
            return ( a * b ) / math.sqrt( a**2 * math.sin(phi-measurementAngle)**2 + b**2 * math.cos(phi-measurementAngle)**2 )

    def calcSelfRadiusAtAnlge(self, angle, num_std_deviations):
        a, b, phi = self.extractErrorElipseParamsFromBivariateGaussian(num_std_deviations)
        #print ( a, b, phi)
        return self.calculateRadiusAtAngle(a, b, phi, angle)

    # Deprecaited (and wrong!)
    # def extractErrorElipseParamsFromBivariateGaussian(self):
    #     # TODO: Change this to https://stats.stackexchange.com/questions/361017/proper-way-of-estimating-the-covariance-error-ellipse-in-2d
    #     # Eigenvalue and eigenvector computations
    #     w, v = np.linalg.eig(self.covariance)

    #     # Use the eigenvalue to figure out which direction is larger
    #     if abs(w[0]) > abs(w[1]):
    #         a = abs(w[0])
    #         b = abs(w[1])
    #         phi = math.atan2(v[0, 0], v[1, 0])
    #     else:
    #         a = abs(w[1])
    #         b = abs(w[0])
    #         phi = math.atan2(v[0, 1], v[1, 1])

    #     return a, b, phi

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

        vals, vecs = self.eigsorted()
        phi = np.arctan2(*vecs[:, 0][::-1])

        # A and B are radii
        a, b = num_std_deviations * np.sqrt(vals)

        return a, b, phi

    def calcXYComponents(self):
        x_comp = abs(self.calcSelfRadiusAtAnlge(math.radians(0)), 1)
        y_comp = abs(self.calcSelfRadiusAtAnlge(math.radians(90)), 1)
        return x_comp, y_comp


def intersectionBivariateGaussiansCovariance(covariance_gaussian_a, covariance_gaussian_b):
    # Now we have both distributions, it is time to do the combination math
    # Source: http://www.geostatisticslessons.com/lessons/errorellipses
    covariance_new = np.subtract(covariance_gaussian_a.transpose(), covariance_gaussian_b.transpose()).transpose()

    return covariance_new


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

    def checkInRangeAndFOV(self, object_angle, object_distance):
        anglediff = ((self.center_angle - object_angle + math.pi + (2*math.pi)) % (2*math.pi)) - math.pi
        if abs(anglediff) <= (self.field_of_view) and (object_distance <= self.max_distance):
            return True
        return False
        
    def getRadialErrorAtDistance(self, object_distance):
        # This version we are disregarding horizontal crosssection for now
        return self.radial_error_b + (object_distance * self.radial_error_x)
        
    def getDistanceErrorAtDistance(self, object_distance):
        return self.distal_error_b + (object_distance * self.distal_error_x)

    def calculateErrorGaussian(self, object_relative_angle_to_detector, target_line_angle, object_distance, simulation):
        # TODO: check ATLAS code and makse sure this is the same
        if self.checkInRangeAndFOV(object_relative_angle_to_detector, object_distance):
            radial_error = self.getRadialErrorAtDistance(object_distance)
            distal_error = self.getDistanceErrorAtDistance(object_distance)
            # Calculate our expected elipse error bounds
            elipse_a_expected = 2 * (object_distance * math.sin(radial_error / 2))
            elipse_b_expected = distal_error
            if elipse_a_expected < elipse_b_expected:
                elipse_temp = elipse_a_expected
                elipse_a_expected = elipse_b_expected
                elipse_b_expected = elipse_temp
                elipse_angle_expected = target_line_angle
            else:
                elipse_angle_expected = target_line_angle + math.radians(90)
            expected_error_gaussian = BivariateGaussian(elipse_a_expected,
                                      elipse_b_expected,
                                      elipse_angle_expected)
            #print("ae: ", elipse_a_expected, elipse_b_expected, elipse_angle_expected)
            #print("ae2: ", expected_error_gaussian.calcSelfRadiusAtAnlge(elipse_angle_expected, 1))
            # Calcuate the actual error if this is a simulation, otherwise just return
            if simulation:
                # Calculate our expected errors in x,y coordinates
                #print("de:", distal_error, " re:", 2 * (object_distance * math.sin(radial_error / 2)))
                actualRadialError = np.random.normal(0, radial_error, 1)[0]
                actualDistanceError = np.random.normal(0, elipse_b_expected, 1)[0]
                x_error_generated = ((object_distance + actualDistanceError) * math.cos(
                    target_line_angle + actualRadialError))
                y_error_generated = ((object_distance + actualDistanceError) * math.sin(
                    target_line_angle + actualRadialError))
                x_actual = ((object_distance) * math.cos(
                    target_line_angle))
                y_actual = ((object_distance) * math.sin(
                    target_line_angle))
                actual_sim_error = [x_error_generated - x_actual, y_error_generated - y_actual]
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


# This function is for full simulation where we fake both LIDAR and camera
# TODO: modularize this more so we can consider other sensor locations and facing angles
def fake_lidar_and_camera(detector, positions, objects, lidar_range, cam_range,
                              cam_center_angle, cam_fov):

        # print ( "FAKING LIDAR" )
        lidar_point_cloud = []
        lidar_point_cloud_error = []
        camera_array_searcher = []
        camera_array = []
        camera_error_array = []

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
            for point, polygon in zip(intersections, intersections_origin_point):
                dist = math.hypot(point[0] - detector.localizationPositionX, point[1] - detector.localizationPositionY)
                if dist < intersect_dist:
                    final_point = point
                    intersect_dist = dist
                    final_polygon = polygon

            # Make sure this worked and is not None
            if final_point != None:
                lidar_point_cloud.append(final_point)
                # Generate error for the individual points
                x_error = np.random.normal(0, 0.05, 1)[0]
                y_error = np.random.normal(0, 0.05, 1)[0]
                lidar_point_cloud_error.append((final_point[0] + x_error, final_point[1] + y_error))

                # See if we can add a camera point as well
                if shared_math.check_in_range_and_fov(angle_idx * angle_change, intersect_dist, detector.theta + cam_center_angle,
                                               math.radians(cam_fov), cam_range):
                    # Object checks out and is in range and not blocked
                    # TODO: Do a little better approxamation of percent seen and account for this
                    point = list(final_polygon.centroid.coords)[0]
                    if point not in camera_array_searcher:
                        # Create the error component of the camera detection
                        relative_angle_to_detector, target_line_angle, relative_distance = shared_math.get_relative_detection_params(
                            detector.localizationPositionX, detector.localizationPositionY, detector.theta, point[0], point[1])
                        success, expected_error_gaussian, actual_sim_error = detector.cameraSensor.calculateErrorGaussian(
                            relative_angle_to_detector, target_line_angle, relative_distance, True)
                        #print ( success, point, expected_error_gaussian, actual_sim_error )
                        #print ( detector.localizationPositionX, detector.localizationPositionY )
                        if success:
                            camera_error_array.append((point[0] + actual_sim_error[0], point[1] + actual_sim_error[1], expected_error_gaussian))
                            camera_array.append((point[0], point[1], expected_error_gaussian))
                            camera_array_searcher.append((point[0], point[1]))

        return lidar_point_cloud, lidar_point_cloud_error, camera_array, camera_error_array