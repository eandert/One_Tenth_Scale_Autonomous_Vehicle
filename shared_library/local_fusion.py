import math
import numpy as np
from numpy.testing._private.utils import measure
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
#from filterpy.kalman import UnscentedKalmanFilter as UKF
#from filterpy.kalman import MerweScaledSigmaPoints
#from stonesoup.types.track import Track

from shared_library import shared_math

CAMERA = 0
LIDAR = 1
MAX_ID = 10000


class MatchClass:
    def __init__(self, x, y, covariance, dx, dy, d_confidence, sensor_type, time, sensor_id):
        self.x = x
        self.y = y
        self.covariance = covariance
        self.dx = dx
        self.dy = dy
        self.velocity_confidence = d_confidence
        self.sensor_type = sensor_type
        self.last_tracked = time
        self.sensor_id = sensor_id


class Tracked:
    # This object tracks a single object that has been detected in a video frame.
    # We use this primarily to match objects seen between frames and included in here
    # is a function for kalman filter to smooth the x and y values as well as a
    # function for prediction where the next bounding box will be based on prior movement.
    def __init__(self, sensor_id, x, y, sensor_type, time, id, fusion_mode):
        self.x = x
        self.y = y
        self.dx = 0
        self.dy = 0
        self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')
        self.lastTracked = time
        self.id = id
        self.idx = 0
        self.min_size = 0.5
        self.dt = .125
        self.track_count = 0
        self.fusion_steps = 0
        self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype = 'float')
        self.first = True

        # Build the match list and add our match to it
        self.match_list = []
        new_match = MatchClass(x, y, None, None, None, None, sensor_type, time, sensor_id)
        self.match_list.append(new_match)

        # Kalman stuff
        self.fusion_mode = fusion_mode
        self.ukf_mode = False

        process_variation = 0.0625

        # Set other parameters for the class
        self.prev_time = -99

        if self.fusion_mode == 0:
            # Set up the Kalman filter
            # Initial State cov
            self.P_t = np.identity(4)
            self.P_t[2][2] = 0.0
            self.P_t[3][3] = 0.0
            self.P_hat_t = self.P_t
            # Process cov
            four = process_variation * (.125*.125*.125*.125)/4.0
            three = process_variation * (.125*.125*.125)/3.0
            two = process_variation * (.125*.125)/2.0
            one = process_variation * (.125)
            self.Q_t = np.array([[three, 0, two, 0],
                                [0, three, 0, two],
                                [two, 0, process_variation, 0],
                                [0, two, 0, process_variation]], dtype = 'float')
            # Control matrix
            self.B_t = np.array([[0], [0], [0], [0]], dtype = 'float')
            #self.B_t = G
            # Control vector
            #self.U_t = 0
            self.U_t = 0
            # Measurment Matrix
            # Generated on the fly
            # Measurment cov
            self.R_t = np.identity(4)
        elif self.fusion_mode == 1:
            # Setup for x_hat = x + dx + dxdx,  y_hat = y + dy + dydy
            # Set up the Kalman filter
            # Initial State cov
            self.P_t = np.identity(6)
            self.P_t[2][2] = 0.0
            self.P_t[3][3] = 0.0
            self.P_t[4][4] = 0.0
            self.P_t[5][5] = 0.0
            self.P_hat_t = self.P_t
            # Process cov
            #self.Q_t = np.identity(6)
            five = process_variation * (.125*.125*.125*.125*.125)/8.0
            four = process_variation * (.125*.125*.125*.125)/4.0
            three = process_variation * (.125*.125*.125)/2.0
            two = process_variation * (.125*.125)
            self.Q_t = np.array([[five, 0, four, 0, 0, 0],
                                [0, five, 0, four, 0, 0],
                                [four, 0, three, 0, three, 0],
                                [0, four, 0, three, 0, three],
                                [0, 0, three, 0, two, 0],
                                [0, 0, 0, three, 0, two]], dtype = 'float')
            # Control matrix
            self.B_t = np.array([[0], [0], [0], [0], [0], [0]], dtype = 'float')
            # Control vector
            self.U_t = 0
            # Measurment cov
            self.R_t = np.identity(6)
        else:
            # model from https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
            # Set up the Kalman filter
            # Initial State cov
            self.P_t = np.identity(5)
            self.P_t[2][2] = 0.0
            self.P_t[3][3] = 0.0
            self.P_t[4][4] = 0.0
            self.P_hat_t = self.P_t
            # Process cov
            #self.Q_t = np.identity(5)
            four = process_variation * (.125*.125*.125*.125)/4.0
            three = process_variation * (.125*.125*.125)/2.0
            angle_variation = process_variation
            two = angle_variation * (.125*.125)
            self.Q_t = np.array([[four, 0, three, 0, 0],
                                [0, four, 0, three, 0],
                                [three, three, two, 0, 0],
                                [0, 0, 0, two, 0],
                                [0, 0, 0, 0, two]], dtype = 'float')
            # Control matrix
            self.B_t = np.array([[0], [0], [0], [0], [0]], dtype = 'float')
            # Control vector
            self.U_t = 0
            # Measurment cov
            self.R_t = np.identity(5)


        # Custom things
        self.lidarMeasurePrevTrue = False
        self.camMeasurePrevTrue = False
        self.lidarMeasurePrevCovTrue = False
        self.camMeasurePrevCovTrue = False

    # Update adds another detection to this track
    def update(self, other, time):
        
        new_match = MatchClass(other[1], other[2], None, None, None, None, other[3], time, other[0])
        self.match_list.append(new_match)

        self.lastTracked = time

        self.track_count += 1

    # Gets our position in an array form so we can use it in the BallTree
    def getPosition(self):
        return [
            [self.x, self.y, self.min_size, self.min_size, math.radians(0)]
        ]

    def getPositionPredicted(self, timestamp, estimate_covariance):
        if self.fusion_steps < 2 or not estimate_covariance:
        # If this kalman fitler has never been run, we can't use it for prediction!
            return [
                [self.x, self.y, self.min_size, self.min_size, math.radians(0)]
            ]
        else:
            try:
                x, y, a, b, phi = self.getKalmanPred(timestamp)
                return [
                    [x, y, a, b, phi]
                ]
            except:
                return [
                [self.x, self.y, self.min_size, self.min_size, math.radians(0)]
                ]

    def clearLastFrame(self):
        self.match_list = []

    def fx(self, x, dt):
        """ state transition function for a 
        constant velocity aircraft"""
    
        F = np.array([[1, 0, dt, 0],
                     [0, 1, 0, dt],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype = 'float')
                     
        return F @ x

    def hx(self, x):
        ret_val = self.tempH_t
        ret_val[0][0] = ret_val[0][0] * x[0]
        ret_val[1][1] = ret_val[1][1] * x[1]
        ret_val[2][2] = ret_val[2][0] * x[2]
        ret_val[3][3] = ret_val[3][1] * x[3]
        return ret_val

    def fusion(self, parameterized_covariance, vehicle, predictive):
        lidarCov = np.array([[0.0, 0.0], [0.0, 0.0]])
        camCov = np.array([[0.0, 0.0], [0.0, 0.0]])
        lidarMeasure = [0, 0]
        camMeasure = [0, 0]
        lidarMeasureH = [0, 0]
        camMeasureH = [0, 0]

        lidar_added = False
        cam_added = False

        # Time to go through the track list and fuse!
        for match in self.match_list:
            if match.sensor_type == LIDAR:
                relative_angle_to_detector, target_line_angle, relative_distance = shared_math.get_relative_detection_params(
                    vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, match.x, match.y)
                success, lidar_expected_error_gaussian, actual_sim_error = vehicle.lidarSensor.calculateErrorGaussian(
                    relative_angle_to_detector, target_line_angle, relative_distance, False, parameterized_covariance)
                try:
                    lidarCov = lidar_expected_error_gaussian.covariance
                except:
                    lidarCov = np.array([[0.1, 0.0], [0.0, 0.1]])
                self.lidarMeasurePrevCovTrue = True
                # Set the new measurements
                lidarMeasure = [match.x, match.y]
                lidarMeasureH = [1, 1]
                lidar_added = True
            elif match.sensor_type == CAMERA:
                relative_angle_to_detector, target_line_angle, relative_distance = shared_math.get_relative_detection_params(
                    vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, match.x, match.y)
                success, camera_expected_error_gaussian, actual_sim_error = vehicle.cameraSensor.calculateErrorGaussian(
                    relative_angle_to_detector, target_line_angle, relative_distance, False, parameterized_covariance)
                try:
                    camCov = camera_expected_error_gaussian.covariance
                except:
                    camCov = np.array([[0.1, 0.0], [0.0, 0.1]])
                # Set the new measurements
                camMeasure = [match.x, match.y]
                camMeasureH = [1, 1]
                cam_added = True

        # Now do the kalman thing!
        if self.idx == 0:
            # Calculate the covariance to be sent out with the result
            if lidar_added and cam_added:
                try:
                    #self.error_covariance = intersectionBivariateGaussiansCovariance(camCov, lidarCov)
                    self.error_covariance = np.mean( np.array([ camCov, lidarCov ]), axis=0 )
                    #print (  "combined! ", camCov, lidarCov, self.error_covariance )
                except Exception as e:
                    print ( " Exception: " + str(e) )
            elif lidar_added:
                self.error_covariance = lidarCov
            elif cam_added:
                self.error_covariance = camCov
            else:
                self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')

            # We have no prior detection so we need to just output what we have but store for later
            # Do a Naive average to get the starting position
            if camMeasure[0] != 0 and lidarMeasure[0]!= 0:
                x_out = (camMeasure[0] + lidarMeasure[0]) / 2.0
                y_out = (camMeasure[1] + lidarMeasure[1]) / 2.0
            elif camMeasure[0] != 0:
                x_out = camMeasure[0]
                y_out = camMeasure[1]
            elif lidarMeasure[0] != 0:
                x_out = lidarMeasure[0]
                y_out = lidarMeasure[1]

            if self.fusion_mode == 0:
                # Store so that next fusion is better
                self.X_hat_t = np.array([[x_out], [y_out], [0], [0]], dtype = 'float')
                # if self.ukf_mode:
                #     self.F_t = np.array([[1, 0, self.dt, 0],
                #                         [0, 1, 0, self.dt],
                #                         [0, 0, 1, 0],
                #                         [0, 0, 0, 1]], dtype = 'float')
                #     self.tempH_t = np.array([[lidarMeasureH[0], 0., 0., 0.],
                #                     [0., lidarMeasureH[1], 0., 0.],
                #                     [camMeasureH[0], 0., 0., 0.],
                #                     [0., camMeasureH[1], 0., 0.]], dtype = 'float')
                #     sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=0.)
                #     self.ukf = UKF(dim_x=4, dim_z=4, fx=self.fx, hx=self.hx, dt=self.dt, points=sigmas)
                #     self.ukf.Q = self.Q_t
                #     self.ukf.x = self.X_hat_t
            elif self.fusion_mode == 1:
                # setup for x, dx, dxdx
                self.X_hat_t = np.array([[x_out], [y_out], [0], [0], [0], [0]], dtype = 'float')
            else:
                # setup for https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
                self.X_hat_t = np.array(
                    [[x_out], [y_out], [0], [0], [0]], dtype = 'float')
            
            # Seed the covariance values directly from the measurement
            self.P_t[0][0] = self.error_covariance[0][0]
            self.P_t[0][1] = self.error_covariance[0][1]
            self.P_t[1][0] = self.error_covariance[1][0]
            self.P_t[1][1] = self.error_covariance[1][1]
            self.P_hat_t = self.P_t
            self.prev_time = self.lastTracked
            self.x = x_out
            self.y = y_out
            self.dx = 0.0
            self.dy = 0.0
            self.idx += 1
        else:
            #try:
            # We have valid data
            # Transition matrix
            # Prediction step!
            elapsed = self.lastTracked - self.prev_time
            if elapsed <= 0.0:
                #print( "Error time elapsed is incorrect! " + str(elapsed) )
                # Set to arbitrary time
                elapsed = 0.125

            if self.fusion_mode == 0:
                self.F_t = np.array([[1, 0, elapsed, 0],
                                    [0, 1, 0, elapsed],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], dtype = 'float')
            elif self.fusion_mode == 1:
                # setup for x, dx, dxdx
                self.F_t = np.array([[1, 0, elapsed, 0, elapsed*elapsed, 0],
                                    [0, 1, 0, elapsed, 0, elapsed*elapsed],
                                    [0, 0, 1, 0, elapsed, 0],
                                    [0, 0, 0, 1, 0, elapsed],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]], dtype = 'float')
            else:
                # setup for https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
                v_1 = self.X_hat_t[2]
                phi_1 = self.X_hat_t[3]
                phi_dot_1 = self.X_hat_t[4]
                #print ( self.X_hat_t, v_1, phi_1, phi_dot_1 )
                if phi_1 == 0.0:
                    # Not moving, No correlation
                    v_x = 999.9
                    v_y = 999.9
                    phi_x = 999.9
                    phi_y = 999.9
                else:
                    v_x = ( 1.0 / phi_dot_1 ) * ( -math.sin(phi_1) + math.sin(phi_1 + elapsed * phi_dot_1))
                    v_y = ( 1.0 / phi_dot_1 ) * ( math.cos(phi_1) - math.cos(phi_1 + elapsed * phi_dot_1))
                    phi_x = ( v_1 / phi_dot_1 ) * ( -math.cos(phi_1) + math.cos(phi_1 + elapsed * phi_dot_1))
                    phi_y = ( v_1 / phi_dot_1 ) * ( -math.sin(phi_1) + math.sin(phi_1 + elapsed * phi_dot_1))
                if phi_dot_1 == 0.0:
                    # Not accelerating, NO correlation
                    phi_dot_x = 999.9
                    phi_dot_y = 999.9
                else:
                    phi_dot_x = ( v_1 * elapsed / phi_dot_1 ) * math.cos(phi_1 + elapsed * phi_dot_1) - ( v_1 / phi_dot_1**2 ) * ( - math.sin(phi_1) + math.sin(phi_1 + elapsed * phi_dot_1))
                    phi_dot_y = ( v_1 * elapsed / phi_dot_1 ) * math.sin(phi_1 + elapsed * phi_dot_1) - ( v_1 / phi_dot_1**2 ) * ( math.cos(phi_1) - math.cos(phi_1 + elapsed * phi_dot_1))
                self.F_t = np.array([[1, 0, v_x, phi_x, phi_dot_x],
                                    [0, 1, v_y, phi_y, phi_dot_y],
                                    [0, 0, 1, 0, 0],
                                    [0, 0, 0, 1, elapsed],
                                    [0, 0, 0, 0, 1]], dtype = 'float')
            
            # if self.ukf_mode:
            #     self.ukf.R = self.R_t
            #     #print ( self.ukf.z )
            #     print ( 1 )
            #     #self.ukf.fx = self.F_t
            #     print ( 2 )
            #     #self.ukf.hx = self.tempH_t
            #     print ( 3 )
            #     self.ukf.predict()
            #     #print ( self.ukf.z )
            #     print ( 4 )
            #     self.ukf.update(self.measure)
            #     print ( 5 )
            #     X_t = self.ukf.x
            #     self.P_t = self.ukf.P
            #     print(X_t, self.P_t)
            # else:

            self.X_hat_t, self.P_hat_t = shared_math.kalman_prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

            # print ( "m ", measure )
            #print ( self.tempH_t )
            # print ( self.R_t )

            #print ( "P_hat: ", self.P_hat_t, " P_t: ", self.P_t )

            # Time to run our predictions!
            # Fist lets do the camera update
            # Fist lets do the camera update
            if lidar_added:
                if self.fusion_mode == 0:
                    H_t = np.array([[lidarMeasureH[0], 0., 0., 0.],
                                   [0., lidarMeasureH[1], 0., 0.]], dtype = 'float')
                elif self.fusion_mode == 1:
                    H_t = np.array([[lidarMeasureH[0], 0., 0., 0., 0., 0.],
                                   [0., lidarMeasureH[1], 0., 0., 0., 0.]], dtype = 'float')
                elif self.fusion_mode == 2:
                    H_t = np.array([[lidarMeasureH[0], 0., 0., 0., 0.],
                                   [0., lidarMeasureH[1], 0., 0., 0.]], dtype = 'float')
                measure = np.array([lidarMeasure[0], lidarMeasure[1]], dtype = 'float')
                covariance = lidarCov
            elif cam_added:
                if self.fusion_mode == 0:
                    H_t = np.array([[camMeasureH[0], 0., 0., 0.],
                                   [0., camMeasureH[1], 0., 0.]], dtype = 'float')
                elif self.fusion_mode == 1:
                    H_t = np.array([[camMeasureH[0], 0., 0., 0., 0., 0.],
                                   [0., camMeasureH[1], 0., 0., 0., 0.]], dtype = 'float')
                elif self.fusion_mode == 2:
                    H_t = np.array([[camMeasureH[0], 0., 0., 0., 0.],
                                   [0., camMeasureH[1], 0., 0., 0.]], dtype = 'float')
                measure = np.array([camMeasure[0], camMeasure[1]], dtype = 'float')
                covariance = camCov
            else:
                if self.fusion_mode == 0:
                    H_t = np.array([[0., 0., 0., 0.],
                                   [0., 0., 0., 0.]], dtype = 'float')
                elif self.fusion_mode == 1:
                    H_t = np.array([[0., 0., 0., 0., 0., 0.],
                                   [0., 0., 0., 0., 0., 0.]], dtype = 'float')
                elif self.fusion_mode == 2:
                    H_t = np.array([[0., 0., 0., 0., 0.],
                                   [0., 0., 0., 0., 0.]], dtype = 'float')
                measure = np.array([.0, .0], dtype = 'float')
                covariance = np.array([[1.0, 0.0], [0.0, 1.0]])

            Z_t = (measure).transpose()
            Z_t = Z_t.reshape(Z_t.shape[0], -1)
            # print ( "z_t2", Z_t )
            X_t, P_hat_t = shared_math.kalman_update(self.X_hat_t, self.P_hat_t, Z_t, covariance, H_t)
            self.X_hat_t = X_t
            self.P_t = P_hat_t

            # Here we run an extra predictive step since it takes 125 ms to compute our data, but
            # we do not save any of this and will re-run next time
            if predictive:
                X_t, P_hat_t = shared_math.kalman_prediction(X_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

            self.prev_time = self.lastTracked

            self.x = X_t[0][0]
            self.y = X_t[1][0]
            if self.fusion_mode == 2:
                # Different setup for fusion mode 2 need to calculate vector from angle and velocity
                self.dx = X_t[2][0] * math.cos(X_t[3][0])
                self.dy = X_t[2][0] * math.sin(X_t[3][0])
            else:
                self.dx = X_t[2][0]
                self.dy = X_t[3][0]
            self.idx += 1
            if P_hat_t[0][0] != 0.0 or P_hat_t[0][1] != 0.0:
                self.error_covariance = np.array([[P_hat_t[0][0], P_hat_t[0][1]], [P_hat_t[1][0], P_hat_t[1][1]]], dtype = 'float')
                self.d_covariance = np.array([[P_hat_t[2][2], P_hat_t[2][3]], [P_hat_t[3][2], P_hat_t[3][3]]], dtype = 'float')
            else:
                #print ( " what the heck: ", P_hat_t)
                self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')
                self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype = 'float')

            self.fusion_steps += 1

            #print ( elapsed, self.x, self.y, self.dx, self.dy, math.degrees(math.hypot(self.dx, self.dy)))
            # except Exception as e:
            #     print ( " Exception: " + str(e) )

    def getKalmanPred(self, time):
        # Only let the kalman predict after it has settled for a minute
        # if self.idx > 7:
        #     elapsed = time - self.prev_time + .125
        #     if elapsed <= 0.0:
        #         print("Error time elapsed is incorrect, must be matched! " + str(elapsed))
        #         # Set to last calcualtion
        #         return self.x, self.y, self.dx, self.dy, math.radians(0)
        #     else:
        #         if self.fusion_mode == 0:
        #             self.F_t = np.array([[1, 0, elapsed, 0],
        #                                 [0, 1, 0, elapsed],
        #                                 [0, 0, 1, 0],
        #                                 [0, 0, 0, 1]])
        #         elif self.fusion_mode == 1:
        #             # setup for x, dx, dxdx
        #             self.F_t = np.array([[1, 0, elapsed, 0, elapsed*elapsed, 0],
        #                                 [0, 1, 0, elapsed, 0, elapsed*elapsed],
        #                                 [0, 0, 1, 0, elapsed, 0],
        #                                 [0, 0, 0, 1, 0, elapsed],
        #                                 [0, 0, 0, 0, 1, 0],
        #                                 [0, 0, 0, 0, 0, 1]])
        #         else:
        #             # setup for https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
        #             v_1 = self.X_hat_t[2]
        #             phi_1 = self.X_hat_t[3]
        #             phi_dot_1 = self.X_hat_t[4]
        #             if v_1 == 0.0:
        #                 # Not moving, infinite correlation
        #                 v_x = 1000000.0
        #                 v_y = 1000000.0
        #             else:
        #                 v_x = ( 1.0 / phi_1 ) * ( -math.sin(phi_1) + math.sin(phi_1 + elapsed * phi_dot_1))
        #                 v_y = ( 1.0 / phi_1 ) * ( math.cos(phi_1) - math.cos(phi_1 + elapsed * phi_dot_1))
        #             if phi_1 == 0.0:
        #                 # Not moving, infinite correlation
        #                 phi_x = 1000000.0
        #                 phi_y = 1000000.0
        #             else:
        #                 phi_x = ( v_1 / phi_1 ) * ( -math.cos(phi_1) + math.cos(phi_1 + elapsed * phi_dot_1))
        #                 phi_y = ( v_1 / phi_1 ) * ( -math.sin(phi_1) + math.sin(phi_1 + elapsed * phi_dot_1))
        #             if phi_dot_1 == 0.0:
        #                 # Not accelerating, infinite correlation
        #                 phi_dot_x = 1000000.0
        #                 phi_dot_y = 1000000.0
        #             else:
        #                 phi_dot_x = ( v_1 * elapsed / phi_dot_1 ) * math.cos(phi_1 + elapsed * phi_dot_1) - ( v_1 / phi_dot_1**2 ) * ( - math.sin(phi_1) + math.sin(phi_1 + elapsed * phi_dot_1))
        #                 phi_dot_y = ( v_1 * elapsed / phi_dot_1 ) * math.sin(phi_1 + elapsed * phi_dot_1) - ( v_1 / phi_dot_1**2 ) * ( math.cos(phi_1) - math.cos(phi_1 + elapsed * phi_dot_1))
        #             self.F_t = np.array([[1, 0, v_x, phi_x, phi_dot_x],
        #                                 [0, 1, v_y, phi_y, phi_dot_y],
        #                                 [0, 0, 1, 0, 0],
        #                                 [0, 0, 0, 1, elapsed],
        #                                 [0, 0, 0, 0, 1]])


        #         X_hat_t, P_hat_t = shared_math.kalman_prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

        #         # Now convert the covariance to a rectangle
        #         return X_hat_t[0][0], X_hat_t[1][0], P_hat_t[0][0], P_hat_t[1][1], math.radians(0)
        #     #return self.x, self.y, self.min_size, self.min_size, math.radians(0)
        # else:
        #     return self.x, self.y, self.min_size, self.min_size, math.radians(0)
        
        # Prediction based mathcing methods seems to be making this fail so we are using no prediction :/
        # Enforce a min size of a vehicle so that a detection has some area overlap to check
        a, b, phi = shared_math.ellipsify(self.error_covariance, 3.0)
        return self.x, self.y, self.min_size + a, self.min_size + b, phi


class FUSION:
    # Fusion is a special class for matching and fusing detections for a variety of sources.
    # The inpus is scalable and therefore must be generated before being fed into this class.
    # A unique list of detections is required from each individual sensor or pre-fused device
    # output or it will not be matched. Detections too close to each other may be combined.
    # This is a modified version of the frame-by-frame tracker seen in:
    # https://github.com/eandert/Jetson_Nano_Camera_Vehicle_Tracker
    def __init__(self, fusion_mode, cav_cis_id):
        # Set other parameters for the class
        self.trackedList = []
        self.id = cav_cis_id
        self.prev_time = -99.0
        self.min_size = 0.5
        self.fusion_mode = 2 #fusion_mode
        self.current_tracked_id = 0
        self.trackShowThreshold = 4
        self.predictive = True

        # Indicate our success
        print('Started FUSION successfully...')

    def fuseDetectionFrame(self, parameterized_covariance, vehicle):
        # Time to go through each track list and fuse!
        result = []
        for track in self.trackedList:
            track.fusion(parameterized_covariance, vehicle, self.predictive)
            if track.fusion_steps >= self.trackShowThreshold:
                # Calculate a custom ID that encodes the sensor id and local fusion track number
                universal_id = self.id * MAX_ID + track.id
                result.append([universal_id, track.x, track.y, track.error_covariance.tolist(), track.dx, track.dy, track.d_covariance.tolist()])
            # Clear the previous detection list
            track.clearLastFrame()

        return result

    def processDetectionFrame(self, sensor_id, timestamp, observations, cleanupTime, estimate_covariance):
        # We need to generate and add the detections from this detector
        detections_position_list = []
        detections_list = []
        for det in observations:
            # Create a rotated rectangle for IOU of 2 ellipses
            # [cx, cy, w, h, angle]
            if estimate_covariance and len(det) >= 4:
                # Calculate our 3 sigma std deviation to create a bounding box for matching
                try:
                    a, b, phi = shared_math.ellipsify(det[3], 3.0)
                    # Enforce a minimum size so matching doesn't fail
                    a += self.min_size
                    b += self.min_size
                    detections_position_list.append([det[1], det[2], self.min_size + a, self.min_size + b, phi])
                except Exception as e:
                    print ( " Failed! ", str(e))
                    # Use an arbitrary size if we have no covariance estimate
                    detections_position_list.append([det[1], det[2], self.min_size, self.min_size, math.radians(0)])
            else:
                #print(" Warning: no covaraince data for ", sensor_id)
                # Use an arbitrary size if we have no covariance estimate
                detections_position_list.append([det[1], det[2], self.min_size, self.min_size, math.radians(0)])
            detections_list.append([det[0], det[1], det[2], sensor_id])

        # Call the matching function to modify our detections in trackedList
        self.matchDetections(detections_position_list, detections_list, timestamp, cleanupTime, estimate_covariance)

    def matchDetections(self, detections_list_positions, detection_list, timestamp, cleanupTime, estimate_covariance):
        try:
            matches = []
            if len(detections_list_positions) > 0:
                if len(self.trackedList) > 0:
                    numpy_formatted = np.array(detections_list_positions).reshape(len(detections_list_positions), 5)
                    thisFrameTrackTree = BallTree(numpy_formatted, metric=shared_math.computeDistanceEuclidean)

                    # Need to check the tree size here in order to figure out if we can even do this
                    length = len(numpy_formatted)
                    if length > 0:
                        for trackedListIdx, track in enumerate(self.trackedList):
                            # The only difference between this and our other version is that
                            # the below line is commented out
                            # track.calcEstimatedPos(timestamp - self.prev_time)
                            tuple = thisFrameTrackTree.query(np.array(track.getPositionPredicted(timestamp, estimate_covariance)), k=length,
                                                            return_distance=True)
                            first = True
                            for IOUVsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                                if .99 >= IOUVsDetection >= 0:
                                    # Only grab the first match
                                    # Before determining if this is a match check if this detection has been matched already
                                    if first:
                                        try:
                                            index = [i[0] for i in matches].index(detectionIdx)
                                            # We have found the detection index, lets see which track is a better match
                                            if matches[index][2] > IOUVsDetection:
                                                # We are better so add ourselves
                                                matches.append([detectionIdx, trackedListIdx, IOUVsDetection])
                                                # Now unmatch the other one because we are better
                                                # This essentiall eliminates double matching
                                                matches[index][2] = 1
                                                matches[index][1] = -99
                                                # Now break the loop
                                                first = False
                                        except:
                                            # No matches in the list, go ahead and add
                                            matches.append([detectionIdx, trackedListIdx, IOUVsDetection])
                                            first = False
                                    else:
                                        # The other matches need to be marked so they arent made into a new track
                                        # Set distance to 1 so we know this wasn't the main match
                                        if detectionIdx not in [i[0] for i in matches]:
                                            # No matches in the list, go ahead and add
                                            matches.append([detectionIdx, -99, 1])

                    # update the tracks that made it through
                    for match in matches:
                        if match[1] != -99:
                            # Now append to the correct track
                            self.trackedList[match[1]].relations.append([match[0], match[2]])

                    # Old way
                    for track in self.trackedList:
                        if len(track.relations) == 1:
                            # Single match, go ahead and update the location
                            track.update(detection_list[track.relations[0][0]], timestamp)
                        elif len(track.relations) > 1:
                            # if we have multiple matches, pick the best one
                            max = 0
                            idx = -99
                            for rel in track.relations:
                                if rel[1] < max:
                                    max = rel[1]
                                    idx = rel[0]

                            if idx != -99:
                                track.update(detection_list[idx], timestamp)

                    if len(matches):
                        missing = sorted(set(range(0, len(detections_list_positions))) - set([i[0] for i in matches]))
                    else:
                        missing = list(range(0, len(detections_list_positions)))

                    added = []
                    for add in missing:
                        # Before we add anything, let's check back against the list to make sure there is no IOU match over .5 with this new item and another new item
                        tuple = thisFrameTrackTree.query((np.array([detections_list_positions[add]])), k=length,
                                                        return_distance=True)
                        # first = True
                        # for IOUsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                        #     # Check to make sure thie IOU match is high
                        #     if .75 >= IOUsDetection >= 0:
                        #         # Make sure this is not ourself
                        #         if add != detectionIdx:
                        #             # If this is not ourself, add ourself only if none of our matches has been added yet
                        #             if detectionIdx in added:
                        #                 first = False
                        #                 break
                        add_this = False
                        for IOUsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                            # Check to make sure thie IOU match is low with existing added
                            if .99 <= IOUsDetection:
                                # Make sure this is not ourself
                                if add != detectionIdx:
                                    # If this is not ourself, add ourself only if none of our matches has been added yet
                                    if detectionIdx not in added:
                                        add_this = True
                                        break

                        # We are the best according to arbitrarily broken tie and can be added
                        if add_this:
                            added.append(add)
                            new = Tracked(detection_list[add][0], detection_list[add][1], detection_list[add][2],
                                        detection_list[add][3], timestamp, self.current_tracked_id, self.fusion_mode)
                            if self.current_tracked_id < MAX_ID:
                                self.current_tracked_id += 1
                            else:
                                self.current_tracked_id = 0
                            self.trackedList.append(new)

                else:
                    for dl in detection_list:
                        new = Tracked(dl[0], dl[1], dl[2], dl[3], timestamp, self.current_tracked_id, self.fusion_mode)
                        if self.current_tracked_id < MAX_ID:
                            self.current_tracked_id += 1
                        else:
                            self.current_tracked_id = 0
                        self.trackedList.append(new)

            remove = []
            for idx, track in enumerate(self.trackedList):
                track.relations = []
                if track.lastTracked < ( timestamp - cleanupTime ):
                    remove.append(idx)

            for delete in reversed(remove):
                self.trackedList.pop(delete)
        except:
            return