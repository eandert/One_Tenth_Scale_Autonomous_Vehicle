import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
from shapely.geometry import Polygon
import bisect

from shared_library import shared_math


max_id = 10000


def binarySearch(a, x):
    'Locate the leftmost value exactly equal to x'
    i = bisect.bisect_left(a, x)
    if i != len(a) and a[i] == x:
        return i
    return -1


class MatchClass:
    def __init__(self, id, x, y, covariance, dx, dy, d_confidence, object_type, time):
        self.x = x
        self.y = y
        self.covariance = covariance
        self.dx = dx
        self.dy = dy
        self.velocity_confidence = d_confidence
        self.type = object_type
        self.last_tracked = time
        self.id = id


class ResizableKalman:
    def __init__(self, time, x, y, fusion_mode):
        # This list will take 
        self.localTrackersTimeAliveList = []
        self.localTrackersHList = []
        self.localTrackersMeasurementList = []
        self.localTrackersCovarainceList = []
        self.localTrackersIDList = []

        # Init the covariance to some value
        self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')
        self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype = 'float')

        # Store the first value for matching purposes
        self.x = x
        self.y = y

        # Store the covariance vs. expected
        self.error_tracker_temp = []

        # Arbitrary to start with, updated each iteration
        self.elapsed = 0.125

        # Arbitrary min tracking size so we are not too small to match to
        self.min_size = .5

        # Process varaition guess
        process_variation = .0625

        # Track the time of the last track
        self.lastTracked = time

        # Track the number of times this Kalman filter has been used
        self.idx = 0

        # The amount of time before a tracker is removed from the list
        self.time_until_removal = .5

        # Store the fusion mode
        self.fusion_mode = fusion_mode

        # Set up the Kalman filter
        # Initial State cov
        if self.fusion_mode == 0:
            # Set up the Kalman filter
            self.F_t_len = 4
            # Setup for x_hat = x + dx,  y_hat = y + dy
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
            # Control vector
            self.U_t = 0
        elif self.fusion_mode == 1:
            # Setup for x_hat = x + dx + dxdx,  y_hat = y + dy + dydy
            self.F_t_len = 6
            self.P_t = np.identity(6)
            self.P_t[2][2] = 0.0
            self.P_t[3][3] = 0.0
            self.P_t[4][4] = 0.0
            self.P_t[5][5] = 0.0
            self.P_hat_t = self.P_t
            # Process cov
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
        else:
            # model from https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
            self.F_t_len = 5
            self.P_t = np.identity(5)
            self.P_t[2][2] = 0.0
            self.P_t[3][3] = 0.0
            self.P_t[4][4] = 0.0
            self.P_hat_t = self.P_t
            # Process cov
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

    def addFrames(self, measurement_list):
        # try:
        # Rebuild the lists every time because measurements come and go
        self.localTrackersCovarainceList = []
        self.localTrackersMeasurementList = []
        self.localTrackersHList = []
        self.localTrackersIDList = []
        # Check if there are more sensors in the area that have not been added
        for match in measurement_list:
            # measurment_index = binarySearch(self.localTrackersIDList, match.id)
            # if measurment_index < 0:
            #     # This is not in the tracked list, needs to be added
            #     self.addTracker(match.id)
            self.localTrackersIDList.append(match.id)
            
            # All should be in the tracked list now, continue building the lists
            # Add the current covariance to the R matrix
            self.localTrackersCovarainceList.append(match.covariance)

            # Add the current measurments to the measurement matrix
            self.localTrackersMeasurementList.append(np.array([match.x, match.y]))#, match.dx, match.dy]))

            # The H matrix is different for radar (type 1), the rest are type 0
            self.localTrackersHList.append(0)

            # Mark this measurements as having been used recently
            self.localTrackersTimeAliveList.append(self.lastTracked)

        # except Exception as e:
        #     print ( " Exception: " + str(e) )

    def h_t(self, h_t_type):
        if h_t_type == 0:
            if self.fusion_mode == 0:
                return np.array([[1., 0., 0., 0.],
                                [0., 1., 0., 0.]], dtype = 'float')
            elif self.fusion_mode == 1:
                return np.array([[1., 0., 0., 0., 0., 0.],
                                [0., 1., 0., 0., 0., 0.]], dtype = 'float')
            elif self.fusion_mode == 2:
                return np.array([[1., 0., 0., 0., 0.],
                                [0., 1., 0., 0., 0.]], dtype = 'float')
            # if self.fusion_mode == 0:
            #     return np.array([[1, 0., 0., 0.],
            #                     [0., 1, 0., 0.],
            #                     [0., 0, 1., 0.],
            #                     [0., 0, 0., 1.]], dtype = 'float')
            # elif self.fusion_mode == 1:
            #     return np.array([[1, 0., 0., 0., 0., 0.],
            #                     [0., 1, 0., 0., 0., 0.],
            #                     [0., 0, 1., 0., 0., 0.],
            #                     [0., 0, 0., 1., 0., 0.]], dtype = 'float')
            # elif self.fusion_mode == 2:
            #     return np.array([[1, 0., 0., 0., 0.],
            #                     [0., 1, 0., 0., 0.],
            #                     [0, 0., 1., 0., 0.],
            #                     [0, 0., 0., 1., 0.]], dtype = 'float')
        else:
            # TODO: implement radar type
            return np.array([[0, 0., 0., 0.],
                            [0., 0, 0., 0.]], dtype = 'float')

    def averageMeasurementsFirstFrame(self):
        if len(self.localTrackersCovarainceList) == 1:
            return self.localTrackersMeasurementList[0], self.localTrackersCovarainceList[0]

        for idx, cov in enumerate(self.localTrackersCovarainceList):
            if idx == 0:
                temporary_c = cov.transpose()
            else:
                temporary_c = np.add(temporary_c, cov.transpose())
        temporary_c = temporary_c.transpose()

        for idx, (pos, cov) in enumerate(zip(self.localTrackersMeasurementList, self.localTrackersCovarainceList)):
            if idx == 0:
                temprorary_mu = np.matmul(cov.transpose(), pos)
            else:
                temprorary_mu = np.add(temprorary_mu, np.matmul(cov.transpose(), pos))
        temprorary_mu = np.matmul(temporary_c, temprorary_mu)

        return temprorary_mu, temporary_c

    def fusion(self, measurement_list, estimate_covariance, monitor):
        # Set the kalman variables and resize the arrays dynalically (if needed
        self.addFrames(measurement_list)
        # Do the kalman thing!
        if self.idx == 0:
            # We have no prior detection so we need to just output what we have but store for later
            # Do a Naive average to get the starting position
            pos, cov= self.averageMeasurementsFirstFrame()

            # Store so that next fusion is better
            self.prev_time = self.lastTracked
            self.x = pos[0]
            self.y = pos[1]
            self.error_covariance = cov
            self.idx += 1

            if self.fusion_mode == 0:
                # Store so that next fusion is better
                self.X_hat_t = np.array(
                    [[self.x], [self.y], [0], [0]], dtype = 'float')
            elif self.fusion_mode == 1:
                # setup for x, dx, dxdx
                self.X_hat_t = np.array(
                    [[self.x], [self.y], [0], [0], [0], [0]], dtype = 'float')
            else:
                # setup for https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
                self.X_hat_t = np.array(
                    [[self.x], [self.y], [0], [0], [0]], dtype = 'float')
            
            # Seed the covariance values directly from the measurement
            self.P_t[0][0] = self.error_covariance[0][0]
            self.P_t[0][1] = self.error_covariance[0][1]
            self.P_t[1][0] = self.error_covariance[1][0]
            self.P_t[1][1] = self.error_covariance[1][1]
            self.P_hat_t = self.P_t
            self.prev_time = self.lastTracked
            self.x = self.x
            self.y = self.y
            self.dx = 0.0
            self.dy = 0.0
            self.idx += 1
        else:
            try:
                # We have valid data
                # Transition matrix
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
                
                # print( " e ")
                # print ( self.X_hat_t )
                # print ( self.R_t )
                # print ( self.H_t )
                # print ( self.measure)

                self.X_hat_t, self.P_hat_t = shared_math.kalman_prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

                if len(self.localTrackersMeasurementList) == 0:
                    nothing_cov = np.array([[1.0, 0.],
                                            [0., 1.0]], dtype = 'float')
                    measure = np.array([.0, .0], dtype = 'float')
                    if self.fusion_mode == 0:
                        nothing_Ht = np.array([[0, 0., 0., 0.],
                                              [0., 0, 0., 0.]], dtype = 'float')
                    elif self.fusion_mode == 1:
                        nothing_Ht = np.array([[0, 0., 0., 0., 0., 0.],
                                              [0., 0., 0., 0., 0., 0.]], dtype = 'float')
                    elif self.fusion_mode == 2:
                        nothing_Ht = np.array([[0, 0., 0., 0., 0.],
                                              [0., 0, 0., 0., 0.]], dtype = 'float')

                    Z_t = (measure).transpose()
                    Z_t = Z_t.reshape(Z_t.shape[0], -1)
                    X_t, self.P_t = shared_math.kalman_update(self.X_hat_t, self.P_hat_t, Z_t, nothing_cov, nothing_Ht)
                    self.X_hat_t = X_t
                    self.P_hat_t = self.P_t
                else:
                    for mu, cov, h_t_type in zip(self.localTrackersMeasurementList, self.localTrackersCovarainceList, self.localTrackersHList):
                        Z_t = (mu).transpose()
                        Z_t = Z_t.reshape(Z_t.shape[0], -1)
                        X_t, self.P_t = shared_math.kalman_update(self.X_hat_t, self.P_hat_t, Z_t, cov, self.h_t(h_t_type))
                        self.X_hat_t = X_t
                        self.P_hat_t = self.P_t

                # Lets check the accuracy of each sensing platform
                self.error_tracker_temp = []
                if monitor:
                    for id, mu, cov, h_t_type in zip(self.localTrackersIDList, self.localTrackersMeasurementList, self.localTrackersCovarainceList, self.localTrackersHList):
                        Z_t = (mu).transpose()
                        Z_t = Z_t.reshape(Z_t.shape[0], -1)
                        y_t_temp = Z_t - self.h_t(h_t_type).dot(self.X_hat_t)
                        #print(y_t_temp)
                        location_error = math.hypot(y_t_temp[0], y_t_temp[1])
                        expected_a, expected_b, expected_angle = shared_math.ellipsify(cov, 1.0)
                        expected_x = shared_math.calculateRadiusAtAngle(expected_a, expected_b, expected_angle, math.radians(0))
                        expected_y = shared_math.calculateRadiusAtAngle(expected_a, expected_b, expected_angle, math.radians(90))
                        expected_location_error = math.hypot(expected_x**2, expected_y**2)
                        #cov
                        #location_error_std = location_error / expected_location_error
                        #print(location_error, expected_location_error, location_error_std)
                        self.error_tracker_temp.append([id, location_error, expected_location_error])
                        #print(" error: ", id, location_error, location_error_std)

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
                if self.P_t[0][0] != 0.0 or self.P_t[0][1] != 0.0:
                    self.error_covariance = np.array([[self.P_t[0][0], self.P_t[0][1]], [self.P_t[1][0], self.P_t[1][1]]], dtype = 'float')
                    self.d_covariance = np.array([[self.P_t[2][2], self.P_t[2][3]], [self.P_t[3][2], self.P_t[3][3]]], dtype = 'float')
                else:
                    #print ( " what the heck: ", self.P_t)
                    self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')
                    self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype = 'float')

                #print ( elapsed, self.x, self.y, self.dx, self.dy, math.degrees(math.hypot(self.dx, self.dy)))
                # Post fusion, lets clear old trackers now
                #self.removeOldTrackers()

            except Exception as e:
                print ( " Exception: " + str(e) )

    def getKalmanPred(self, time):
        # Prediction based mathcing methods seems to be making this fail so we are using no prediction :/
        # Enforce a min size of a vehicle so that a detection has some area overlap to check
        a, b, phi = shared_math.ellipsify(self.error_covariance, 3.0)
        return self.x, self.y, self.min_size + a, self.min_size + b, phi


class GlobalTracked:
    # This object tracks a single object that has been detected in a video frame.
    # We use this primarily to match objects seen between frames and included in here
    # is a function for kalman filter to smooth the x and y values as well as a
    # function for prediction where the next bounding box will be based on prior movement.
    def __init__(self, sensor_id, x, y, covariance, dx, dy, dcovariance, time, track_id, fusion_mode):
        self.x = x
        self.y = y
        self.dx = 0
        self.dy = 0
        self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')
        #self.typeArray = [0, 0, 0, 0]
        #self.typeArray[object_type] += 1
        #self.type = self.typeArray.index(max(self.typeArray))
        self.lastTracked = time
        self.id = track_id
        self.idx = 0
        self.min_size = 0.5
        self.track_count = 0
        self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype = 'float')
        self.match_list = []
        self.fusion_steps = 0
        self.error_monitor = []

        # Add this first match
        new_match = MatchClass(sensor_id, x, y, covariance, dx, dy, dcovariance, 0, time)
        self.match_list.append(new_match)

        # Kalman stuff
        self.fusion_mode = fusion_mode
        self.kalman = ResizableKalman(time, x, y, fusion_mode)

    # Update adds another detection to this track
    def update(self, other, time):
        new_match = MatchClass(other[0], other[1], other[2], other[3], other[4], other[5], other[6], 0, time)
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
            x, y, a, b, phi = self.kalman.getKalmanPred(timestamp)
            return [
                [x, y, a, b, phi]
            ]

    def fusion(self, estimate_covariance, monitor):
        self.kalman.fusion(self.match_list, estimate_covariance, monitor)
        self.x = self.kalman.x
        self.y = self.kalman.y
        self.error_covariance = self.kalman.error_covariance
        self.dx = self.kalman.dx
        self.dy = self.kalman.dy
        self.d_covariance = self.kalman.d_covariance
        self.error_monitor = self.kalman.error_tracker_temp
        self.fusion_steps += 1

    def clearLastFrame(self):
        self.match_list = []


class GlobalFUSION:
    # Fusion is a special class for matching and fusing detections for a variety of sources.
    # The inpus is scalable and therefore must be generated before being fed into this class.
    # A unique list of detections is required from each individual sensor or pre-fused device
    # output or it will not be matched. Detections too close to each other may be combined.
    # This is a modified version of the frame-by-frame tracker seen in:
    # https://github.com/eandert/Jetson_Nano_Camera_Vehicle_Tracker
    def __init__(self, fusion_mode):
        # Set other parameters for the class
        self.trackedList = []
        self.id = 0
        self.prev_time = -99.0
        self.min_size = 0.5
        self.trackShowThreshold = 4
        self.fusion_mode = fusion_mode

        # Indicate our success
        print('Started FUSION successfully...')

    def fuseDetectionFrame(self, estimate_covariance, monitor):
        # Time to go through each track list and fuse!
        result = []
        cooperative_monitoring = []
        for track in self.trackedList:
            track.fusion(estimate_covariance, monitor)
            if track.fusion_steps >= self.trackShowThreshold:
                result.append([track.id, track.x, track.y, track.error_covariance.tolist(), track.dx, track.dy, track.d_covariance.tolist()])
                if track.error_monitor:
                    cooperative_monitoring.append(track.error_monitor)
                else:
                    cooperative_monitoring.append([])
            # Clear the previous detection list
            track.clearLastFrame()
            # Clean up the tracks for next time
            self.cleanDetections(estimate_covariance)

        return result, cooperative_monitoring

    def processDetectionFrame(self, timestamp, observations, cleanupTime, estimate_covariance):
        # We need to generate and add the detections from this detector
        detections_position_list = []
        detections_list = []
        for det in observations:
            # #TODO: Figure out why performance is worse with this method
            # # Create a rotated rectangle for IOU of 2 ellipses
            # # [cx, cy, w, h, angle]
            # if estimateCovariance and len(det) >= 3:
            #     # Calculate our 3 sigma std deviation to create a bounding box for matching
            #     try:
            #         a, b, phi = shared_math.ellipsify(np.array(det[3]), 3.0)
            #         # Enforce a minimum size so matching doesn't fail
            #         a += self.min_size
            #         b += self.min_size
            #         detections_position_list.append([det[1], det[2], a, b, phi])
            #     except Exception as e:
            #         print ( " Failed! ", str(e))
            #         # Use an arbitrary size if we have no covariance estimate
            #         detections_position_list.append([det[1], det[2], self.min_size, self.min_size, math.radians(0)])
            # else:
            #     # Use an arbitrary size if we have no covariance estimate
            detections_position_list.append([det[1], det[2], self.min_size, self.min_size, math.radians(0)])
            #detections_position_list.append([det[0], det[1], self.min_size, self.min_size, math.radians(0)])
            detections_list.append([det[0], det[1], det[2], np.array(det[3]), det[4], det[5], np.array(det[6])])

        # Call the matching function to modify our detections in trackedList
        self.matchDetections(detections_position_list, detections_list, timestamp, cleanupTime, estimate_covariance)

    def matchDetections(self, detections_list_positions, detection_list, timestamp, cleanupTime, estimate_covariance):
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
                    add_this = False
                    # for IOUsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                    #     # Check to make sure thie IOU match is high
                    #     if .75 >= IOUsDetection >= 0:
                    #         # Make sure this is not ourself
                    #         if add != detectionIdx:
                    #             # If this is not ourself, add ourself only if none of our matches has been added yet
                    #             if detectionIdx in added:
                    #                 first = False
                    #                 break
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
                        new = GlobalTracked(detection_list[add][0], detection_list[add][1], detection_list[add][2],
                                      detection_list[add][3], detection_list[add][4], detection_list[add][5],
                                      detection_list[add][6], timestamp, self.id, self.fusion_mode)
                        if self.id < max_id:
                            self.id += 1
                        else:
                            self.id = 0
                        self.trackedList.append(new)

            else:
                for dl in detection_list:
                    new = GlobalTracked(dl[0], dl[1], dl[2], dl[3], dl[4], dl[5], dl[6], timestamp, self.id, self.fusion_mode)
                    if self.id < max_id:
                        self.id += 1
                    else:
                        self.id = 0
                    self.trackedList.append(new)

        remove = []
        for idx, track in enumerate(self.trackedList):
            track.relations = []
            if track.lastTracked < ( timestamp - cleanupTime ):
                remove.append(idx)

        for delete in reversed(remove):
            self.trackedList.pop(delete)

    def cleanDetections(self, estimate_covariance):
        detections_position_list = []
        detections_position_list_id = []
        for track in self.trackedList:
            a, b, phi = shared_math.ellipsify(track.error_covariance, 3.0)
            detections_position_list.append([track.x, track.y, self.min_size + a, self.min_size + b, phi])
            detections_position_list_id.append(track.id)
        matches = []
        remove = []
        if len(detections_position_list) > 0:
            if len(self.trackedList) > 0:
                numpy_formatted = np.array(detections_position_list).reshape(len(detections_position_list), 5)
                thisFrameTrackTree = BallTree(numpy_formatted, metric=shared_math.computeDistanceEllipseBox)

                # Need to check the tree size here in order to figure out if we can even do this
                length = len(numpy_formatted)
                if length > 0:
                    for trackedListIdx, track in enumerate(self.trackedList):
                        # The only difference between this and our other version is that
                        # the below line is commented out
                        # track.calcEstimatedPos(timestamp - self.prev_time)
                        tuple = thisFrameTrackTree.query(np.array(track.getPositionPredicted(self.prev_time, estimate_covariance)), k=length,
                                                         return_distance=True)
                        first = True
                        for IOUVsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                            # 100% match is ourself! Look for IOU > .50 for now to delete
                            if .50 >= IOUVsDetection > 0.001:
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
                        if match[0] != match[1]:
                            if match[1] not in remove and match[0] not in remove:
                                # Check which track is older and keep that one
                                check0 = self.trackedList[match[0]].lastTracked
                                check1 = self.trackedList[match[1]].lastTracked
                                # Arbitrary tie break towards earlier in the list
                                if check0 > check1:
                                    if self.trackedList[match[0]].fusion_steps >= self.trackShowThreshold:
                                        bisect.insort(remove, match[1])
                                elif check0 < check1:
                                    if self.trackedList[match[1]].fusion_steps >= self.trackShowThreshold:
                                        bisect.insort(remove, match[0])
                                else:
                                    check0 = self.trackedList[match[0]].fusion_steps
                                    check1 = self.trackedList[match[1]].fusion_steps  
                                    if check0 >= check1:
                                        if check0 >= self.trackShowThreshold:
                                            bisect.insort(remove, match[1])
                                    else:
                                        if check1 >= self.trackShowThreshold:
                                            bisect.insort(remove, match[0])
                      
        for delete in reversed(remove):
            print( "Cleaning track ", delete)
            self.trackedList.pop(delete)
        #print(len(self.trackedList), len(remove), remove)