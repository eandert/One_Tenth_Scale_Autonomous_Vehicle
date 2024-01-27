import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
from shapely.geometry import Polygon
import bisect

from shared_library import shared_math


max_id = 10000


class MatchClass:
    def __init__(self, id, x, y, covariance, dx, dy, d_confidence, confidence, trust_score, object_type, time):
        self.x = x
        self.y = y
        self.covariance = covariance
        self.dx = dx
        self.dy = dy
        self.velocity_confidence = d_confidence
        self.type = object_type
        self.last_tracked = time
        self.id = id
        self.confidence = confidence
        self.trust_score = trust_score


class ResizableKalman:
    def __init__(self, time, x, y, fusion_mode):
        # This list will take
        self.localTrackersHList = []
        self.localTrackersMeasurementList = []
        self.localTrackersCovarianceList = []
        self.localTrackersIDList = []

        # Init the covariance to some value
        self.error_covariance = np.array(
            [[1.0, 0.0], [0.0, 1.0]], dtype='float')
        self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype='float')

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
        process_variation = .16

        # Track the time of the last track
        self.last_tracked = time

        # Track the number of times this Kalman filter has been used
        self.idx = 0

        # The amount of time before a tracker is removed from the list
        self.time_until_removal = .5

        # Store the fusion mode
        self.fusion_mode = fusion_mode

        # Trupercept stuff
        self.trupercept_list = []

        # Set up the Kalman filter
        # Initial State cov
        if self.fusion_mode == 0:
            # Set up the Kalman filter
            self.F_t_len = 4
            # Setup for x_hat = x + dx,  y_hat = y + dy
            # Initial State cov
            self.P_hat_t = np.identity(4)
            self.P_hat_t[2][2] = 0.0
            self.P_hat_t[3][3] = 0.0
            # Process cov
            four = process_variation * (.125*.125*.125*.125)/4.0
            three = process_variation * (.125*.125*.125)/3.0
            two = process_variation * (.125*.125)/2.0
            one = process_variation * (.125)
            self.Q_t = np.array([[three, 0, two, 0],
                                [0, three, 0, two],
                                [two, 0, process_variation, 0],
                                [0, two, 0, process_variation]], dtype='float')
            # Control matrix
            self.B_t = np.array([[0], [0], [0], [0]], dtype='float')
            # Control vector
            self.U_t = 0
        elif self.fusion_mode == 1:
            # Setup for x_hat = x + dx + dxdx,  y_hat = y + dy + dydy
            self.F_t_len = 6
            self.P_hat_t = np.identity(6)
            self.P_hat_t[2][2] = 0.0
            self.P_hat_t[3][3] = 0.0
            self.P_hat_t[4][4] = 0.0
            self.P_hat_t[5][5] = 0.0
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
                                [0, 0, 0, three, 0, two]], dtype='float')
            # Control matrix
            self.B_t = np.array([[0], [0], [0], [0], [0], [0]], dtype='float')
            # Control vector
            self.U_t = 0
        else:
            # model from https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
            self.F_t_len = 5
            self.P_hat_t = np.identity(5)
            self.P_hat_t[2][2] = 0.0
            self.P_hat_t[3][3] = 0.0
            self.P_hat_t[4][4] = 0.0
            # Process cov
            four = process_variation * (.125*.125*.125*.125)/4.0
            three = process_variation * (.125*.125*.125)/2.0
            angle_variation = process_variation
            two = angle_variation * (.125*.125)
            self.Q_t = np.array([[four, 0, three, 0, 0],
                                [0, four, 0, three, 0],
                                [three, three, two, 0, 0],
                                [0, 0, 0, two, 0],
                                [0, 0, 0, 0, two]], dtype='float')
            # Control matrix
            self.B_t = np.array([[0], [0], [0], [0], [0]], dtype='float')
            # Control vector
            self.U_t = 0

    def addFrames(self, measurement_list):
        # try:
        # Rebuild the lists every time because measurements come and go
        self.localTrackersCovarianceList = []
        self.localTrackersMeasurementList = []
        self.localTrackersHList = []
        self.localTrackersIDList = []
        self.localTrackersExtraList = []
        # Check if there are more sensors in the area that have not been added
        for match in measurement_list:
            self.localTrackersIDList.append(match.id)
            self.localTrackersCovarianceList.append(match.covariance)
            # , match.dx, match.dy]))
            self.localTrackersMeasurementList.append(
                np.array([match.x, match.y]))
            self.localTrackersHList.append(0)
            self.localTrackersExtraList.append(
                [match.confidence, match.trust_score])

    def h_t(self, h_t_type):
        if h_t_type == 0:
            if self.fusion_mode == 0:
                return np.array([[1., 0., 0., 0.],
                                [0., 1., 0., 0.]], dtype='float')
            elif self.fusion_mode == 1:
                return np.array([[1., 0., 0., 0., 0., 0.],
                                [0., 1., 0., 0., 0., 0.]], dtype='float')
            elif self.fusion_mode == 2:
                return np.array([[1., 0., 0., 0., 0.],
                                [0., 1., 0., 0., 0.]], dtype='float')
        else:
            # TODO: implement radar type
            return np.array([[0, 0., 0., 0.],
                            [0., 0, 0., 0.]], dtype='float')

    def averageMeasurementsFirstFrame(self):
        if len(self.localTrackersCovarianceList) == 1:
            return self.localTrackersMeasurementList[0], self.localTrackersCovarianceList[0]

        for idx, cov in enumerate(self.localTrackersCovarianceList):
            if idx == 0:
                temporary_c = cov.transpose()
            else:
                temporary_c = np.add(temporary_c, cov.transpose())
        temporary_c = temporary_c.transpose()

        for idx, (pos, cov) in enumerate(zip(self.localTrackersMeasurementList, self.localTrackersCovarianceList)):
            if idx == 0:
                temporary_mu = np.matmul(cov.transpose(), pos)
            else:
                temporary_mu = np.add(
                    temporary_mu, np.matmul(cov.transpose(), pos))
        temporary_mu = np.matmul(temporary_c, temporary_mu)

        return temporary_mu, temporary_c

    def fusion(self, measurement_list, estimate_covariance, monitor):
        # Set the kalman variables and resize the arrays dynalically (if needed
        self.addFrames(measurement_list)
        # Do the kalman thing!
        if self.idx == 0:
            # We have no prior detection so we need to just output what we have but store for later
            # Do a Naive average to get the starting position
            pos, cov = self.averageMeasurementsFirstFrame()

            # Store so that next fusion is better
            self.prev_time = self.last_tracked
            self.x = pos[0]
            self.y = pos[1]
            self.error_covariance = cov
            self.idx += 1

            if self.fusion_mode == 0:
                # Store so that next fusion is better
                self.X_hat_t = np.array(
                    [[self.x], [self.y], [0], [0]], dtype='float')
            elif self.fusion_mode == 1:
                # setup for x, dx, dxdx
                self.X_hat_t = np.array(
                    [[self.x], [self.y], [0], [0], [0], [0]], dtype='float')
            else:
                # setup for https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
                self.X_hat_t = np.array(
                    [[self.x], [self.y], [0], [0], [0]], dtype='float')

            # Seed the covariance values directly from the measurement
            self.P_hat_t[0][0] = self.error_covariance[0][0]
            self.P_hat_t[0][1] = self.error_covariance[0][1]
            self.P_hat_t[1][0] = self.error_covariance[1][0]
            self.P_hat_t[1][1] = self.error_covariance[1][1]
            self.prev_time = self.last_tracked
            self.x = self.x
            self.y = self.y
            self.dx = 0.0
            self.dy = 0.0
            self.idx += 1
        else:
            # try:
            # We have valid data
            # Transition matrix
            elapsed = self.last_tracked - self.prev_time
            if elapsed <= 0.0:
                # print( "Error time elapsed is incorrect! " + str(elapsed) )
                # Set to arbitrary time
                elapsed = 0.125

            if self.fusion_mode == 0:
                self.F_t = np.array([[1, 0, elapsed, 0],
                                    [0, 1, 0, elapsed],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], dtype='float')

            elif self.fusion_mode == 1:
                # setup for x, dx, dxdx
                self.F_t = np.array([[1, 0, elapsed, 0, elapsed*elapsed, 0],
                                    [0, 1, 0, elapsed, 0, elapsed*elapsed],
                                    [0, 0, 1, 0, elapsed, 0],
                                    [0, 0, 0, 1, 0, elapsed],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]], dtype='float')

            else:
                # setup for https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
                v_1 = self.X_hat_t[2]
                phi_1 = self.X_hat_t[3]
                phi_dot_1 = self.X_hat_t[4]
                # print ( self.X_hat_t, v_1, phi_1, phi_dot_1 )
                if phi_1 == 0.0:
                    # Not moving, No correlation
                    v_x = 999.9
                    v_y = 999.9
                    phi_x = 999.9
                    phi_y = 999.9
                else:
                    v_x = (1.0 / phi_dot_1) * (-math.sin(phi_1) +
                                               math.sin(phi_1 + elapsed * phi_dot_1))
                    v_y = (1.0 / phi_dot_1) * (math.cos(phi_1) -
                                               math.cos(phi_1 + elapsed * phi_dot_1))
                    phi_x = (v_1 / phi_dot_1) * (-math.cos(phi_1) +
                                                 math.cos(phi_1 + elapsed * phi_dot_1))
                    phi_y = (v_1 / phi_dot_1) * (-math.sin(phi_1) +
                                                 math.sin(phi_1 + elapsed * phi_dot_1))
                if phi_dot_1 == 0.0:
                    # Not accelerating, NO correlation
                    phi_dot_x = 999.9
                    phi_dot_y = 999.9
                else:
                    phi_dot_x = (v_1 * elapsed / phi_dot_1) * math.cos(phi_1 + elapsed * phi_dot_1) - (
                        v_1 / phi_dot_1**2) * (- math.sin(phi_1) + math.sin(phi_1 + elapsed * phi_dot_1))
                    phi_dot_y = (v_1 * elapsed / phi_dot_1) * math.sin(phi_1 + elapsed * phi_dot_1) - (
                        v_1 / phi_dot_1**2) * (math.cos(phi_1) - math.cos(phi_1 + elapsed * phi_dot_1))
                self.F_t = np.array([[1, 0, v_x, phi_x, phi_dot_x],
                                    [0, 1, v_y, phi_y, phi_dot_y],
                                    [0, 0, 1, 0, 0],
                                    [0, 0, 0, 1, elapsed],
                                    [0, 0, 0, 0, 1]], dtype='float')

            self.X_hat_t, self.P_hat_t = shared_math.kalman_prediction(
                self.X_hat_t, self.P_hat_t, self.F_t, self.B_t, self.U_t, self.Q_t)

            added = 0
            if len(self.localTrackersMeasurementList) != 0:
                for mu, cov, h_t_type, extra in zip(self.localTrackersMeasurementList, self.localTrackersCovarianceList, self.localTrackersHList, self.localTrackersExtraList):
                    if extra[1] < 1.2:
                        Z_t = mu.transpose()
                        Z_t = Z_t.reshape(Z_t.shape[0], -1)
                        self.X_hat_t, self.P_hat_t = shared_math.kalman_update(
                            self.X_hat_t, self.P_hat_t, Z_t, cov.dot(extra[1]), self.h_t(h_t_type))
                        added += 1
            if len(self.localTrackersMeasurementList) == 0 or added == 0:
                nothing_cov = np.array([[1.0, 0.],
                                        [0., 1.0]], dtype='float')
                measure = np.array([.0, .0], dtype='float')
                if self.fusion_mode == 0:
                    nothing_Ht = np.array([[0, 0., 0., 0.],
                                           [0., 0, 0., 0.]], dtype='float')
                elif self.fusion_mode == 1:
                    nothing_Ht = np.array([[0, 0., 0., 0., 0., 0.],
                                           [0., 0., 0., 0., 0., 0.]], dtype='float')
                elif self.fusion_mode == 2:
                    nothing_Ht = np.array([[0, 0., 0., 0., 0.],
                                           [0., 0, 0., 0., 0.]], dtype='float')

                Z_t = (measure).transpose()
                Z_t = Z_t.reshape(Z_t.shape[0], -1)
                self.X_hat_t, self.P_hat_t = shared_math.kalman_update(
                    self.X_hat_t, self.P_hat_t, Z_t, nothing_cov, nothing_Ht)

            # Lets check the accuracy of each sensing platform
            self.error_tracker_temp = []
            length = len(self.localTrackersIDList)
            if monitor:
                # Our method
                if self.P_hat_t[0][0] != 0.0 and self.P_hat_t[0][1] != 0.0:
                    global_error_x, global_error_y, global_error_angle = shared_math.ellipsify(
                        [[self.P_hat_t[0][0], self.P_hat_t[0][1]], [self.P_hat_t[1][0], self.P_hat_t[1][1]]], 1.0)
                else:
                    global_error_x = 0.0
                    global_error_y = 0.0
                    global_error_angle = 0.0
                for id, mu, cov, h_t_type in zip(self.localTrackersIDList, self.localTrackersMeasurementList, self.localTrackersCovarianceList, self.localTrackersHList):
                    Z_t = (mu).transpose()
                    Z_t = Z_t.reshape(Z_t.shape[0], -1)
                    y_t_temp = Z_t - self.h_t(h_t_type).dot(self.X_hat_t)
                    # print(y_t_temp)
                    location_error = math.hypot(y_t_temp[0], y_t_temp[1])
                    expected_a, expected_b, expected_angle = shared_math.ellipsify(
                        cov, 1.0)
                    expected_x = shared_math.calculateRadiusAtAngle(
                        expected_a, expected_b, expected_angle, math.radians(0))
                    expected_y = shared_math.calculateRadiusAtAngle(
                        expected_a, expected_b, expected_angle, math.radians(90))
                    expected_location_error = math.hypot(
                        expected_x, expected_y) - math.hypot(global_error_x, global_error_y)
                    # cov
                    location_error_std = location_error / expected_location_error
                    # print(location_error, expected_location_error, location_error_std)
                    self.error_tracker_temp.append(
                        [id, location_error_std, length])
                    # print(" error: ", id, location_error, expected_location_error, location_error_std)

                # TruPercept
                trupercept_list = []
                # print(self.localTrackersIDList, self.localTrackersMeasurementList, self.localTrackersExtraList)
                for id_test, mu_test, confidence_test in zip(self.localTrackersIDList, self.localTrackersMeasurementList, self.localTrackersExtraList):
                    # print(self.localTrackersIDList, self.localTrackersMeasurementList, self.localTrackersExtraList)
                    # trupercept_sub_list = []
                    # for id, mu, confidence in zip(self.localTrackersIDList, self.localTrackersMeasurementList, self.localTrackersExtraList):
                    #     if id_test != id:
                    # iou = 1 - shared_math.computeDistanceEllipseBox([mu_test[0], mu_test[1], self.min_size, self.min_size, 0], [mu[0], mu[1], self.min_size, self.min_size, 0])
                    trupercept_list.append([id_test, confidence_test[0]])
                    # trupercept_list.append(trupercept_sub_list)
                self.trupercept_list = trupercept_list

            self.prev_time = self.last_tracked
            self.x = self.X_hat_t[0][0]
            self.y = self.X_hat_t[1][0]
            if self.fusion_mode == 2:
                # Different setup for fusion mode 2 need to calculate vector from angle and velocity
                self.dx = self.X_hat_t[2][0] * math.cos(self.X_hat_t[3][0])
                self.dy = self.X_hat_t[2][0] * math.sin(self.X_hat_t[3][0])
            else:
                self.dx = self.X_hat_t[2][0]
                self.dy = self.X_hat_t[3][0]
            self.idx += 1
            if self.P_hat_t[0][0] != 0.0 or self.P_hat_t[0][1] != 0.0:
                self.error_covariance = np.array([[self.P_hat_t[0][0], self.P_hat_t[0][1]], [
                                                 self.P_hat_t[1][0], self.P_hat_t[1][1]]], dtype='float')
                self.d_covariance = np.array([[self.P_hat_t[2][2], self.P_hat_t[2][3]], [
                                             self.P_hat_t[3][2], self.P_hat_t[3][3]]], dtype='float')
            else:
                self.error_covariance = np.array(
                    [[1.0, 0.0], [0.0, 1.0]], dtype='float')
                self.d_covariance = np.array(
                    [[2.0, 0.0], [0.0, 2.0]], dtype='float')

            # except Exception as e:
            #     print ( " Exception: " + str(e) )

    def getKalmanPred(self, time):
        # Prediction based mathcing methods seems to be making this fail so we are using no prediction :/
        # Enforce a min size of a vehicle so that a detection has some area overlap to check
        a, b, phi = shared_math.ellipsify(self.error_covariance, 1.0)
        return self.x, self.y, self.min_size + a, self.min_size + b, phi


class GlobalTracked:
    # This object tracks a single object that has been detected in a video frame.
    # We use this primarily to match objects seen between frames and included in here
    # is a function for kalman filter to smooth the x and y values as well as a
    # function for prediction where the next bounding box will be based on prior movement.
    def __init__(self, sensor_id, x, y, covariance, dx, dy, dcovariance, confidence, trust_score, time, track_id, fusion_mode):
        self.x = x
        self.y = y
        self.dx = 0
        self.dy = 0
        self.error_covariance = np.array(
            [[1.0, 0.0], [0.0, 1.0]], dtype='float')
        # self.typeArray = [0, 0, 0, 0]
        # self.typeArray[object_type] += 1
        # self.type = self.typeArray.index(max(self.typeArray))
        self.last_tracked = time
        self.id = track_id
        self.idx = 0
        self.min_size = 0.5
        self.track_count = 0
        self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype='float')
        self.match_list = []
        self.fusion_steps = 0
        self.error_monitor = []
        self.num_trackers = 0

        # Trupercept stuff
        self.trupercept_list = []

        # Add this first match
        new_match = MatchClass(sensor_id, x, y, covariance, dx,
                               dy, dcovariance, confidence, trust_score, 0, time)
        self.match_list.append(new_match)

        # Kalman stuff
        self.fusion_mode = fusion_mode
        self.kalman = ResizableKalman(time, x, y, fusion_mode)

    # Update adds another detection to this track
    def update(self, other, time):
        new_match = MatchClass(other[0], other[1], other[2], other[3],
                               other[4], other[5], other[6], other[7], other[8], 0, time)
        self.match_list.append(new_match)

        self.last_tracked = time

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
        self.num_trackers = len(self.kalman.localTrackersIDList)
        self.fusion_steps += 1
        self.trupercept_list = self.kalman.trupercept_list

    def clearLastFrame(self):
        self.match_list = []


class GlobalFUSION:
    # Fusion is a special class for matching and fusing detections for a variety of sources.
    # The input is scalable and therefore must be generated before being fed into this class.
    # A unique list of detections is required from each individual sensor or pre-fused device
    # output or it will not be matched. Detections too close to each other may be combined.
    # This is a modified version of the frame-by-frame tracker seen in:
    # https://github.com/eandert/Jetson_Nano_Camera_Vehicle_Tracker
    def __init__(self, fusion_mode):
        # Set other parameters for the class
        self.tracked_list = []
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
        trupercept_monitoring = []
        for track in self.tracked_list:
            track.fusion(estimate_covariance, monitor)
            if track.fusion_steps >= self.trackShowThreshold:
                result.append([track.id, track.x, track.y, track.error_covariance.tolist(
                ), track.dx, track.dy, track.d_covariance.tolist(), track.num_trackers])
                if track.error_monitor:
                    cooperative_monitoring.append(track.error_monitor)
                    trupercept_monitoring.append(track.trupercept_list)
            # Clear the previous detection list
            track.clearLastFrame()

        if len(self.tracked_list) >= 2:
            # Clean up the tracks for next time
            self.cleanDetections(estimate_covariance)

        return result, cooperative_monitoring, trupercept_monitoring

    def processDetectionFrame(self, timestamp, observations, cleanupTime, estimate_covariance, trust_score=1.0):
        # We need to generate and add the detections from this detector
        detections_list_positions = []
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
            detections_list_positions.append(
                [det[1], det[2], self.min_size, self.min_size, math.radians(0)])
            # detections_position_list.append([det[0], det[1], self.min_size, self.min_size, math.radians(0)])
            detections_list.append([det[0], det[1], det[2], np.array(
                det[3]), det[4], det[5], np.array(det[6]), det[7], trust_score])

        # Call the matching function to modify our detections in tracked_list
        self.matchDetections(detections_list_positions, detections_list,
                             timestamp, cleanupTime, estimate_covariance)

    def matchDetections(self, detections_list_positions, detection_list, timestamp, cleanupTime, estimate_covariance):
        matches = []
        if len(detections_list_positions) > 0:
            if len(self.tracked_list) > 0:
                numpy_formatted = np.array(detections_list_positions).reshape(
                    len(detections_list_positions), 5)
                thisFrameTrackTree = BallTree(
                    numpy_formatted, metric=shared_math.computeDistanceEuclidean)

                # Need to check the tree size here in order to figure out if we can even do this
                length = len(numpy_formatted)
                if length > 0:
                    for tracked_listIdx, track in enumerate(self.tracked_list):
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
                                        index = [i[0] for i in matches].index(
                                            detectionIdx)
                                        # We have found the detection index, lets see which track is a better match
                                        if matches[index][2] > IOUVsDetection:
                                            # We are better so add ourselves
                                            matches.append(
                                                [detectionIdx, tracked_listIdx, IOUVsDetection])
                                            # Now unmatch the other one because we are better
                                            # This essentiall eliminates double matching
                                            matches[index][2] = 1
                                            matches[index][1] = -99
                                            # Now break the loop
                                            first = False
                                    except:
                                        # No matches in the list, go ahead and add
                                        matches.append(
                                            [detectionIdx, tracked_listIdx, IOUVsDetection])
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
                        self.tracked_list[match[1]].relations.append(
                            [match[0], match[2]])

                # Old way
                for track in self.tracked_list:
                    if len(track.relations) == 1:
                        # Single match, go ahead and update the location
                        track.update(
                            detection_list[track.relations[0][0]], timestamp)
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
                    missing = sorted(
                        set(range(0, len(detections_list_positions))) - set([i[0] for i in matches]))
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
                                            detection_list[add][6], detection_list[add][7], detection_list[add][8],
                                            timestamp, self.id, self.fusion_mode)
                        if self.id < max_id:
                            self.id += 1
                        else:
                            self.id = 0
                        self.tracked_list.append(new)

            else:
                for dl in detection_list:
                    new = GlobalTracked(dl[0], dl[1], dl[2], dl[3], dl[4], dl[5],
                                        dl[6], dl[7], dl[8], timestamp, self.id, self.fusion_mode)
                    if self.id < max_id:
                        self.id += 1
                    else:
                        self.id = 0
                    self.tracked_list.append(new)

        remove = []
        for idx, track in enumerate(self.tracked_list):
            track.relations = []
            if track.last_tracked < (timestamp - cleanupTime):
                remove.append(idx)

        for delete in reversed(remove):
            self.tracked_list.pop(delete)

    def cleanDetections(self, estimate_covariance):
        detections_position_list = []
        detections_position_list_id = []
        for track in self.tracked_list:
            a, b, phi = shared_math.ellipsify(track.error_covariance, 3.0)
            detections_position_list.append(
                [track.x, track.y, self.min_size + a, self.min_size + b, phi])
            detections_position_list_id.append(track.id)
        matches = []
        remove = []
        if len(detections_position_list) > 0:
            if len(self.tracked_list) > 0:
                numpy_formatted = np.array(detections_position_list).reshape(
                    len(detections_position_list), 5)
                thisFrameTrackTree = BallTree(
                    numpy_formatted, metric=shared_math.computeDistanceEllipseBox)

                # Need to check the tree size here in order to figure out if we can even do this
                length = len(numpy_formatted)
                if length > 0:
                    for tracked_listIdx, track in enumerate(self.tracked_list):
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
                                        index = [i[0] for i in matches].index(
                                            detectionIdx)
                                        # We have found the detection index, lets see which track is a better match
                                        if matches[index][2] > IOUVsDetection:
                                            # We are better so add ourselves
                                            matches.append(
                                                [detectionIdx, tracked_listIdx, IOUVsDetection])
                                            # Now unmatch the other one because we are better
                                            # This essentiall eliminates double matching
                                            matches[index][2] = 1
                                            matches[index][1] = -99
                                            # Now break the loop
                                            first = False
                                    except:
                                        # No matches in the list, go ahead and add
                                        matches.append(
                                            [detectionIdx, tracked_listIdx, IOUVsDetection])
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
                                check0 = self.tracked_list[match[0]
                                                           ].last_tracked
                                check1 = self.tracked_list[match[1]
                                                           ].last_tracked
                                # Arbitrary tie break towards earlier in the list
                                if check0 > check1:
                                    if self.tracked_list[match[0]].fusion_steps >= self.trackShowThreshold:
                                        bisect.insort(remove, match[1])
                                elif check0 < check1:
                                    if self.tracked_list[match[1]].fusion_steps >= self.trackShowThreshold:
                                        bisect.insort(remove, match[0])
                                else:
                                    check0 = self.tracked_list[match[0]
                                                               ].fusion_steps
                                    check1 = self.tracked_list[match[1]
                                                               ].fusion_steps
                                    if check0 >= check1:
                                        if check0 >= self.trackShowThreshold:
                                            bisect.insort(remove, match[1])
                                    else:
                                        if check1 >= self.trackShowThreshold:
                                            bisect.insort(remove, match[0])

        for delete in reversed(remove):
            print("Cleaning track ", delete)
            self.tracked_list.pop(delete)
        # print(len(self.tracked_list), len(remove), remove)
