import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
from shapely.geometry import Polygon
import bisect
import shared_math


max_id = 10000


def binarySearch(a, x):
    'Locate the leftmost value exactly equal to x'
    i = bisect.bisect_left(a, x)
    if i != len(a) and a[i] == x:
        return i
    return -1


class MatchClass:
    def __init__(self, x, y, covariance, dx, dy, d_confidence, object_type, time, id):
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
        self.localTrackersIDList = []

        # Init the covariance to some value
        self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')
        self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype = 'float')

        # Store the first value for matching purposes
        self.x = x
        self.y = y

        # Arbitrary to start with, updated each iteration
        self.elapsed = 0.125

        # Arbitrary min tracking size so we are not too small to match to
        self.min_size = .5

        # Track the time of the last track
        self.lastTracked = time

        # Track the number of times this Kalman filter has been used
        self.idx = 0

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
            four = (.125*.125*.125*.125)/4.0
            three = (.125*.125*.125)/2.0
            two = (.125*.125)
            self.Q_t = np.array([[four, 0, three, 0],
                                [0, four, 0, three],
                                [three, 0, two, 0],
                                [0, three, 0, two]], dtype = 'float')
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
            five = (.125*.125*.125*.125*.125)/8.0
            four = (.125*.125*.125*.125)/4.0
            three = (.125*.125*.125)/2.0
            two = (.125*.125)
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
            four = (.125*.125*.125*.125)/4.0
            three = (.125*.125*.125)/2.0
            two = (.125*.125)
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
        # Regenerate the measurement array every frame
        self.H_t = np.zeros([len(self.localTrackersIDList), self.F_t_len], dtype = float)

        # Check if there are more sensors in the area that have not been added
        for match in measurement_list:
            print ( len(self.localTrackersIDList) )
            measurment_index = binarySearch(self.localTrackersIDList, match.id)
            print ( measurment_index )
            if measurment_index < 0:
                # This is not in the tracked list, needs to be added
                self.addTracker(match.id)
                measurment_index = len(self.localTrackersIDList) - 1
            print ( measurment_index )
            
            # All should be in the tracked list now, continue building the tables
            # Add the current covariance to the R matrix
            temp_index = 2 * measurment_index
            print ( temp_index )
            self.R_t[temp_index][temp_index] = match.covariance[0][0]
            self.R_t[temp_index][temp_index + 1] = match.covariance[0][1]
            self.R_t[temp_index + 1][temp_index] = match.covariance[1][0]
            self.R_t[temp_index + 1][temp_index + 1] = match.covariance[1][1]

            # Add the current measurments to the measurement matrix
            self.measure[temp_index] = match.x
            self.measure[temp_index + 1] = match.y

            # Measurement array needs to add 2 ones, always the same two locations
            # R needs 2 more rows
            np.r_[ self.R_t, np.zeros(self.F_t_len) ]
            np.r_[ self.R_t, np.zeros(self.F_t_len) ]

            self.H_t[measurment_index][0] = 1.0
            self.H_t[measurment_index + 1][1] = 1.0

    def addTracker(self, id):
        # Make sure we have something, if not create the array here
        # Start off with measuring only 1 sensor x and y, this will be dynamically built
        if len(self.localTrackersIDList) == 0:
            self.measure = np.array([0.0, 0.0])
            self.R_t = np.zeros([2, self.F_t_len], dtype = float)

            # Add to the list and list searcher
            self.localTrackersIDList.append(id)
            self.localTrackersTimeAliveList.append(self.lastTracked)
        else:
            # Not the first entry so we can use multiplication
            # Add to the list and list searcher
            self.localTrackersIDList.append(id)
            self.localTrackersTimeAliveList.append(self.lastTracked)

            newlength = len(self.localTrackersIDList) + 2

            # R needs 2 more columns
            np.c_[ self.R_t, np.zeros(newlength - 2) ]
            np.c_[ self.R_t, np.zeros(newlength - 2) ]

            # R needs 2 more rows
            np.r_[ self.R_t, np.zeros(newlength) ]
            np.r_[ self.R_t, np.zeros(newlength) ]

            # Current measurment array needs 2 extra spaces
            np.c_[ self.measure, [0.0], [0.0] ]

    def fusion(self, measurement_list, estimate_covariance):
        # Set the kalman variables and resize the arrays dynalically (if needed
        self.addFrames(measurement_list)
        # Do the kalman thing!
        if self.idx == 0:
            # We have no prior detection so we need to just output what we have but store for later
            # Do a Naive average to get the starting position
            a = np.matmul(self.measure, self.H_t)
            x_out = ( a[0] / len(self.localTrackersIDList) )
            y_out = ( a[1] / len(self.localTrackersIDList) )

            # Store so that next fusion is better
            self.X_hat_t = np.array(
                [[x_out], [y_out], [0], [0], [0], [0]])
            self.prev_time = self.lastTracked
            self.x = x_out
            self.y = y_out
            self.idx += 1

            if self.fusion_mode == 0:
                # Store so that next fusion is better
                self.X_hat_t = np.array(
                    [[x_out], [y_out], [0], [0]], dtype = 'float')
            elif self.fusion_mode == 1:
                # setup for x, dx, dxdx
                self.X_hat_t = np.array(
                    [[x_out], [y_out], [0], [0], [0], [0]], dtype = 'float')
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
                
                X_hat_t, self.P_hat_t = shared_math.kalman_prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

                # print ( "m ", measure )
                # print ( tempH_t )
                # print ( self.R_t )

                #print ( "P_hat: ", self.P_hat_t, " P_t: ", self.P_t )

                Z_t = (self.measure).transpose()
                Z_t = Z_t.reshape(Z_t.shape[0], -1)
                X_t, self.P_t = shared_math.kalman_update(X_hat_t, self.P_hat_t, Z_t, self.R_t, self.H_t)
                self.X_hat_t = X_t

                #print ( "P_hat2: ", self.P_hat_t, " P_t2: ", self.P_t )

                self.P_hat_t = self.P_t
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
    def __init__(self, sensed_id, x, y, covariance, object_type, time, id, fusion_mode):
        self.x = x
        self.y = y
        self.dx = 0
        self.dy = 0
        self.error_covariance = np.array([[1.0, 0.0], [0.0, 1.0]], dtype = 'float')
        self.typeArray = [0, 0, 0, 0]
        self.typeArray[object_type] += 1
        self.type = self.typeArray.index(max(self.typeArray))
        self.lastTracked = time
        self.id = id
        self.idx = 0
        self.min_size = 0.5
        self.track_count = 0
        self.d_covariance = np.array([[2.0, 0.0], [0.0, 2.0]], dtype = 'float')
        self.match_list = []

        # Add this first match
        new_match = MatchClass(x, y, covariance, None, None, None, object_type, time, sensed_id)
        self.match_list.append(new_match)

        # Kalman stuff
        self.fusion_mode = fusion_mode
        self.kalman = ResizableKalman(time, x, y, fusion_mode)

    # Update adds another detection to this track
    def update(self, other, time):
        new_match = MatchClass(other[1], other[2], other[3], None, None, None, other[4], time, other[0])
        self.match_list.append(new_match)

        self.lastTracked = time

        self.track_count += 1

    # Gets our position in an array form so we can use it in the BallTree
    def getPosition(self):
        return [
            [self.x, self.y, self.min_size, self.min_size, math.radians(0)]
        ]

    def getPositionPredicted(self, timestamp):
        x, y, a, b, phi = self.kalman.getKalmanPred(timestamp)
        return [
            [x, y, a, b, phi]
        ]

    def fusion(self, estimate_covariance):
        self.kalman.fusion(self.match_list, estimate_covariance)

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
        self.fusion_mode = fusion_mode

        # Indicate our success
        print('Started FUSION successfully...')

    def fuseDetectionFrame(self, estimate_covariance):
        # Time to go through each track list and fuse!
        result = []
        for track in self.trackedList:
            track.fusion(estimate_covariance)
            if track.track_count >= 3:
                result.append([track.id, track.x, track.y, track.error_covariance, track.dx, track.dy, track.d_covariance])
            # Clear the previous detection list
            track.clearLastFrame()

        return result

    def processDetectionFrame(self, sensor_id, timestamp, observations, cleanupTime, estimateCovariance):
        # We need to generate and add the detections from this detector
        detections_position_list = []
        detections_list = []
        for det in observations:
            # Create a rotated rectangle for IOU of 2 ellipses
            # [cx, cy, w, h, angle]
            if estimateCovariance and len(det) >= 3:
                # Calculate our 3 sigma std deviation to create a bounding box for matching
                try:
                    a, b, phi = det[2].extractErrorElipseParamsFromBivariateGaussian(3)
                    # Enforce a minimum size so matching doesn't fail
                    a += self.min_size
                    b += self.min_size
                    detections_position_list.append([det[0], det[1], a, b, phi])
                except Exception as e:
                    print ( " Failed! ", str(e))
                    # Use an arbitrary size if we have no covariance estimate
                    detections_position_list.append([det[0], det[1], self.min_size, self.min_size, math.radians(0)])
            else:
                # Use an arbitrary size if we have no covariance estimate
                detections_position_list.append([det[0], det[1], self.min_size, self.min_size, math.radians(0)])
            detections_list.append([0, det[0], det[1], det[2], sensor_id])

        # Call the matching function to modify our detections in trackedList
        self.matchDetections(detections_position_list, detections_list, timestamp, cleanupTime)

    def matchDetections(self, detections_list_positions, detection_list, timestamp, cleanupTime):
        matches = []
        if len(detections_list_positions) > 0:
            if len(self.trackedList) > 0:
                numpy_formatted = np.array(detections_list_positions).reshape(len(detections_list_positions), 5)
                thisFrameTrackTree = BallTree(numpy_formatted, metric=shared_math.computeDistanceEllipseBox)

                # Need to check the tree size here in order to figure out if we can even do this
                length = len(numpy_formatted)
                if length > 0:
                    for trackedListIdx, track in enumerate(self.trackedList):
                        # The only difference between this and our other version is that
                        # the below line is commented out
                        # track.calcEstimatedPos(timestamp - self.prev_time)
                        tuple = thisFrameTrackTree.query(np.array(track.getPositionPredicted(timestamp)), k=length,
                                                         return_distance=True)
                        first = True
                        for IOUVsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                            if .95 >= IOUVsDetection >= 0:
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
                    first = True
                    for IOUsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                        # Check to make sure thie IOU match is high
                        if .25 >= IOUsDetection >= 0:
                            # Make sure this is not ourself
                            if add != detectionIdx:
                                # If this is not ourself, add ourself only if none of our matches has been added yet
                                if detectionIdx in added:
                                    first = False
                                    break

                    # We are the best according to arbitrarily broken tie and can be added
                    if first:
                        added.append(add)
                        new = GlobalTracked(detection_list[add][0], detection_list[add][1], detection_list[add][2],
                                      detection_list[add][3], detection_list[add][4], timestamp, self.id, self.fusion_mode)
                        if self.id < 1000000:
                            self.id += 1
                        else:
                            self.id = 0
                        self.trackedList.append(new)

            else:
                for dl in detection_list:
                    new = GlobalTracked(dl[0], dl[1], dl[2], dl[3], dl[4], timestamp, self.id, self.fusion_mode)
                    if self.id < 1000000:
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