import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
from shapely.geometry import Polygon
import bisect


def binarySearch(a, x):
    'Locate the leftmost value exactly equal to x'
    i = bisect.bisect_left(a, x)
    if i != len(a) and a[i] == x:
        return i
    return -1


class ResizableKalman:
    def __init__(self, time, x, y):
        # This list will take 
        self.localTrackersList = []
        self.localTrackersIDList = []

        # Store our last position
        self.x = x
        self.y = y

        # Arbitrary to start with, updated each iteration
        self.elapsed = 0.125

        # Track the time of the last track
        self.lastTracked = time

        # Track the number of times this Kalman filter has been used
        self.idx = 0

        # Set up the Kalman filter
        # Initial State cov
        if self.fusion_mode == 0:
            # Set up the Kalman filter
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
            # Measurment cov
            self.R_t = np.identity(6)
        else:
            # model from https://journals.sagepub.com/doi/abs/10.1177/0959651820975523
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
            # Measurment cov
            self.R_t = np.identity(5)

        # Start off with measuring only 1 sensor x and y, this will be dynamically built
        self.H_t = np.array([[1, 0],
                            [0, 1]])
        self.measure = np.array([0.0, 0.0])

    def addFilterFrames(self, measurementList):
        # Regenerate the measurement array every frame
        self.H_t = np.zeros([self.F_t_len, len(measurementList)], dtype = float)

        # Check if there are more sensors in the area that have not been added
        for each in measurementList:
            measurment_index = binarySearch(self.localTrackersIDList, each[0])
            if measurment_index < 0:
                # This is not in the tracked list, needs to be added
                self.addTracker(each[0])
                measurment_index = len(self.localTrackersIDList) - 1
            
            # All should be in the tracked list now, continue building the tables
            # Add the current covariance to the R matrix
            temp_index = 2 * measurment_index
            self.R_t[temp_index][temp_index] = each[2][0][0]
            self.R_t[temp_index][temp_index + 1] = each[2][0][1]
            self.R_t[temp_index + 1][temp_index] = each[2][1][0]
            self.R_t[temp_index + 1][temp_index + 1] = each[2][1][1]

            # Add the current measurments to the measurement matrix
            self.measure[temp_index] = each[1][0]
            self.measure[temp_index + 1] = each[1][1]

            # Measurement array needs to add 2 ones, always the same two locations
            self.H_t[measurment_index][0] = 1.0
            self.H_t[measurment_index + 1][1] = 1.0

    def addTracker(self, id):
        # Add to the list and list searcher
        self.localTrackersIDList.append(id)

        newlength = len(self.localTrackersIDList) * 2

        # R needs 2 more columns
        np.c_[ self.R_t, np.zeros(newlength - 2) ]
        np.c_[ self.R_t, np.zeros(newlength - 2) ]

        # R needs 2 more rows
        np.r_[ self.R_t, np.zeros(newlength) ]
        np.r_[ self.R_t, np.zeros(newlength) ]

        # Current measurment array needs 2 extra spaces
        np.c_[ self.measure, [0.0], [0.0] ]

    def fusion(self, estimate_covariance):
        debug = False

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
        else:
            try:
                # We have valid data
                # Transition matrix
                elapsed = self.lastTracked - self.prev_time
                if elapsed < 0.0:
                    print( "Error time elapsed is incorrect! " + str(elapsed) )
                    # Set to arbitrary time
                    elapsed = 0.125

                self.F_t = np.array([[1, 0, elapsed, 0, elapsed*elapsed, 0],
                                    [0, 1, 0, elapsed, 0, elapsed*elapsed],
                                    [0, 0, 1, 0, elapsed, 0],
                                    [0, 0, 0, 1, 0, elapsed],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]])
                
                X_hat_t, self.P_hat_t = prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

                Z_t = (self.measure).transpose()
                Z_t = Z_t.reshape(Z_t.shape[0], -1)
                X_t, self.P_t = update(X_hat_t, self.P_hat_t, Z_t, self.R_t, self.H_t)
                self.X_hat_t = X_t
                self.P_hat_t = self.P_t
                x_out = X_t[0][0]
                y_out = X_t[1][0]
                self.prev_time = self.lastTracked
                self.x = x_out
                self.y = y_out
                self.idx += 1
            except Exception as e:
                print ( " Exception: " + str(e) )

    def getKalmanPred(self, time):
        if self.idx > 0:
            elapsed = time - self.prev_time
            if elapsed < 0.0:
                print("Error time elapsed is incorrect! " + str(elapsed))
                # Set to arbitrary time
                elapsed = 0.125
            self.F_t = np.array([[1, 0, elapsed, 0, elapsed*elapsed, 0],
                                    [0, 1, 0, elapsed, 0, elapsed*elapsed],
                                    [0, 0, 1, 0, elapsed, 0],
                                    [0, 0, 0, 1, 0, elapsed],
                                    [0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 1]])
            X_hat_t, P_hat_t = prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

            return X_hat_t[0][0], X_hat_t[1][0]
        else:
            return self.x, self.y


class GlobalTracked:
    # This object tracks a single object that has been detected in a video frame.
    # We use this primarily to match objects seen between frames and included in here
    # is a function for kalman filter to smooth the x and y values as well as a
    # function for prediction where the next bounding box will be based on prior movement.
    def __init__(self, xmin, ymin, xmax, ymax, type, confidence, x, y, crossSection, sensorId, detectionId, covariance, time, id):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.x = x
        self.y = y
        self.typeArray = [0, 0, 0, 0]
        self.typeArray[type] += 1
        self.type = self.typeArray.index(max(self.typeArray))
        self.confidence = confidence
        self.id = id
        self.crossSection = crossSection
        self.trackVelocity = 0.0
        self.min_size = 1.0
        self.track_count = 0
        self.lastTracked = time

        self.xmin_list = []
        self.ymin_list = []
        self.xmax_list = []
        self.ymax_list = []
        self.x_list = []
        self.y_list = []
        self.type_list = []
        self.confidence_list = []
        self.lastTracked_list = []
        self.crossSection_list = []
        self.sensorId_List = []
        self.detectionId_List = []
        self.errorCovarianceList = []

        self.xmin_list.append(xmin)
        self.ymin_list.append(ymin)
        self.xmax_list.append(xmax)
        self.ymax_list.append(ymax)
        self.x_list.append(x)
        self.y_list.append(y)
        self.type_list.append(type)
        self.confidence_list.append(confidence)
        self.lastTracked_list.append(time)
        self.crossSection_list.append(crossSection)
        self.sensorId_List.append(sensorId)
        self.detectionId_List.append(detectionId)
        self.errorCovarianceList.append(covariance)

        # Kalman stuff
        self.kalman = ResizableKalman(time, x, y)

    # Update adds another detection to this track
    def update(self, position, other, time, id):
        self.x_list.append(other[2])
        self.y_list.append(other[3])
        self.type_list.append(other[0])
        self.confidence_list.append(other[1])
        self.lastTracked_list.append(time)
        self.crossSection_list.append(other[4])
        self.sensorId_List.append(other[5])
        self.detectionId_List.append(other[6])

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

    def fusion(self, estimate_covariance, vehicle):
        self.kalman.fusion(estimate_covariance)

    def clearLastFrame(self):
        self.x_list = []
        self.y_list = []
        self.type_list = []
        self.confidence_list = []
        self.lastTracked_list = []
        self.crossSection_list = []
        self.sensorId_List = []


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

    def dumpDetectionFrame(self):
        # Build the result list
        result = []
        for track in self.trackedList:
            result.append([track.detector_id, track.id, track.x_list, track.y_list, track.crossSection_list])

        return result

    def fuseDetectionFrame(self, estimate_covariance, vehicle):
        debug = False
        result = []

        # Time to go through each track list and fuse!
        for track in self.trackedList:
            track.fusion(estimate_covariance, vehicle)
            if track.track_count >= 3:
                result.append([track.detector_id, track.id, track.x, track.y, track.error_covariance, track.dx, track.dy])
            # Clear the previous detection list
            track.clearLastFrame()

        return result

    def processDetectionFrame(self, sensor_id, timestamp, observations, cleanupTime, estimateCovariance):
        debug = False

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
            detections_list.append([0, 90, det[0], det[1], self.min_size, sensor_id, det[5]])

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
                        track.update(detection_list[track.relations[0][0]], timestamp, timestamp - self.prev_time)
                    elif len(track.relations) > 1:
                        # if we have multiple matches, pick the best one
                        max = 0
                        idx = -99
                        for rel in track.relations:
                            if rel[1] < max:
                                max = rel[1]
                                idx = rel[0]

                        if idx != -99:
                            track.update(detection_list[idx])

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
                        new = Tracked(detection_list[add][0], detection_list[add][1], detection_list[add][2],
                                      detection_list[add][3], detection_list[add][4], detection_list[add][5], 
                                      detection_list[add][6], timestamp, self.id, self.fusion_mode)
                        if self.id < 1000000:
                            self.id += 1
                        else:
                            self.id = 0
                        self.trackedList.append(new)

            else:
                for dl in detection_list:
                    new = Tracked(dl[0], dl[1], dl[2], dl[3], dl[4], dl[5], dl[6], timestamp, self.id, self.fusion_mode)
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