import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree
from shapely.geometry import Polygon

CAMERA = 0
LIDAR = 1


class Tracked:
    # This object tracks a single object that has been detected in a video frame.
    # We use this primarily to match objects seen between frames and included in here
    # is a function for kalman filter to smooth the x and y values as well as a
    # function for prediction where the next bounding box will be based on prior movement.
    def __init__(self, xmin, ymin, xmax, ymax, type, confidence, x, y, crossSection, sensorId, time, id):
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
        self.lastTracked = time
        self.id = id
        self.crossSection = crossSection
        self.trackVelocity = 0.0
        self.idx = 0
        self.min_size = .5

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

        # Kalman stuff
        # Set other parameters for the class
        self.prev_time = -99

        # Set up the Kalman filter
        # Initial State cov
        self.P_t = np.identity(8)
        # Process cov
        self.Q_t = np.identity(8)
        # End if it not commented
        # Control matrix
        self.B_t = np.array([[0], [0], [0], [0], [0], [0], [0], [0]])
        # Control vector
        self.U_t = 0
        # Measurment Matrix
        # Generated on the fly
        # Measurment cov
        self.R_t = np.identity(4)

    # Update adds another detection to this track
    def update(self, position, other, time, id):
        self.xmin_list.append(position[0])
        self.ymin_list.append(position[1])
        self.xmax_list.append(position[2])
        self.ymax_list.append(position[3])
        self.x_list.append(other[2])
        self.y_list.append(other[3])
        self.type_list.append(other[0])
        self.confidence_list.append(other[1])
        self.lastTracked_list.append(time)
        self.crossSection_list.append(other[4])
        self.sensorId_List.append(other[5])

        self.lastTracked = time

    # Gets our position in an array form so we can use it in the BallTree
    def getPosition(self):
        return [
            [self.x - self.min_size, self.y + self.min_size, self.x + self.min_size, self.y - self.min_size]
        ]

    def getPositionPredicted(self, timestamp):
        x, y = self.getKalmanPred(timestamp)
        return [
         x - self.min_size, y + self.min_size, x + self.min_size, y - self.min_size
        ]

    def fusion(self):
        debug = False

        lidarCov = [[0, 0], [0, 0]]
        camCov = [[0, 0], [0, 0]]
        lidarMeasure = [0, 0]
        camMeasure = [0, 0]
        lidarMeasureH = [0, 0]
        camMeasureH = [0, 0]

        # Time to go through the track list and fuse!
        for x, y, theta, sensor_id in zip(self.x_list, self.y_list, self.crossSection_list, self.sensorId_List):
            if sensor_id == LIDAR:
                lidarCov = [[1, 0], [0, 1]]
                lidarMeasure = [x, y]
                lidarMeasureH = [1, 1]
            elif sensor_id == CAMERA:
                camCov = [[1, 0], [0, 1]]
                camMeasure = [x, y]
                camMeasureH = [1, 1]

        # Now do the kalman thing!
        if self.idx == 0:
            # We have no prior detection so we need to just output what we have but store for later
            # Do a Naive average to get the starting position
            if camMeasure[0] != 0 and lidarMeasure[0]:
                x_out = (camMeasure[0] + lidarMeasure[0]) / 2.0
                y_out = (camMeasure[1] + lidarMeasure[1]) / 2.0
            elif camMeasure[0] != 0:
                x_out = camMeasure[0]
                y_out = camMeasure[1]
            elif lidarMeasure[0] != 0:
                x_out = lidarMeasure[0]
                y_out = lidarMeasure[1]
            # Store so that next fusion is better
            self.X_hat_t = np.array(
                [[x_out], [y_out], [0], [0], [0], [0], [0], [0]])
            self.prev_time = self.lastTracked
            print(x_out, y_out, lidarMeasure[0], lidarMeasure[1], camMeasure[0], camMeasure[1], self.idx)
            self.x = x_out
            self.y = y_out
            self.idx += 1
        else:
            try:
                # We have valid data
                # Transition matrix
                elapsed = self.lastTracked - self.prev_time
                print("elapsed ", elapsed)
                if elapsed < 0.0:
                    print( "Error time elapsed is incorrect! " + str(elapsed) )
                    # Set to arbitrary time
                    elapsed = 0.125

                self.F_t = np.array([[1, 0, 0, 0, elapsed, 0, elapsed*elapsed, 0],
                                    [0, 1, 0, 0, 0, elapsed, 0, elapsed*elapsed],
                                    [0, 0, 1, 0, elapsed, 0, elapsed*elapsed, 0],
                                    [0, 0, 0, 1, 0, elapsed, 0, elapsed*elapsed],
                                    [0, 0, 0, 0, 1, 0, elapsed, 0],
                                    [0, 0, 0, 0, 0, 1, 0, elapsed],
                                    [0, 0, 0, 0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1]])
                
                X_hat_t, self.P_hat_t = prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

                tempH_t = np.array([[lidarMeasureH[0], 0, 0, 0, 0, 0, 0, 0],
                                    [0, lidarMeasureH[1], 0, 0, 0, 0, 0, 0],
                                    [0, 0, camMeasureH[0], 0, 0, 0, 0, 0],
                                    [0, 0, 0, camMeasureH[1], 0, 0, 0, 0]])

                measure_with_error = np.array(
                    [X_hat_t[0][0] - lidarMeasure[0], X_hat_t[1][0] - lidarMeasure[1], X_hat_t[0][0] - camMeasure[0], X_hat_t[1][0] - camMeasure[1]])

                measure_with_error = np.array([lidarMeasure[0], lidarMeasure[1], camMeasure[0], camMeasure[1]])

                print ( measure_with_error )
                
                # Measurment cov
                self.R_t = np.array(
                    [[lidarCov[0][0], lidarCov[0][1], 0, 0],
                     [lidarCov[1][0], lidarCov[1][1], 0, 0],
                     [0, 0, camCov[0][0], camCov[0][1]],
                     [0, 0, camCov[1][0], camCov[1][1]]])

                Z_t = (measure_with_error).transpose()
                Z_t = Z_t.reshape(Z_t.shape[0], -1)
                X_t, self.P_t = update(X_hat_t, self.P_hat_t, Z_t, self.R_t, tempH_t)
                self.X_hat_t = X_t
                self.P_hat_t = self.P_t
                x_out = X_t[0][0]
                y_out = X_t[1][0]
                print ( x_out, y_out, lidarMeasure[0], lidarMeasure[1], camMeasure[0], camMeasure[1], self.idx)
                self.prev_time = self.lastTracked
                self.x = x_out
                self.y = y_out
                self.idx += 1
            except Exception as e:
                print ( " Exception: " + str(e) )

    def getKalmanPred(self, time):
        if self.idx > 0:
            elapsed = time - self.prev_time
            print("elapsed ", elapsed)
            if elapsed < 0.0:
                print("Error time elapsed is incorrect! " + str(elapsed))
                # Set to arbitrary time
                elapsed = 0.125
            self.F_t = np.array([[1, 0, 0, 0, elapsed, 0, elapsed, 0]
                                    , [0, 1, 0, 0, 0, elapsed, 0, elapsed]
                                    , [0, 0, 1, 0, elapsed, 0, elapsed, 0]
                                    , [0, 0, 0, 1, 0, elapsed, 0, elapsed]
                                    , [0, 0, 0, 0, 1, 0, elapsed, 0]
                                    , [0, 0, 0, 0, 0, 1, 0, elapsed]
                                    , [0, 0, 0, 0, 0, 0, 1, 0]
                                    , [0, 0, 0, 0, 0, 0, 0, 1]])
            X_hat_t, P_hat_t = prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)

            return X_hat_t[0][0], X_hat_t[1][0]
        else:
            return self.x, self.y


def computeDistance(a, b, epsilon=1e-5):
    x1, y1, x2, y2 = a
    polygonA = Polygon([(x1, y1), (x2, y1), (x2, y2), (x1, y1)])
    x1, y1, x2, y2 = b
    polygonb = Polygon([(x1, y1), (x2, y1), (x2, y2), (x1, y1)])

    intersection = polygonA.intersection(polygonb)

    if intersection.area <= 0.0:
        return 1

    iou = intersection.area / polygonA.union(polygonb).area

    # Modified to invert the IOU so that it works with the BallTree class
    if iou <= 0:
        distance = 1
    else:
        distance = 1 - iou
    return distance

# Fusion is a special class for matching and fusing detections for a variety of sources.
# The inpus is scalable and therefore must be generated before being fed into this class.
# A unique list of detections is required from each individual sensor or pre-fused device
# output or it will not be matched. Detections too close to each other may be combined.
# This is a modified version of the frame-by-frame tracker seen in:
# https://github.com/eandert/Jetson_Nano_Camera_Vehicle_Tracker
class FUSION:
    def __init__(self):
        # Set other parameters for the class
        self.trackedList = []
        self.id = 0
        self.prev_time = -99.0
        self.min_size = .5

        # Indicate our success
        print('Started FUSION successfully...')

    def dumpDetectionFrame(self):
        # Build the result list
        result = []
        for track in self.trackedList:
            result.append([track.id, track.x_list, track.y_list, track.crossSection_list])

        # Now clear the internal list
        #self.trackedList = []

        return result

    def fuseDetectionFrame(self):
        debug = False
        result = []

        # Time to go through each track list and fuse!
        for track in self.trackedList:
            track.fusion()
            result.append([track.id, track.x, track.y])

        return result

    def processDetectionFrame(self, sensor_id, timestamp, observations, cleanupTime):
        debug = False

        # We need to generate and add the detections from this detector
        detections_position_list = []
        detections_list = []
        for det in observations:
            detections_position_list.append([det[0] - self.min_size, det[1] + self.min_size, det[0] + self.min_size, det[1] - self.min_size])
            detections_list.append([0, 90, det[0], det[1], self.min_size * 2, sensor_id])

        print("past")
        print(detections_position_list)

        # Call the matching function to modify our detections in trackedList
        self.matchDetections(detections_position_list, detections_list, timestamp, cleanupTime)

    def matchDetections(self, detections_list_positions, detection_list, timestamp, cleanupTime):
        matches = []
        print ( len(detections_list_positions) )
        if len(detections_list_positions) > 0:
            print(len(self.trackedList))
            if len(self.trackedList) > 0:
                numpy_formatted = np.array(detections_list_positions).reshape(len(detections_list_positions), 4)
                thisFrameTrackTree = BallTree(numpy_formatted, metric=computeDistance)

                # Need to check the tree size here in order to figure out if we can even do this
                length = len(numpy_formatted)
                if length > 0:
                    for trackedListIdx, track in enumerate(self.trackedList):
                        # The only difference between this and our other version is that
                        # the below line is commented out
                        # track.calcEstimatedPos(timestamp - self.prev_time)
                        tuple = thisFrameTrackTree.query((np.array([track.getPositionPredicted(timestamp)])), k=length,
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
                        track.update(detections_list_positions[track.relations[0][0]],
                                     detection_list[track.relations[0][0]], timestamp, timestamp - self.prev_time)
                    elif len(track.relations) > 1:
                        # if we have multiple matches, pick the best one
                        max = 0
                        idx = -99
                        for rel in track.relations:
                            if rel[1] < max:
                                max = rel[1]
                                idx = rel[0]

                        if idx != -99:
                            track.update(detections_list_positions[idx], detection_list[idx])

                if len(matches):
                    missing = sorted(set(range(0, len(detections_list_positions))) - set([i[0] for i in matches]))
                else:
                    missing = list(range(0, len(detections_list_positions)))

                added = []
                for add in missing:
                    # Before we add anything, let's check back against the list to make sure there is no IOU match over .75 with this new item and another new item
                    tuple = thisFrameTrackTree.query((np.array([detections_list_positions[add]])), k=length,
                                                     return_distance=True)
                    first = True
                    for IOUsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                        # Check to make sure thie IOU match is high
                        if .75 >= IOUsDetection >= 0:
                            # Make sure this is not ourself
                            if add != detectionIdx:
                                # If this is not ourself, add ourself only if none of our matches has been added yet
                                if detectionIdx in added:
                                    first = False
                                    break

                    # We are the best according to arbitrarily broken tie and can be added
                    if first:
                        added.append(add)
                        print("new_p")
                        new = Tracked(detections_list_positions[add][0], detections_list_positions[add][1],
                                      detections_list_positions[add][2], detections_list_positions[add][3],
                                      detection_list[add][0], detection_list[add][1], detection_list[add][2],
                                      detection_list[add][3], detection_list[add][4], detection_list[add][5], timestamp, self.id)
                        print("new")
                        if self.id < 1000000:
                            self.id += 1
                        else:
                            self.id = 0
                        self.trackedList.append(new)

            else:
                for dl, dlp in zip(detection_list, detections_list_positions):
                    print("add_p")
                    new = Tracked(dlp[0], dlp[1], dlp[2], dlp[3], dl[0], dl[1], dl[2], dl[3], dl[4], dl[5], timestamp, self.id)
                    print ("add")
                    if self.id < 1000:
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

def prediction(X_hat_t_1, P_t_1, F_t, B_t, U_t, Q_t):
    X_hat_t = F_t.dot(X_hat_t_1) + (B_t.dot(U_t).reshape(B_t.shape[0], -1))
    P_t = np.diag(np.diag(F_t.dot(P_t_1).dot(F_t.transpose()))) + Q_t
    return X_hat_t, P_t

def update(X_hat_t, P_t, Z_t, R_t, H_t):
    K_prime = P_t.dot(H_t.transpose()).dot(np.linalg.inv(H_t.dot(P_t).dot(H_t.transpose()) + R_t))
    # print("K:\n",K_prime)
    # print("X_hat:\n",X_hat_t)
    X_t = X_hat_t + K_prime.dot(Z_t - H_t.dot(X_hat_t))
    P_t = P_t - K_prime.dot(H_t).dot(P_t)
    return X_t, P_t