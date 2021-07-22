# import sys
# sys.path.insert(1, '../ATLAS/src')
#
# import sensor_calculation
# import sensor_fusion
#
#
# class Fusion:
#     def __init__(self):
#         self.fusion_average = sensor_fusion.SensorFusion(sensor_fusion.method_averageOfAllDetections)
#         self.fusion_kalman = sensor_fusion.SensorFusion(sensor_fusion.method_dxdyKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections)
#
# # Build the object where we store all the accuracy stats
# trackAccuracy = TrackAccuracyObject()
# trackAccuracy.trackingId = vehId
# trackAccuracy.horizontalCrossSection = horizontalCrossSection

import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree


class Tracked:
    # This object tracks a single object that has been detected in a video frame.
    # We use this primarily to match objects seen between frames and included in here
    # is a function for kalman filter to smooth the x and y values as well as a
    # function for prediction where the next bounding box will be based on prior movement.
    def __init__(self, xmin, ymin, xmax, ymax, type, confidence, x, y, crossSection, time, id):
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

    # Update adds another detection to this track
    def update(self, position, other, time, timePassed):
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

    # Gets our position in an array form so we can use it in the BallTree
    def getPosition(self):
        return [
            self.xmin, self.ymin, self.xmax, self.ymax]


def convertBack(x, y, w, h):
    # Converts xmin ymin xmax ymax to centroid x,y with height and width
    xmin = int(round(x - w / 2))
    xmax = int(round(x + w / 2))
    ymin = int(round(y - h / 2))
    ymax = int(round(y + h / 2))
    return (xmin, ymin, xmax, ymax)


def computeDistance(a, b, epsilon=1e-5):
    # From tutorial http://ronny.rest/tutorials/module/localization_001/iou/#
    """ Given two boxes `a` and `b` defined as a list of four numbers:
            [x1,y1,x2,y2]
        where:
            x1,y1 represent the upper left corner
            x2,y2 represent the lower right corner
        It returns the Intersect of Union score for these two boxes.

    Args:
        a:          (list of 4 numbers) [x1,y1,x2,y2]
        b:          (list of 4 numbers) [x1,y1,x2,y2]
        epsilon:    (float) Small value to prevent division by zero

    Returns:
        (float) The Intersect of Union score.
    """
    # COORDINATES OF THE INTERSECTION BOX
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])

    # AREA OF OVERLAP - Area where the boxes intersect
    width = (x2 - x1)
    height = (y2 - y1)
    # handle case where there is NO overlap
    if (width < 0) or (height < 0):
        return 1
    area_overlap = width * height

    # COMBINED AREA
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    area_combined = area_a + area_b - area_overlap

    # RATIO OF AREA OF OVERLAP OVER COMBINED AREA
    iou = area_overlap / (area_combined + epsilon)

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
    def __init__(self, timestamp):
        # Set other parameters for the class
        self.trackedList = []
        self.id = 0
        self.time = 0
        self.prev_time = timestamp
        self.min_size = .1

        # Indicate our success
        print('Started FUSION successfully...')

    def processDetectionFrame(self, id, timestamp, observations):
        debug = False

        # We need to generate and add the detections from this detector
        detections_position_list = []
        detections_list = []
        for det in observations:
            detections_position_list.append([det[0] - self.min_size, det[1] - self.min_size, det[0] + self.min_size, det[1] + self.min_size])
            detections_list.append([0, 90, det[0], det[1], self.min_size * 2])

        # Call the matching function to modilfy our detections in trackedList
        self.matchDetections(detections_position_list, detections_list, id, timestamp)

    def dumpDetectionFrame(self):
        # Build the result list
        result = []
        for track in self.trackedList:
            result.append([track.id, track.x_list, track.y_list, track.crossSection_list])

        # Now clear the internal list
        self.trackedList = []

        return result

    def matchDetections(self, detections_list_positions, detection_list, id, timestamp):
        self.time += 1
        matches = []
        if len(detections_list_positions) > 0:
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
                        tuple = thisFrameTrackTree.query((np.array([track.getPositionPredicted()])), k=length,
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
                                     detection_list[track.relations[0][0]], self.time, timestamp - self.prev_time)
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
                        new = Tracked(detections_list_positions[add][0], detections_list_positions[add][1],
                                      detections_list_positions[add][2], detections_list_positions[add][3],
                                      detection_list[add][0], detection_list[add][1], detection_list[add][2],
                                      detection_list[add][3], detection_list[add][4], self.time, self.id)
                        if self.id < 1000000:
                            self.id += 1
                        else:
                            self.id = 0
                        self.trackedList.append(new)

            else:
                for dl, dlp in zip(detection_list, detections_list_positions):
                    new = Tracked(dlp[0], dlp[1], dlp[2], dlp[3], dl[0], dl[1], dl[2], dl[3], dl[4], self.time, self.id)
                    if self.id < 1000:
                        self.id += 1
                    else:
                        self.id = 0
                    self.trackedList.append(new)

        remove = []
        for idx, track in enumerate(self.trackedList):
            track.relations = []
            if track.lastTracked < self.time - 2:
                remove.append(idx)

        for delete in reversed(remove):
            self.trackedList.pop(delete)
