import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree

from shared_library import shared_math


'''This object tracks a single object that has been detected in a LIDAR frame.
We use this primarily to match objects seen between frames and included in here
is a function for kalman filter to smooth the x and y values as well as a
function for prediction where the next bounding box will be based on prior movement.'''
class Tracked:
    def __init__(self, xmin, ymin, xmax, ymax, type, confidence, x, y, crossSection, time, id):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.x = x
        self.y = y
        self.lastX = self.x
        self.lastY = self.y
        self.dX = 0.0
        self.dY = 0.0
        self.timeToIntercept = 0.0
        self.typeArray = [0, 0, 0, 0]
        self.typeArray[type] += 1
        self.type = self.typeArray.index(max(self.typeArray))
        self.confidence = confidence
        self.lastTracked = time
        self.id = id
        self.relations = []
        self.velocity = 0
        self.lastVelocity = [0, 0, 0, 0, 0]
        self.lastTimePassed = 0
        self.lastXmin = 0
        self.lastYmin = 0
        self.lastXmax = 0
        self.lastYmax = 0
        self.lastX2min = 0
        self.lastY2min = 0
        self.lastX2max = 0
        self.lastY2max = 0
        self.lastHistory = 0
        self.crossSection = crossSection

        # Kalman filter for position
        self.acceleration = 0
        # Don't know this yet
        self.delta_t = 0
        # Transition matrix
        self.F_t = np.array([[1, 0, self.delta_t, 0], [0, 1, 0, self.delta_t], [0, 0, 1, 0], [0, 0, 0, 1]])
        # Initial State cov
        self.P_t = np.identity(4)
        # Process cov
        self.Q_t = np.identity(4)
        self.timeOfLifeInherited = 0
        # Initial State cov
        self.P_t = np.identity(4)
        # Process cov
        self.Q_t = np.identity(4)
        # Control matrix
        self.B_t = np.array([[0], [0], [0], [0]])
        # Control vector
        self.U_t = self.acceleration
        # Measurment Matrix
        self.H_t = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        # Measurment cov
        self.R_t = np.identity(2)
        #Set up the first iteration params
        self.X_hat_t = np.array([[self.x], [self.y], [0], [0]])

    def update(self, position, other, time, timePassed):
        self.xmin = position[0]
        self.ymin = position[1]
        self.xmax = position[2]
        self.ymax = position[3]
        self.typeArray[other[0]] += 1
        self.type = self.typeArray.index(max(self.typeArray))
        self.confidence = other[1]
        # Covnert to meters from feet here
        x_measured = other[2]# * 0.3048
        y_measured = other[3]# * 0.3048
        self.crossSection = other[4]# * 0.3048

        # Kalman filter stuff
        # We are changing the transition matrix every time with the changed timestep as sometimes the timestep is not consistent and
        # we should account for extra change
        self.F_t = np.array([[1, 0, timePassed, 0], [0, 1, 0, timePassed], [0, 0, 1, 0], [0, 0, 0, 1]])
        X_hat_t, self.P_hat_t = self.predictionKalman(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)
        measure_with_error = np.array([x_measured, y_measured])
        self.R_t = np.array([[1, 0],
                             [0, 1]])
        Z_t = (measure_with_error).transpose()
        Z_t = Z_t.reshape(Z_t.shape[0], -1)
        X_t, self.P_t = self.updateKalman(X_hat_t, self.P_hat_t, Z_t, self.R_t, self.H_t)
        self.X_hat_t = X_t
        self.P_hat_t = self.P_t
        # Now update our values for x and y
        self.x = .3*X_t[0][0] + .7*self.x
        self.y = .3*X_t[1][0] + .7*self.y

        # Velocity stuff
        # Calculate current velocity frame from the last 5 or less frames (if available)
        self.lastVelocity[self.lastHistory%5] = abs(math.hypot(self.x-self.lastX, self.y-self.lastY) / timePassed)
        self.lastHistory += 1
        if self.lastHistory >= 5:
            # We have 5 histories so divide by 5
            sum = 0
            for v in self.lastVelocity: sum += v
            self.velocity = sum/5.0
        else:
            # We have less than 5 histories, adjust as such
            sum = 0
            for v in self.lastVelocity: sum += v
            self.velocity = sum/self.lastHistory

        # Calcute the cahnge in x and change in y, we will use this for the timeToIntercept
        # variable which will be used in the case of forward collision warning.
        if self.x != self.lastX:
            self.dX = (0.3*self.dX) + (0.7*(self.x - self.lastX))
        if self.y != self.lastY:
            self.dY = (0.3*self.dY) + (0.7*(self.y - self.lastY))
        if self.dY != 0:
            self.timeToIntercept = (0.7*(self.y / self.dY * (1 / 30.0))) + (0.3*self.timeToIntercept )
        self.lastX = self.x
        self.lastY = self.y
        self.lastTracked = time

    # This is the predict function for the Kalman filter
    def predictionKalman(self, X_hat_t_1, P_t_1, F_t, B_t, U_t, Q_t):
        X_hat_t = F_t.dot(X_hat_t_1) + (B_t.dot(U_t).reshape(B_t.shape[0], -1))
        P_t = np.diag(np.diag(F_t.dot(P_t_1).dot(F_t.transpose()))) + Q_t
        return X_hat_t, P_t

    # This is the update function for the Kalman filter
    def updateKalman(self, X_hat_t, P_t, Z_t, R_t, H_t):
        K_prime = P_t.dot(H_t.transpose()).dot(np.linalg.inv(H_t.dot(P_t).dot(H_t.transpose()) + R_t))
        X_t = X_hat_t + K_prime.dot(Z_t - H_t.dot(X_hat_t))
        P_t = P_t - K_prime.dot(H_t).dot(P_t)

        return X_t, P_t

    # Gets our position in an array form so we can use it in the BallTree
    def getPosition(self):
        return [
         self.xmin, self.ymin, self.xmax, self.ymax]

    # Gets our predicted position in an array form so we can use it in the BallTree
    def getPositionPredicted(self):
        return [
         self.xminp, self.yminp, self.xmaxp, self.ymaxp]

    # Calcualtes our estimated next bounding box position velicity variables dxmin/max and dymin/max
    # using the previous x and y values
    def calcEstimatedPos(self, timePassed):
        # A moving average window of 3 seems most effective for this as it can change rapidly
        if self.lastHistory >= 2:
            dxmin = (0.7 * (self.xmin - self.lastXmin) / (timePassed)) + (
                        0.3 * (self.lastXmin - self.lastX2min) / (self.lastTimePassed))
            dymin = (0.7 * (self.ymin - self.lastYmin) / (timePassed)) + (
                        0.3 * (self.lastYmin - self.lastY2min) / (self.lastTimePassed))
            dxmax = (0.7 * (self.xmax - self.lastXmax) / (timePassed)) + (
                        0.3 * (self.lastXmax - self.lastX2max) / (self.lastTimePassed))
            dymax = (0.7 * (self.ymax - self.lastYmax) / (timePassed)) + (
                        0.3 * (self.lastYmax - self.lastY2max) / (self.lastTimePassed))
            self.lastX2min = self.lastXmin
            self.lastY2min = self.lastYmin
            self.lastX2max = self.lastXmax
            self.lastY2max = self.lastYmax
            self.lastXmin = self.xmin
            self.lastYmin = self.ymin
            self.lastXmax = self.xmax
            self.lastYmax = self.ymax
            self.xminp = self.xmin + dxmin * timePassed
            self.yminp = self.ymin + dymin * timePassed
            self.xmaxp = self.xmax + dxmax * timePassed
            self.ymaxp = self.ymax + dymax * timePassed
            #print("xp, x ", self.xp, self.x)
            #print("yp, y ", self.yp, self.y)
            self.lastTimePassed = timePassed
            return
        # When we have onle 1 history that math changes
        if self.lastHistory == 1:
            dxmin = (self.xmin - self.lastXmin)/(timePassed)
            dymin = (self.ymin - self.lastYmin)/(timePassed)
            dxmax = (self.xmax - self.lastXmax) / (timePassed)
            dymax = (self.ymax - self.lastYmax) / (timePassed)
            self.lastX2min = self.lastXmin
            self.lastY2min = self.lastYmin
            self.lastX2max = self.lastXmax
            self.lastY2max = self.lastYmax
            self.lastXmin = self.xmin
            self.lastYmin = self.ymin
            self.lastXmax = self.xmax
            self.lastYmax = self.ymax
            self.xminp = self.xmin + dxmin * timePassed
            self.yminp = self.ymin + dymin * timePassed
            self.xmaxp = self.xmax + dxmax * timePassed
            self.ymaxp = self.ymax + dymax * timePassed
            #print("xp, x " , self.xp, self.x)
            #print("yp, y ", self.yp, self.y)
            self.lastTimePassed = timePassed
            return
        # When we just initialized the object there is no math as we have no history

        if self.lastHistory == 0:
            self.lastXmin = self.xmin
            self.lastYmin = self.ymin
            self.lastXmax = self.xmax
            self.lastYmax = self.ymax
            self.xminp = self.xmin
            self.yminp = self.ymin
            self.xmaxp = self.xmax
            self.ymaxp = self.ymax
            return

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
    if (width<0) or (height <0):
        return 1
    area_overlap = width * height

    # COMBINED AREA
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    area_combined = area_a + area_b - area_overlap

    # RATIO OF AREA OF OVERLAP OVER COMBINED AREA
    iou = area_overlap / (area_combined+epsilon)

    # Modified to invert the IOU so that it works with the BallTree class
    if iou <= 0:
        distance = 1
    else:
        distance = 1 - iou
    return distance


'''This class contains the code for parsing the data from the LIDAR process.
Uses DBscan to cluster points and then matches them to either potential CAVs or
other large obstacles (read carboard boxes used for localization, walls, etc.)
so that we do not match large objects to moving CAVs during fusion.'''
class LIDAR:
    def __init__(self, timestamp):
        # Set other parameters for the class
        self.trackedList = []
        self.id = 0
        self.time = 0
        self.prev_time = timestamp
        self.min_size = .2

        # Indicate our success
        print('Started LIDAR successfully...')

    def processLidarFrame(self, output, timestamp, vehicle_x, vehicle_y, vehicle_theta, lidar_sensor):
        debug = False

        if len(output) < 1:
            return [[],timestamp]

        #print ( "got", output )

        array = np.array(output)

        #print ( "conv", array )

        db = DBSCAN(eps=0.1, min_samples=3).fit(array)
        y_pred = db.fit_predict(array)
        
        if debug:
            print ( y_pred )

        # Get the length and intiialize the arrays accordingly
        n_clusters_ = len(set(y_pred)) - (1 if -1 in y_pred else 0)
        clusterMaxRange = [0.0] * n_clusters_
        centerPointsX = [0.0] * n_clusters_
        centerPointsY = [0.0] * n_clusters_
        numPoints = [0] * n_clusters_

        # Get the centerpoint avarage
        for point, idx in zip(array, y_pred):
            numPoints[idx] += 1
            centerPointsX[idx] += point[0]
            centerPointsY[idx] += point[1]
        for idx, num in enumerate(numPoints):
            centerPointsX[idx] = centerPointsX[idx] / num
            centerPointsY[idx] = centerPointsY[idx] / num

        # Get the max range of the points from the centerpoint
        for point, idx in zip(array, y_pred):
            newRange = math.hypot(point[0] - centerPointsX[idx], point[1] - centerPointsY[idx])
            if newRange > clusterMaxRange[idx]:
                clusterMaxRange[idx] = newRange
        if debug:
            print ( clusterMaxRange )
            print ( centerPointsX, centerPointsY )

        # Filter out the big objects from the small ones
        # Small objects could be our vehicles, big ones are stationary
        # This is a pretty arbitrary assumption but it works well for
        # our test case
        bigX = []
        bigY = []
        smallX = []
        smallY = []
        for clusterRange, x, y in zip(clusterMaxRange, centerPointsX, centerPointsY):
            if clusterRange > 1.5:
                bigX.append(x)
                bigY.append(y)
            else:
                smallX.append(x)
                smallY.append(y)

        # For now we are only going to consider the small detections
        # TODO: do something with the large ones
        detections_position_list = []
        detections_list = []

        # For the small detections we should add .25 to offset for the center of the vehicle from teh edge
        for x, y in zip(smallX, smallY):
            det_dir = math.atan2(vehicle_y - y, vehicle_x - x) - math.radians(180)
            x = x + (.25 * math.cos(det_dir))
            y = y + (.25 * math.sin(det_dir))
            detections_position_list.append(
                [x - self.min_size, y - self.min_size, x + self.min_size, y + self.min_size])
            detections_list.append([0, 90, x, y, self.min_size * 2])

        # Call the matching function to modilfy our detections in trackedList
        self.matchDetections(detections_position_list, detections_list, timestamp)

        # Print the clusters as well as the small and large points on top
        if debug:
            plt.cla()
            # Create our output array
            plotx = []
            ploty = []
            labels = []
            for detection in self.trackedList:
                if detection.lastHistory >= 1:
                    plotx.append(detection.x)
                    ploty.append(detection.y)
                    labels.append(detection.id)
            plt.scatter(array[:,0], array[:,1],c=y_pred, cmap='Paired')
            plt.scatter(plotx, ploty, c='yellow')
            for i, txt in enumerate(labels):
                plt.annotate(txt, (plotx[i], ploty[i]))
            plt.scatter(bigX, bigY, c ='red')
            #plt.scatter(centerPointsX, centerPointsY)
            plt.title("DBSCAN")
            plt.pause(0.05)

        result = []
        for track in self.trackedList:
            if track.lastHistory >= 5:
                result.append([track.x, track.y])

        return result, timestamp


    '''Similar to the frame by frame matchign done for image recognition to keeps IDs contant, this fuction
    does the same but for LIDAR point boundin boxes instead of image bounding boxes. It operates from an 
    overhead perpective rather than a horizonatal one like a a camera'''
    def matchDetections(self, detections_list_positions, detection_list, timestamp):
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
                        track.calcEstimatedPos(timestamp - self.prev_time)
                        tuple = thisFrameTrackTree.query((np.array([track.getPositionPredicted()])), k=length, return_distance=True)
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
                        # Single mathc, go ahead and update the location
                        track.update(detections_list_positions[track.relations[0][0]], detection_list[track.relations[0][0]], self.time, timestamp - self.prev_time)
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
                    for IOUVsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                        # Check to make sure thie IOU match is high
                        if .75 >= IOUVsDetection >= 0:
                            # Make sure this is not ourself
                            if add != detectionIdx:
                                # If this is not ourself, add ourself only if none of our matches has been added yet
                                if detectionIdx in added:
                                    first = False
                                    break

                    # We are the best according to arbitrarily broken tie and can be added
                    if first:
                        added.append(add)
                        new = Tracked(detections_list_positions[add][0], detections_list_positions[add][1], detections_list_positions[add][2], detections_list_positions[add][3], detection_list[add][0], detection_list[add][1], detection_list[add][2], detection_list[add][3], detection_list[add][4], self.time, self.id)
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
