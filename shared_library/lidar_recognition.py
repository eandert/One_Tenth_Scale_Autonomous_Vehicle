import math
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.neighbors import BallTree

from shared_library import shared_math


class ObjectTracker2D:
    """
    This object tracks a single object that has been detected in a LIDAR frame.
    We use this primarily to match objects seen between frames. Included in here
    is a function for a Kalman filter to smooth the x and y values, as well as a
    function for prediction where the next bounding box will be based on prior movement.

    Represents a tracked object with its position, velocity, and other attributes.

    Attributes:
        xmin (float): The minimum x-coordinate of the bounding box.
        ymin (float): The minimum y-coordinate of the bounding box.
        xmax (float): The maximum x-coordinate of the bounding box.
        ymax (float): The maximum y-coordinate of the bounding box.
        x (float): The current x-coordinate of the object.
        y (float): The current y-coordinate of the object.
        lastX (float): The previous x-coordinate of the object.
        lastY (float): The previous y-coordinate of the object.
        typeArray (list): An array to store the count of different types of the object.
        type (int): The most frequent type of the object.
        confidence (float): The confidence score of the object detection.
        last_tracked (float): The timestamp of the last update.
        id (int): The unique identifier of the object.
        relations (list): A list of related objects.
        velocity (float): The current velocity of the object.
        lastVelocity (list): A list of the last 5 velocities of the object.
        lastTimePassed (float): The time passed since the last update.
        lastXmin (float): The previous minimum x-coordinate of the bounding box.
        lastYmin (float): The previous minimum y-coordinate of the bounding box.
        lastXmax (float): The previous maximum x-coordinate of the bounding box.
        lastYmax (float): The previous maximum y-coordinate of the bounding box.
        lastX2min (float): The second previous minimum x-coordinate of the bounding box.
        lastY2min (float): The second previous minimum y-coordinate of the bounding box.
        lastX2max (float): The second previous maximum x-coordinate of the bounding box.
        lastY2max (float): The second previous maximum y-coordinate of the bounding box.
        track_history_length (int): The length of the track history.
        crossSection (float): The cross-sectional area of the object.
        acceleration (float): The acceleration of the object.
        delta_t (float): The time step for the Kalman filter.
        F_t (numpy.ndarray): The transition matrix for the Kalman filter.
        P_t (numpy.ndarray): The initial state covariance matrix for the Kalman filter.
        Q_t (numpy.ndarray): The process covariance matrix for the Kalman filter.
        timeOfLifeInherited (float): The time of life inherited from related objects.
        B_t (numpy.ndarray): The control matrix for the Kalman filter.
        U_t (float): The control vector for the Kalman filter.
        H_t (numpy.ndarray): The measurement matrix for the Kalman filter.
        R_t (numpy.ndarray): The measurement covariance matrix for the Kalman filter.
        X_hat_t (numpy.ndarray): The estimated state vector for the Kalman filter.
    """

    def __init__(self, xmin, ymin, xmax, ymax, type, confidence, x, y, crossSection, time, id):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.x = x
        self.y = y
        self.lastX = self.x
        self.lastY = self.y
        self.typeArray = [0, 0, 0, 0]
        self.typeArray[type] += 1
        self.type = self.typeArray.index(max(self.typeArray))
        self.confidence = confidence
        self.last_tracked = time
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
        self.track_history_length = 0
        self.crossSection = crossSection

        # Kalman filter for position
        self.acceleration = 0
        # This is .125 seconds
        self.delta_t = 0.125
        # Transition matrix
        self.F_t = np.array(
            [[1, 0, self.delta_t, 0], [0, 1, 0, self.delta_t], [0, 0, 1, 0], [0, 0, 0, 1]])
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
        # Set up the first iteration params
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
        x_measured = other[2]  # * 0.3048
        y_measured = other[3]  # * 0.3048
        self.crossSection = other[4]  # * 0.3048

        # Kalman filter stuff
        # We are changing the transition matrix every time with the changed timestep as sometimes the timestep is not consistent and
        # we should account for extra change
        self.F_t = np.array(
            [[1, 0, timePassed, 0], [0, 1, 0, timePassed], [0, 0, 1, 0], [0, 0, 0, 1]])
        X_hat_t, self.P_hat_t = self.predictionKalman(
            self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t, self.Q_t)
        measure_with_error = np.array([x_measured, y_measured])
        self.R_t = np.array([[1, 0],
                             [0, 1]])
        Z_t = (measure_with_error).transpose()
        Z_t = Z_t.reshape(Z_t.shape[0], -1)
        X_t, self.P_t = self.updateKalman(
            X_hat_t, self.P_hat_t, Z_t, self.R_t, self.H_t)
        self.X_hat_t = X_t
        self.P_hat_t = self.P_t
        # Now update our values for x and y
        self.x = .3*X_t[0][0] + .7*self.x
        self.y = .3*X_t[1][0] + .7*self.y

        # Velocity stuff
        # Calculate current velocity frame from the last 5 or less frames (if available)
        self.lastVelocity[self.track_history_length % 5] = abs(
            math.hypot(self.x-self.lastX, self.y-self.lastY) / timePassed)
        self.track_history_length += 1
        if self.track_history_length >= 5:
            # We have 5 histories so divide by 5
            sum = 0
            for v in self.lastVelocity:
                sum += v
            self.velocity = sum/5.0
        else:
            # We have less than 5 histories, adjust as such
            sum = 0
            for v in self.lastVelocity:
                sum += v
            self.velocity = sum/self.track_history_length

        # Setup the memory variables
        self.lastX = self.x
        self.lastY = self.y
        self.last_tracked = time

    def predictionKalman(self, X_hat_t_1, P_t_1, F_t, B_t, U_t, Q_t):
        """
        Perform Kalman prediction step.

        Args:
            X_hat_t_1 (numpy.ndarray): State estimate at time t-1.
            P_t_1 (numpy.ndarray): Covariance matrix of state estimate at time t-1.
            F_t (numpy.ndarray): State transition matrix.
            B_t (numpy.ndarray): Control input matrix.
            U_t (numpy.ndarray): Control input vector.
            Q_t (numpy.ndarray): Process noise covariance matrix.

        Returns:
            tuple: A tuple containing the updated state estimate (X_hat_t) and covariance matrix (P_t).
        """
        X_hat_t = F_t.dot(X_hat_t_1) + (B_t.dot(U_t).reshape(B_t.shape[0], -1))
        P_t = np.diag(np.diag(F_t.dot(P_t_1).dot(F_t.transpose()))) + Q_t
        return X_hat_t, P_t

    def updateKalman(self, X_hat_t, P_t, Z_t, R_t, H_t):
        """
        Updates the Kalman filter with the given parameters.

        Args:
            X_hat_t (numpy.ndarray): The estimated state vector at time t.
            P_t (numpy.ndarray): The estimated error covariance matrix at time t.
            Z_t (numpy.ndarray): The measurement vector at time t.
            R_t (numpy.ndarray): The measurement noise covariance matrix at time t.
            H_t (numpy.ndarray): The measurement matrix at time t.

        Returns:
            tuple: A tuple containing the updated state vector (X_t) and error covariance matrix (P_t).
        """
        K_prime = P_t.dot(H_t.transpose()).dot(
            np.linalg.inv(H_t.dot(P_t).dot(H_t.transpose()) + R_t))
        X_t = X_hat_t + K_prime.dot(Z_t - H_t.dot(X_hat_t))
        P_t = P_t - K_prime.dot(H_t).dot(P_t)

        return X_t, P_t

    def getPosition(self):
        """
        Returns the position of the object as a list of coordinates primarily so we can use it in the BallTree.

        Returns:
            list: A list containing the x and y coordinates of the object's minimum and maximum points.
        """
        return [self.xmin, self.ymin, self.xmax, self.ymax]

    def getPositionPredicted(self):
        """
        Returns the predicted position as an array.

        The predicted position is represented by the coordinates (xminp, yminp, xmaxp, ymaxp).
        This array can be used in the BallTree for further processing.

        Returns:
            list: The predicted position as an array [xminp, yminp, xmaxp, ymaxp].
        """
        return [self.xminp, self.yminp, self.xmaxp, self.ymaxp]

    def calcEstimatedPos(self, timePassed):
        """
        Calculates the estimated next bounding box position velocity variables dxmin/max and dymin/max
        using the previous x and y values.

        Args:
            timePassed (float): The time passed since the last calculation.

        Returns:
            None
        """
        # TODO(eandert): This should ideally be replaced with the Kalman filter output
        # A moving average window of 3 seems most effective for this as it can change rapidly
        if self.track_history_length >= 2:
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
            # print("xp, x ", self.xp, self.x)
            # print("yp, y ", self.yp, self.y)
            self.lastTimePassed = timePassed
            return
        # When we have onle 1 history that math changes
        if self.track_history_length == 1:
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
            # print("xp, x " , self.xp, self.x)
            # print("yp, y ", self.yp, self.y)
            self.lastTimePassed = timePassed
            return
        # When we just initialized the object there is no math as we have no history

        if self.track_history_length == 0:
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
    """
    Converts xmin, ymin, xmax, ymax to centroid x, y with height and width.

    Parameters:
    x (float): The x-coordinate of the bounding box center.
    y (float): The y-coordinate of the bounding box center.
    w (float): The width of the bounding box.
    h (float): The height of the bounding box.

    Returns:
    tuple: A tuple containing the converted values (xmin, ymin, xmax, ymax).
    """
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
    iou = area_overlap / (area_combined+epsilon)

    # Modified to invert the IOU so that it works with the BallTree class
    if iou <= 0:
        distance = 1
    else:
        distance = 1 - iou
    return distance


class LIDARClustering2D:
    """
    This class is responsible for parsing the data from the LIDAR process. It uses DBSCAN 
    (Density-Based Spatial Clustering of Applications with Noise) to cluster points and 
    then matches them to either potential Connected Autonomous Vehicles (CAVs) or other 
    large obstacles (such as cardboard boxes used for localization, walls, etc.). This 
    ensures that large objects are not incorrectly identified as moving CAVs during the 
    fusion process.

    Attributes:
        tracked_list (list): A list to keep track of identified objects.
        id (int): An identifier for the LIDARClustering2D instance.
        time (int): The current time.
        prev_time (int): The previous timestamp.
        min_size (float): The minimum size for an object to be considered in the clustering process.

    Methods:
        __init__(self, timestamp): Initializes the LIDARClustering2D instance with the given timestamp.
        processLidarFrame(self, output, timestamp, vehicle_x, vehicle_y, vehicle_theta, lidar_sensor): Process a lidar frame and perform object recognition.
        matchDetections(self, detections_list_positions, detection_list, timestamp): Matches LIDAR point bounding boxes to keep IDs constant across frames.
    """

    def __init__(self, timestamp):
        """
        Initializes the LIDARClustering2D instance with the given timestamp.

        Args:
            timestamp (int): The initial timestamp.

        Side effects:
            Prints a success message to the console.
        """
        # Set other parameters for the class
        self.tracked_list = []
        self.id = 0
        self.time = 0
        self.prev_time = timestamp
        self.min_size = .2

        # Indicate our success
        print('Started LIDAR successfully...')

    def processLidarFrame(self, output, timestamp, vehicle_x, vehicle_y, vehicle_theta, lidar_sensor):
        """
        Process a lidar frame and perform object recognition.

        Args:
            output (list): List of lidar points.
            timestamp (float): Timestamp of the lidar frame.
            vehicle_x (float): X-coordinate of the vehicle.
            vehicle_y (float): Y-coordinate of the vehicle.
            vehicle_theta (float): Orientation angle of the vehicle.
            lidar_sensor (object): Lidar sensor object.

        Returns:
            list: List of recognized objects with their properties.
            float: Updated timestamp.

        Raises:
            Exception: If an error occurs during the lidar frame processing.

        """
        try:
            debug = False

            if len(output) < 1:
                # probably the LIDAR is not on, skip but make a message about it
                print("LIDAR ERROR: No points detected")
                return [], timestamp

            unfiltered_array = np.array(output)

            # TODO(eandert): Filter points outside of our route / map
            # Create an empty list
            filter_arr = []

            # Go through each element in the list
            # for element in unfiltered_array:
            #     if bounding_box[0][0] >= element[0] <= bounding_box[0][1] and bounding_box[1][0] >= element[1] <= bounding_box[1][1]:
            #         filter_arr.append(True)
            #     else:
            #         filter_arr.append(False)

            # array = unfiltered_array[filter_arr]

            array = unfiltered_array

            db = DBSCAN(eps=0.1, min_samples=3).fit(array)
            y_pred = db.fit_predict(array)

            if debug:
                print(y_pred)

            # Get the length and intiialize the arrays accordingly
            n_clusters_ = len(set(y_pred)) - (1 if -1 in y_pred else 0)
            cluster_max_range = [0.0] * n_clusters_
            center_points_x = [0.0] * n_clusters_
            center_points_y = [0.0] * n_clusters_
            numPoints = [0] * n_clusters_

            # Get the centerpoint avarage
            for point, idx in zip(array, y_pred):
                numPoints[idx] += 1
                center_points_x[idx] += point[0]
                center_points_y[idx] += point[1]
            for idx, num in enumerate(numPoints):
                center_points_x[idx] = center_points_x[idx] / num
                center_points_y[idx] = center_points_y[idx] / num

            # Get the max range of the points from the centerpoint
            for point, idx in zip(array, y_pred):
                new_range = math.hypot(
                    point[0] - center_points_x[idx], point[1] - center_points_y[idx])
                if new_range > cluster_max_range[idx]:
                    cluster_max_range[idx] = new_range
            if debug:
                print(cluster_max_range)
                print(center_points_x, center_points_y)

            # Filter out the big objects from the small ones
            # Small objects could be our vehicles, big ones are stationary
            # This is a pretty arbitrary assumption but it works well for
            # our test case
            big_x = []
            big_y = []
            small_x = []
            small_y = []
            for clusterRange, x, y in zip(cluster_max_range, center_points_x, center_points_y):
                if clusterRange > 1.5:
                    big_x.append(x)
                    big_y.append(y)
                else:
                    small_x.append(x)
                    small_y.append(y)

            # For now we are only going to consider the small detections
            # TODO(eandert): do something with the large ones
            detections_position_list = []
            detections_list = []

            # For the small detections we should add .25 to offset for the center of the vehicle from the edge
            for x, y in zip(small_x, small_y):
                det_dir = math.atan2(
                    vehicle_y - y, vehicle_x - x) - math.radians(180)
                x = x + (.25 * math.cos(det_dir))
                y = y + (.25 * math.sin(det_dir))
                detections_position_list.append(
                    [x - self.min_size, y - self.min_size, x + self.min_size, y + self.min_size])
                detections_list.append([0, 90, x, y, self.min_size * 2])

            # Call the matching function to modilfy our detections in tracked_list
            self.matchDetections(detections_position_list,
                                 detections_list, timestamp)

            # Print the clusters as well as the small and large points on top
            if debug:
                plt.cla()
                # Create our output array
                plotx = []
                ploty = []
                labels = []
                for detection in self.tracked_list:
                    if detection.track_history_length >= 1:
                        plotx.append(detection.x)
                        ploty.append(detection.y)
                        labels.append(detection.id)
                plt.scatter(array[:, 0], array[:, 1], c=y_pred, cmap='Paired')
                plt.scatter(plotx, ploty, c='yellow')
                for i, txt in enumerate(labels):
                    plt.annotate(txt, (plotx[i], ploty[i]))
                plt.scatter(big_x, big_y, c='red')
                # plt.scatter(center_points_x, center_points_y)
                plt.title("DBSCAN")
                plt.pause(0.05)

            result = []
            for track in self.tracked_list:
                if track.track_history_length >= 5:
                    # Calculate the covariance
                    relative_angle_to_detector, target_line_angle, relative_distance = shared_math.get_relative_detection_params(
                        vehicle_x, vehicle_y, vehicle_theta, track.x, track.y)
                    success_lidar, expected_error_gaussian_lidar, actual_sim_error_lidar = lidar_sensor.calculateErrorGaussian(
                        relative_angle_to_detector, target_line_angle, relative_distance, True, True)
                    result.append(
                        [track.id, track.x, track.y, expected_error_gaussian_lidar.covariance.tolist(), 0.0, 0.0, []])

            return result, timestamp

        except Exception as e:
            print("LIDAR ERROR in processLidarFrame", e)
            return [], timestamp

    def matchDetections(self, detections_list_positions, detection_list, timestamp):
        '''Similar to the frame by frame matching done for image recognition to keep IDs constant, this function
        does the same but for LIDAR point bounding boxes instead of image bounding boxes. It operates from an 
        overhead perspective rather than a horizontal one like a camera.

        Args:
            detections_list_positions (list): List of positions of LIDAR point bounding boxes.
            detection_list (list): List of LIDAR point bounding boxes.
            timestamp (float): Timestamp of the current frame.

        Returns:
            None
        '''
        self.time += 1
        matches = []
        if len(detections_list_positions) > 0:
            if len(self.tracked_list) > 0:
                numpy_formatted = np.array(detections_list_positions).reshape(
                    len(detections_list_positions), 4)
                thisFrameTrackTree = BallTree(
                    numpy_formatted, metric=computeDistance)

                # Need to check the tree size here in order to figure out if we can even do this
                length = len(numpy_formatted)
                if length > 0:
                    for tracked_listIdx, track in enumerate(self.tracked_list):
                        track.calcEstimatedPos(timestamp - self.prev_time)
                        tuple = thisFrameTrackTree.query(
                            (np.array([track.getPositionPredicted()])), k=length, return_distance=True)
                        first = True
                        for IOUVsDetection, detectionIdx in zip(tuple[0][0], tuple[1][0]):
                            if .95 >= IOUVsDetection >= 0:
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
                        # Single mathc, go ahead and update the location
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
                            track.update(
                                detections_list_positions[idx], detection_list[idx])

                if len(matches):
                    missing = sorted(
                        set(range(0, len(detections_list_positions))) - set([i[0] for i in matches]))
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
                        new = ObjectTracker2D(detections_list_positions[add][0], detections_list_positions[add][1], detections_list_positions[add][2], detections_list_positions[add]
                                              [3], detection_list[add][0], detection_list[add][1], detection_list[add][2], detection_list[add][3], detection_list[add][4], self.time, self.id)
                        if self.id < 1000000:
                            self.id += 1
                        else:
                            self.id = 0
                        self.tracked_list.append(new)

            else:
                for dl, dlp in zip(detection_list, detections_list_positions):
                    new = ObjectTracker2D(dlp[0], dlp[1], dlp[2], dlp[3], dl[0],
                                          dl[1], dl[2], dl[3], dl[4], self.time, self.id)
                    if self.id < 1000:
                        self.id += 1
                    else:
                        self.id = 0
                    self.tracked_list.append(new)

        remove = []
        for idx, track in enumerate(self.tracked_list):
            track.relations = []
            if track.last_tracked < self.time - 2:
                remove.append(idx)

        for delete in reversed(remove):
            self.tracked_list.pop(delete)
