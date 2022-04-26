import math
import numpy as np
import scipy.stats as st
import bisect

def binarySearch(a, x):
    'Locate the leftmost value exactly equal to x'
    i = bisect.bisect_left(a, x)
    if i != len(a) and a[i] == x:
        return i
    return -1

class TruPercept():
    def __init__(self):
        # Amount of time in the past to look
        self.freshnessLength = 100
        # Theshold for IOU match
        self.iouThreshold = .5
        # Storage for detected vehicles
        self.trustStorageSearcher = []
        self.confidenceScore = []
        self.evaluationScore = []
        self.trustStorage = []
        self.visibilityScore = []
        # For second method
        self.trustStorage2 = []
        self.visibilityScore2 = []
        # Keep track of the last update
        self.lastUpdate = []

        # Record the results
        self.negMethod1 = 0
        self.posMethod1 = 0
        self.negMethod2 = 0
        self.posMethod2 = 0

    def addVehicleTrack(self, cavID, trackId, confidenceScore, evlauationScore, visibilityScore, trustFrameScore, visibilityScore2, trustFrameScore2, time):
        """Adds a new trust score to the list, calls the sub class to insert the vehicle to the field."""
        # Create a new AV
        try:
            if len(self.trustStorageSearcher) == 0:
                # If the list is empty bisect is going to return an error
                self.trustStorageSearcher.append(cavID+","+trackId)
                self.confidenceScore.append([confidenceScore])
                self.evaluationScore.append(evlauationScore)
                self.visibilityScore.append(visibilityScore)
                self.trustStorage.append([trustFrameScore])
                self.visibilityScore2.append(visibilityScore2)
                self.trustStorage2.append([trustFrameScore2])
                self.lastUpdate.append(time)
            else:
                # Use the bisect method to sort in place, we use the Searcher list to keep the class list in order
                position = bisect.bisect_left(self.trustStorageSearcher, cavID+","+trackId)
                # Insert the AV to the list
                self.trustStorageSearcher.insert(position, cavID+","+trackId)
                self.confidenceScore.insert(position, [confidenceScore])
                self.evaluationScore.insert(position, evlauationScore)
                self.visibilityScore.insert(position, visibilityScore)
                self.trustStorage.insert(position, [trustFrameScore])
                self.visibilityScore2.insert(position, visibilityScore2)
                self.trustStorage2.insert(position, [trustFrameScore2])
                self.lastUpdate.insert(position, time)
        except Exception as e:
            print(" Error addVehicleTrack ", str(e))

    def addTrustFrame(self, cavID, trackId, confidenceScore, evlauationScore, visibilityScore, trustFrameScore, visibilityScore2, trustFrameScore2, time):
        vehIDIndex = binarySearch(self.trustStorageSearcher, cavID+","+trackId)
        if vehIDIndex >= 0:
            self.confidenceScore[vehIDIndex].append(confidenceScore)
            self.evaluationScore[vehIDIndex] = evlauationScore
            self.visibilityScore[vehIDIndex] = visibilityScore
            self.trustStorage[vehIDIndex].append(trustFrameScore)
            self.visibilityScore2[vehIDIndex] = visibilityScore2
            self.trustStorage2[vehIDIndex].append(trustFrameScore2)
            self.lastUpdate[vehIDIndex] = time
            if len(self.trustStorage[vehIDIndex]) >= self.freshnessLength:
                self.confidenceScore[vehIDIndex].pop(0)
                self.trustStorage[vehIDIndex].pop(0)
                self.trustStorage2[vehIDIndex].pop(0)
            return True
        else:
            # If we fall through the loop to here, add the AV and respond accordingly
            self.addVehicleTrack(cavID, trackId, confidenceScore, evlauationScore, visibilityScore, trustFrameScore, visibilityScore2, trustFrameScore2, time)
            return True

    def calculateTrustFrameForDetection(self, trackedVehId, dataset, step):
        try:
            # Match the detection based on IOU
            # TODO: do this for real, for now we assume it is 100% correct

            # Calculate the visibility of the object from the sensor perspective
            # TODO: do this for real, for now we assume all vehicles are visible

            # Calculate the trust for the current frame

            # Trust of the current vehicle for current frame =
            # Sum of all : visibility * IOU match / Sum of all : visibility
            # Where all is every detection of a vehicle that the current vehicle has seen
            visibilitySum = 0.0
            visibilitySum2 = 0.0
            iouMatchSum = 0.0
            iouMatchSum2 = 0.0
            confidenceScore = []
            evaluationScore = []
            visibilityScore = []
            visibilityScore2 = []
            for j, observation in enumerate(dataset):
                # Get the expected error distance
                a, b, phi = observation.expectedErrorGaussian.extractErrorElipseParamsFromBivariateGaussian()
                error_expected = math.hypot(a, b)
                # TODO: change this, for now all visibility is 1
                # Method 1 from peper
                angle = math.atan(observation.horizontalCrossSection/observation.detectionDistance)
                visibilitySum += angle
                visibilityScore.append(visibilitySum)
                # Method 2, new method from
                visibilitySum2 += error_expected
                visibilityScore2.append(visibilitySum)
                # TODO: change this, for now all IOU is 1
                iou = 1
                if iou > self.iouThreshold:
                    error_actual = math.hypot(observation.errorX_actual, observation.errorY_actual)
                    # Calculate the standard deviations away, 0 is on the mean
                    std_deviations_away = error_actual/error_expected
                    # Divide this by 2 to give
                    confidence = (1 - st.norm.cdf(std_deviations_away))/2.0
                    evaluation = confidence
                    iouMatchSum += confidence
                    iouMatchSum2 += error_actual
                else:
                    confidence = 0.0
                    evaluation = confidence
                    iouMatchSum += 0.0
                    iouMatchSum2 += 0.0
                confidenceScore.append(confidence)
                evaluationScore.append(evaluation)
            vehicleTrustValue = iouMatchSum / visibilitySum
            vehicleTrustValue2 = iouMatchSum2 / visibilitySum2
            for j, observation in enumerate(dataset):
                self.addTrustFrame(observation.trackingId, trackedVehId, confidenceScore[j], evaluationScore[j], visibilityScore[j], vehicleTrustValue, visibilityScore2[j], vehicleTrustValue2, step)
        except Exception as e:
            print(" Error calculateTrustFrameForDetection: ", str(e))

    def calculateOverallTrust(self, step):
        minSteps = 5

        method1 = {}

        try:
            # Store the EVs
            evDict = {}
            evDictTime = {}
            evDictMin = {}
            # Brute force this, compute the whole array
            for name, confidenceScore, evaluationScore, visibilityScore, vehicleTrustValue, update in zip(self.trustStorageSearcher, self.confidenceScore, self.evaluationScore, self.visibilityScore, self.trustStorage, self.lastUpdate):
                cavID = name.split(',')[0]
                # Trust score storage
                trustScore = 0.0
                for confidence, trust in zip(confidenceScore, vehicleTrustValue):
                    trustScore += (confidence * trust) / confidence
                if cavID not in evDict:
                    evDict[cavID] = (visibilityScore * trustScore * evaluationScore)# / (visibilityScore * trustScore)
                    evDictMin[cavID] = 1
                else:
                    evDict[cavID] += (visibilityScore * trustScore * evaluationScore)# / (visibilityScore * trustScore)
                    evDictMin[cavID] += 1
                # Store the time for later computations
                if cavID not in evDictTime:
                    evDictTime[cavID] = update
                else:
                    # If we have an entry, check and see which time is later
                    if evDictTime[cavID] < update:
                        evDictTime[cavID] = update

            # Print the result for now
            print ("Sensor Eval:")
            for i in evDict:
                # Only print if this is the current timestep
                if evDictMin[i] >= minSteps:# and evDictTime[i] == step:
                    print( i, evDict[i] )
                    if evDict[i] < 1.0:
                        self.negMethod1 += 1
                        method1[i] = -1
                    else:
                        self.posMethod1 += 1
                        method1[i] = 1

        except Exception as e:
            print(" Error calculateOverallTrust: ", str(e))

        method2 = {}

        try:
            # Store the EVs
            evDict = {}
            evDictTime = {}
            evDictMin = {}
            # Brute force this, compute the whole array
            for name, confidenceScore, evaluationScore, visibilityScore, vehicleTrustValue, update in zip(self.trustStorageSearcher, self.confidenceScore, self.evaluationScore, self.visibilityScore2, self.trustStorage2, self.lastUpdate):
                cavID = name.split(',')[0]
                # Trust score storage
                trustScore = 0.0
                for confidence, trust in zip(confidenceScore, vehicleTrustValue):
                    trustScore += (confidence * trust) / confidence
                if cavID not in evDict:
                    evDict[cavID] = (visibilityScore * trustScore * evaluationScore)# / (visibilityScore * trustScore)
                    evDictMin[cavID] = 1
                else:
                    evDict[cavID] += (visibilityScore * trustScore * evaluationScore)# / (visibilityScore * trustScore)
                    evDictMin[cavID] += 1
                # Store the time for later computations
                if cavID not in evDictTime:
                    evDictTime[cavID] = update
                else:
                    # If we have an entry, checka nd see which time is later
                    if evDictTime[cavID] < update:
                        evDictTime[cavID] = update

            # Print the result for now
            print ("Sensor Eval Method 2:")
            for i in evDict:
                # Only print if this is the current timestep
                if evDictMin[i] >= minSteps:# and evDictTime[i] == step:
                    print( i, evDict[i] )
                    if evDict[i] < 1.0:
                        self.negMethod2 += 1
                        method2[i] = -1
                    else:
                        self.posMethod2 += 1
                        method2[i] = 1

        except Exception as e:
            print(" Error calculateOverallTrust: ", str(e))

        print ( " Method 1, pos: ", self.posMethod1, " neg: ", self.negMethod1)
        print ( " Method 2, pos: ", self.posMethod2, " neg: ", self.negMethod2)

        return method1, method2