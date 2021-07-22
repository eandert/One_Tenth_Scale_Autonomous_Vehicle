import numpy as np
import math
import matplotlib.pyplot as plt
import statistics 
import bisect

def binarySearch(a, x):
    'Locate the leftmost value exactly equal to x'
    i = bisect.bisect_left(a, x)
    if i != len(a) and a[i] == x:
        return i
    return -1

from matplotlib.patches import Ellipse

def relativeAngleDiff( angle1, angle2 ):
    anglediff = ((angle1 - angle2 + math.pi + (2 * math.pi)) % (2 * math.pi)) - math.pi
    return anglediff

class Sensor:
    def __init__(self, name, type, centerAngle, maxRange, horizontalFOV, horizontalResolution, horizontalPixels, distanceError_expected, velocityError_expected = -1, iouRateM = 0, iouRateDistanceM = 0):
        self.TYPE_CAMERA = 3
        self.TYPE_LIDAR = 4
        self.TYPE_RADAR = 5
        self.type = type
        self.name = name
        self.centerAngle = math.radians(centerAngle)
        self.maxRange = maxRange
        self.horizontalFOV = math.radians(horizontalFOV/2.0)
        self.horizontalPixels = horizontalPixels
        self.distanceError_expected = distanceError_expected
        self.velocityError_expected = velocityError_expected
        self.iouRateM = math.radians(iouRateM)
        self.iouRateDistanceM = iouRateDistanceM
        self.errorMagnitudeRadians = 0.0
        self.errorMagnitudeDistance = 0.0
        
        if self.type == self.TYPE_CAMERA:
            self.horizontalResolution = self.horizontalFOV/self.horizontalPixels
        else:
            self.horizontalResolution = math.radians(horizontalResolution)
            
        self.radialError_expected = 2 * self.horizontalResolution
        
    def checkInRangeAndFOV(self, targetAngle, distance):
        #print ( self.name, targetAngle, self.centerAngle, self.horizontalFOV, "  range max, dist", self.maxRange, distance )
        anglediff = ((self.centerAngle - targetAngle + math.pi + (2*math.pi)) % (2*math.pi)) - math.pi
        #print ( anglediff )
        if abs(anglediff) <= (self.horizontalFOV) and (distance <= self.maxRange):
            return True
            #print ( " in angle " )
        #print ( " not in angle " )
        return False
        
    def getRadialErrorAtDistance(self, distance, horizontalCrossSection):
        #print ( self.iouRateM, distance, math.asin((distance * self.iouRateM * horizontalCrossSection)/distance) )
        #return self.radialError_expected + math.asin((distance * self.iouRateM * horizontalCrossSection)/distance)
        # This version we are disregarding horizontal crosssection for now
        return self.radialError_expected + (distance * self.iouRateM)
        
    def getDistanceErrorAtDistance(self, distance, horizontalCrossSection):
        #print ( self.iouRateDistanceM, distance, (distance * self.iouRateDistanceM * horizontalCrossSection))
        return self.distanceError_expected + (distance * self.iouRateDistanceM)
    
                
class TrackedObject:
    def __init__(self, vehId):
        # Store when this is the first iteration so we do not calc certain values
        self.noStoredData = True
        self.last_step = -99
        self.vehId = vehId
        
        # Initial conditions
        self.lastExpectedErrorGaussian = BivariateGaussian(0, 0, 0)
        self.thetaErrorExpected_last = 0
        self.velocityErrorExpected_last = 0
        self.xErrorActual_last = 0
        self.yErrorActual_last = 0
        self.x_now_last = 0
        self.y_detect_last = 0
        
class TrackAccuracyObject:
    def __init__(self):
        self.trackingId = None
        self.errorX_actual = 0
        self.errorY_actual = 0
        self.errorAngle_actual = 0
        self.errorVelocity_actual = 0
        self.expectedErrorGaussian = BivariateGaussian(0,0,0)
        self.errorAngle_expected = 0
        self.errorVelocity_expected = 0
        self.detectionDistance = 0
        self.horizontalCrossSection = 0

class BivariateGaussian:
    def __init__(self, a, b, phi, mu = None, cov = None):
        if cov is None:
            # Create the bivariate gaussian matrix
            #self.mu = np.array([a, b])
            # Fixed 12/10 changed above to below
            self.mu = np.array([0.0,0.0])
            self.covariance = [[a, 0], [0, b]]

            # RΣR^T to rotate the ellipse where Σ is the original covariance matrix
            rotate = np.array([[math.cos(phi), math.sin(phi)], [-math.sin(phi), math.cos(phi)]])
            self.covariance = np.matmul(rotate, self.covariance)
            self.covariance = np.matmul(self.covariance, rotate.transpose())
        else:
            # Create the bivariate gaussian matrix
            self.mu = mu
            self.covariance = cov

    def ellipsify(self, num = 50, multiplier = 3):
        # Default multiplier is 2 because that should cover 95% of errors
        a, b, phi = self.extractErrorElipseParamsFromBivariateGaussian()
        #print("a (ellipsify) ", str(a))
        #print("b (ellipsify)", str(b))
        #print("phi (ellipsify) ", str(math.degrees(phi)))
        ellipse = []
        pointEvery = math.radians(360/num)
        for count in range(num + 1):
            cur_angle = pointEvery * count
            range_val = self.calculateRadiusAtAngle(a, b, phi, cur_angle) * multiplier
            x_val = self.mu[0] + range_val * math.cos(cur_angle)
            y_val = self.mu[1] + range_val * math.sin(cur_angle)
            ellipse.append([x_val,y_val])

        return ellipse

    def dot(self):

        ellipse = []

        ellipse.append([self.mu[0], self.mu[1]])
        ellipse.append([self.mu[0], self.mu[1]+.1])
        ellipse.append([self.mu[0], self.mu[1]-.1])
        ellipse.append([self.mu[0], self.mu[1]])
        ellipse.append([self.mu[0]+.1, self.mu[1]])
        ellipse.append([self.mu[0]-.1, self.mu[1]])
        ellipse.append([self.mu[0], self.mu[1]])

        return ellipse

    def calculateRadiusAtAngle(self, a, b, phi, measurementAngle):
        denominator = math.sqrt( a**2 * math.sin(phi-measurementAngle)**2 + b**2 * math.cos(phi-measurementAngle)**2 ) / 2
        if denominator == 0:
            print ( "Warning: calculateEllipseRadius denom 0! - check localizer definitions " )
            #print ( a, b, phi, measurementAngle )
            return 0
        else:
            return ( a * b ) / math.sqrt( a**2 * math.sin(phi-measurementAngle)**2 + b**2 * math.cos(phi-measurementAngle)**2 ) / 2

    def calcSelfRadiusAtAnlge(self, angle):
        a, b, phi = self.extractErrorElipseParamsFromBivariateGaussian()
        return self.calculateRadiusAtAngle(a, b, phi, angle)

    def extractErrorElipseParamsFromBivariateGaussian(self):
        # Eigenvalue and eigenvector computations
        w, v = np.linalg.eig(self.covariance)

        # Use the eigenvalue to figure out which direction is larger
        if abs(w[0]) > abs(w[1]):
            a = abs(w[0])
            b = abs(w[1])
            phi = math.atan2(v[0, 0], v[1, 0])
        else:
            a = abs(w[1])
            b = abs(w[0])
            phi = math.atan2(v[0, 1], v[1, 1])

        return a, b, phi

def addBivariateGaussians(gaussianA, gaussianB, disregardMu = True):
    # Now we have both distributions, it is time to do the combination math
    # Source: http://www.geostatisticslessons.com/lessons/errorellipses
    covariance_new = np.add(gaussianA.covariance.transpose(), gaussianB.covariance.transpose()).transpose()

    # If our center is at 0,0 so we can skip the harder math, but it is here if needed
    if disregardMu:
        # Assume we are centered at 0,0, we simply use the fist mu only
        mu_new = gaussianA.mu
    else:
        mu_new = np.matmul(covariance_new, np.add(np.matmul(gaussianA.covariance.transpose(), gaussianA.mu), np.matmul(gaussianB.covariance.transpose(), gaussianB.mu)))

    gaussianC = BivariateGaussian(0,0,0,mu_new, covariance_new)

    return gaussianC

def subtractBivariateGaussians(gaussianA, gaussianB, disregardMu = True):
    # Now we have both distributions, it is time to do the combination math
    # Source: http://www.geostatisticslessons.com/lessons/errorellipses
    covariance_new = np.subtract(gaussianA.covariance.transpose(), gaussianB.covariance.transpose()).transpose()

    # If our center is at 0,0 so we can skip the harder math, but it is here if needed
    if disregardMu:
        # Assume we are centered at 0,0
        mu_new = gaussianA.mu
    else:
        mu_new = np.matmul(covariance_new, np.subtract(np.matmul(gaussianA.covariance.transpose(), gaussianA.mu), np.matmul(gaussianB.covariance.transpose(), gaussianB.mu)))

    gaussianC = BivariateGaussian(0,0,0,mu_new, covariance_new)

    return gaussianC

class SensorPackage:
    def __init__(self, name, localizer, localizerName):
        self.sensorList = []
        self.LOCALIZER_VLP64 = 0
        self.LOCALIZER_STATIC = 100
        self.localizer = localizer
        self.localizerName = localizerName
        self.longestSensorRange = 0
        self.minSensRange = .1
        self.name = name
        self.errorMagnitudeRadians = 0.0
        self.errorMagnitudeDistance = 0.0
        self.sensorsOff = 0
        
        # Localization Error
        self.locStep = -1
        self.locErrorX_actual = 0
        self.locErrorY_actual = 0
        self.locationGaussian = None
        
        # Stored detections
        self.detectionsList = []
        self.detectionsListSearcher = []
        
    def addCamera(self, name, centerAngle, maxRange, horizontalFOV, horizontalPixels, distanceError_expected, velocityError_expected = -1, iouRateM = 0, iouRateDistanceM = 0):
        container = Sensor(name, 3, centerAngle, maxRange, horizontalFOV, 0, horizontalPixels, distanceError_expected, velocityError_expected, iouRateM, iouRateDistanceM)
        self.sensorList.append(container)
        if maxRange > self.longestSensorRange:
            self.longestSensorRange = maxRange
        
    def addLIDAR(self, name, centerAngle, maxRange, horizontalFOV, horizontalResolution, distanceError_expected, velocityError_expected = -1, iouRateM = 0, iouRateDistanceM = 0):
        container = Sensor(name, 4, centerAngle, maxRange, horizontalFOV, horizontalResolution, 0, distanceError_expected, velocityError_expected, iouRateM, iouRateDistanceM)
        self.sensorList.append(container)
        if maxRange > self.longestSensorRange:
            self.longestSensorRange = maxRange
        
    def addRadar(self, name, centerAngle, maxRange, horizontalFOV, horizontalResolution, distanceError_expected, velocityError_expected = .1, iouRateM = 0, iouRateDistanceM = 0):
        container = Sensor(name, 5, centerAngle, maxRange, horizontalFOV, horizontalResolution, 0, distanceError_expected, velocityError_expected, iouRateM, iouRateDistanceM)
        self.sensorList.append(container)
        if maxRange > self.longestSensorRange:
            self.longestSensorRange = maxRange
        
    def addLocalizer(self, name, type):
        self.localizer = type
        self.localizerName = name
    
    def generate_localization_error_vlp64(self):
        #return .072, .005 * np.random.choice(161, 2, p=[0.00006467319641,0.00007165746337,0.00007938326216,0.00008792756864,0.00009737510329,0.0001078190752,0.0001193619922,0.0001321165424,0.0001462065521,0.0001617680267,0.0001789502787,0.0001979171506,0.0002188483375,0.0002419408164,0.0002674103883,0.0002954933392,0.0003264482261,0.0003605577945,0.000398131031,0.0004395053582,0.0004850489731,0.0005351633341,0.0005902857956,0.0006508923912,0.0007175007611,0.0007906732182,0.0008710199432,0.0009592022939,0.00105593621,0.001161995685,0.001278216276,0.001405498606,0.001544811794,0.001697196772,0.001863769373,0.002045723117,0.002244331555,0.002460950026,0.002697016662,0.002954052407,0.003233659821,0.003537520344,0.003867389695,0.00422509096,0.004612504893,0.005031556843,0.005484199643,0.005972391641,0.006498068982,0.007063111024,0.007669297658,0.008318257054,0.009011402132,0.009749853795,0.01053434863,0.01136512843,0.01224180848,0.01316322107,0.01412723013,0.01513051233,0.0161682991,0.01723407347,0.01831921435,0.01941258009,0.02050002175,0.02156381504,0.02258199853,0.02352760343,0.02436775868,0.02506265202,0.02556432553,0.02070710368,0.01636116834,0.01252651951,0.00920315719,0.006391081382,0.004090292085,0.002300789298,0.001022573021,0.0002556432553,0,0.0002556432553,0.001022573021,0.002300789298,0.004090292085,0.006391081382,0.00920315719,0.01252651951,0.01636116834,0.02070710368,0.02556432553,0.02506265202,0.02436775868,0.02352760343,0.02258199853,0.02156381504,0.02050002175,0.01941258009,0.01831921435,0.01723407347,0.0161682991,0.01513051233,0.01412723013,0.01316322107,0.01224180848,0.01136512843,0.01053434863,0.009749853795,0.009011402132,0.008318257054,0.007669297658,0.007063111024,0.006498068982,0.005972391641,0.005484199643,0.005031556843,0.004612504893,0.00422509096,0.003867389695,0.003537520344,0.003233659821,0.002954052407,0.002697016662,0.002460950026,0.002244331555,0.002045723117,0.001863769373,0.001697196772,0.001544811794,0.001405498606,0.001278216276,0.001161995685,0.00105593621,0.0009592022939,0.0008710199432,0.0007906732182,0.0007175007611,0.0006508923912,0.0005902857956,0.0005351633341,0.0004850489731,0.0004395053582,0.000398131031,0.0003605577945,0.0003264482261,0.0002954933392,0.0002674103883,0.0002419408164,0.0002188483375,0.0001979171506,0.0001789502787,0.0001617680267,0.0001462065521,0.0001321165424,0.0001193619922,0.0001078190752,0.00009737510329,0.00008792756864,0.00007938326216,0.00007165746337,0.00006467319641]) - .4
        return [.175,.075], [np.random.normal(0, .175, 1)[0], np.random.normal(0, .075, 1)[0]]
     
    def generate_localization_error_static(self):
        #return .072, .005 * np.random.choice(161, 2, p=[0.00006467319641,0.00007165746337,0.00007938326216,0.00008792756864,0.00009737510329,0.0001078190752,0.0001193619922,0.0001321165424,0.0001462065521,0.0001617680267,0.0001789502787,0.0001979171506,0.0002188483375,0.0002419408164,0.0002674103883,0.0002954933392,0.0003264482261,0.0003605577945,0.000398131031,0.0004395053582,0.0004850489731,0.0005351633341,0.0005902857956,0.0006508923912,0.0007175007611,0.0007906732182,0.0008710199432,0.0009592022939,0.00105593621,0.001161995685,0.001278216276,0.001405498606,0.001544811794,0.001697196772,0.001863769373,0.002045723117,0.002244331555,0.002460950026,0.002697016662,0.002954052407,0.003233659821,0.003537520344,0.003867389695,0.00422509096,0.004612504893,0.005031556843,0.005484199643,0.005972391641,0.006498068982,0.007063111024,0.007669297658,0.008318257054,0.009011402132,0.009749853795,0.01053434863,0.01136512843,0.01224180848,0.01316322107,0.01412723013,0.01513051233,0.0161682991,0.01723407347,0.01831921435,0.01941258009,0.02050002175,0.02156381504,0.02258199853,0.02352760343,0.02436775868,0.02506265202,0.02556432553,0.02070710368,0.01636116834,0.01252651951,0.00920315719,0.006391081382,0.004090292085,0.002300789298,0.001022573021,0.0002556432553,0,0.0002556432553,0.001022573021,0.002300789298,0.004090292085,0.006391081382,0.00920315719,0.01252651951,0.01636116834,0.02070710368,0.02556432553,0.02506265202,0.02436775868,0.02352760343,0.02258199853,0.02156381504,0.02050002175,0.01941258009,0.01831921435,0.01723407347,0.0161682991,0.01513051233,0.01412723013,0.01316322107,0.01224180848,0.01136512843,0.01053434863,0.009749853795,0.009011402132,0.008318257054,0.007669297658,0.007063111024,0.006498068982,0.005972391641,0.005484199643,0.005031556843,0.004612504893,0.00422509096,0.003867389695,0.003537520344,0.003233659821,0.002954052407,0.002697016662,0.002460950026,0.002244331555,0.002045723117,0.001863769373,0.001697196772,0.001544811794,0.001405498606,0.001278216276,0.001161995685,0.00105593621,0.0009592022939,0.0008710199432,0.0007906732182,0.0007175007611,0.0006508923912,0.0005902857956,0.0005351633341,0.0004850489731,0.0004395053582,0.000398131031,0.0003605577945,0.0003264482261,0.0002954933392,0.0002674103883,0.0002419408164,0.0002188483375,0.0001979171506,0.0001789502787,0.0001617680267,0.0001462065521,0.0001321165424,0.0001193619922,0.0001078190752,0.00009737510329,0.00008792756864,0.00007938326216,0.00007165746337,0.00006467319641]) - .4
        return [.01,.01], np.random.normal(0, .01, 2)
        
    def generate_locatization_error_frame(self, travelAngle, x_pos, y_pos, step):
        # Only generate this once per frame
        if self.locStep != step:
            # Get the localizer error
            if self.localizer == self.LOCALIZER_VLP64:
                expectedLocalizationError, actualLocalizationError = self.generate_localization_error_vlp64()
            elif self.localizer == self.LOCALIZER_STATIC:
                expectedLocalizationError, actualLocalizationError = self.generate_localization_error_static()
            else:
                expectedLocalizationError, actualLocalizationError = [0,0], [0,0]

            if expectedLocalizationError[0] >= expectedLocalizationError[1]:
                self.locationGaussian = BivariateGaussian(expectedLocalizationError[0], expectedLocalizationError[1],
                                                          travelAngle + math.radians(90))
                locErrorA_actual = actualLocalizationError[0]
                locErrorB_actual = actualLocalizationError[1]
                locErrorTheta = travelAngle
            else:
                self.locationGaussian = BivariateGaussian(expectedLocalizationError[0], expectedLocalizationError[1],
                                                          travelAngle)
                locErrorA_actual = actualLocalizationError[1]
                locErrorB_actual = actualLocalizationError[0]
                locErrorTheta = travelAngle + math.radians(90)

            self.locErrorX_actual = (locErrorA_actual * math.cos(locErrorTheta)) - (locErrorB_actual * math.sin(locErrorTheta))
            self.locErrorY_actual = (locErrorA_actual * math.sin(locErrorTheta)) - (locErrorB_actual * math.cos(locErrorTheta))

            # Add in our actual measured position here so we have it everywhere we need it
            self.locationGaussian.mu = [x_pos, y_pos]

            self.locStep = step

    def calculateEllipseRadius(self, a, b, phi, measurementAngle):
        denominator = math.sqrt( a**2 * math.sin(phi-measurementAngle)**2 + b**2 * math.cos(phi-measurementAngle)**2 ) / 2
        if denominator == 0:
            print ( "Warning: calculateEllipseRadius denom 0! - check localizer definitions " )
            #print ( a, b, phi, measurementAngle )
            return 0
        else:
            return ( a * b ) / math.sqrt( a**2 * math.sin(phi-measurementAngle)**2 + b**2 * math.cos(phi-measurementAngle)**2 ) / 2

    def addTrack(self, vehId):
        """Adds a new tracked vehicle to the list, calls the sub class to insert the vehicle to the field."""
        # Create a new AV
        new = TrackedObject(vehId)
        if len(self.detectionsListSearcher) == 0:
            # If the list is empty bisect is going to return an error
            self.detectionsListSearcher.append(vehId)
            self.detectionsList.append(new)
            return 0
        else:
            # Use the bisect method to sort in place, we use the Searcher list to keep the class list in order
            position = bisect.bisect_left(self.detectionsListSearcher, vehId)
            # Insert the AV to the list
            self.detectionsListSearcher.insert(position,vehId)
            self.detectionsList.insert(position,new)
            return position
        
    def getTrackIndex(self, vehId):
        vehIdIndex = binarySearch(self.detectionsListSearcher, vehId)
        if vehIdIndex >= 0:
            return vehIdIndex
        else:
            # If we fall through the loop to here, add the AV and respond accordingly
            return self.addTrack(vehId)
            
    def removeTrack(self, vehId):
        """Checks is AV object is in the list. If it is present it is deleted and returns true, otherwise returns false"""
        vehIdIndex = binarySearch(self.detectionsListSearcher, vehId)
        if vehIdIndex >= 0:
            self.detectionsListSearcher.pop(vehIdIndex)
            self.detectionsList.pop(vehIdIndex)
            return True
        else:
            # We did not find the AV object with that ID so return false.
            return False

    def cleanupTracks(self, step):
        removeList = []
        for each in self.detectionsList:
            if each.last_step != step:
                removeList.append(each.vehId)
                
        for remove in removeList:
            self.removeTrack(remove)

    def sensor_selection(self, vehId, x_detect, y_detect, x_sens, y_sens, sensor_angle, horizontalCrossSection):
        trackIndex = self.getTrackIndex(vehId)

        sensorRadialError_expected = 99
        radialSensorOfChoice = ""
        sensorVelocityError_expected = 99
        velocitySensorOfChoice = ""
        sensorDistanceError_expected = 99
        distanceSensorOfChoice = ""

        delta_x = x_detect - x_sens
        delta_y = y_detect - y_sens
        target_angle = math.atan2(delta_y, delta_x)

        #print("ta:" + str(target_angle))

        distance = math.hypot(delta_x, delta_y)
        sensor_angle = math.radians(sensor_angle)
        relative_angle = relativeAngleDiff(target_angle, sensor_angle)

        #print(relative_angle)
        #print(distance)

        sensorCheck = []

        # Iterate through our sensor package and select the lowest error rate sensor
        for sensor in self.sensorList:
            # distance < maxRange and angle < centerAngle + horizontalFOV/2 and angle > centerAngle - horizontalFOV/2
            print ( distance, sensor.maxRange, sensor.centerAngle, sensor.horizontalFOV )
            if sensor.checkInRangeAndFOV(relative_angle, distance):
                # (2*distance*ATAN(RADIANS(horizontalResolution/horizontalFOV))
                # print ( "sensorRadialError_expected = " , sensorRadialError_expected, " new sensor = " , sensor.calculateRadialError(distance) )
                checkRadialError = sensor.getRadialErrorAtDistance(distance, horizontalCrossSection)
                if sensorRadialError_expected > checkRadialError:
                    sensorRadialError_expected = checkRadialError
                    radialSensorOfChoice = sensor.name
                # print ( "sensorVelocityError_expected = " , sensorVelocityError_expected, " new sensor = " , sensor[7] )
                checkDistanceError = sensor.getDistanceErrorAtDistance(distance, horizontalCrossSection)
                if sensorDistanceError_expected > checkDistanceError:
                    sensorDistanceError_expected = checkDistanceError
                    velocitySensorOfChoice = sensor.name
                # print ( "sensorDistanceError_expected = " , sensorDistanceError_expected, " new sensor = " , sensor[6] )
                if (sensor.velocityError_expected != -1) and (sensorVelocityError_expected > sensor.velocityError_expected):
                    sensorVelocityError_expected = sensor.velocityError_expected
                    distanceSensorOfChoice = sensor.name
                    # Calculate our expected errors in x,y coordinates
                errorX_expected = ((distance + checkDistanceError) * math.cos(
                    target_angle + checkRadialError)) - delta_x
                errorY_expected = ((distance + checkDistanceError) * math.sin(
                    target_angle + checkRadialError)) - delta_y
                radialErrorMeters = 2 * (distance * math.sin(checkRadialError / 2))
                sensorCheck.append([sensor.name, errorX_expected, errorY_expected, checkDistanceError, radialErrorMeters])
                print ( sensor.name, errorX_expected, errorY_expected, checkDistanceError, radialErrorMeters )

        # Error check here to make sure we have a sensor with the correct angle
        if sensorRadialError_expected == -99 or sensorDistanceError_expected == -99:
            print("radialSensorOfChoice ", self.name, relative_angle, distance)
            return False

        # Calculate our expected elipse error bounds
        detectionErrorElipseA_expected = 2 * (distance * math.sin(sensorRadialError_expected / 2))
        detectionErrorElipseB_expected = sensorDistanceError_expected
        if detectionErrorElipseA_expected < detectionErrorElipseB_expected:
            detectionErrorElipseAngle_expected = target_angle
        else:
            detectionErrorElipseAngle_expected = target_angle + math.radians(90)

        # We need to create the real error to get our measurement values
        errorX_expected = []
        errorY_expected = []
        #print ( "sensorRadialError_expected, sensorDistanceError_expected", sensorRadialError_expected, sensorDistanceError_expected )
        for idx in range(10000):
            actualRadialError = np.random.normal(0, sensorRadialError_expected, 1)[0]
            actualDistanceError = np.random.normal(0, sensorDistanceError_expected, 1)[0]
            errorX_expected.append(((distance + actualDistanceError) * math.cos(
                target_angle + actualRadialError)) - delta_x)
            errorY_expected.append(((distance + actualDistanceError) * math.sin(
                target_angle + actualRadialError)) - delta_y)
            #print(idx, actualRadialError, actualDistanceError, errorX_expected[idx], errorY_expected[idx])

        return sensorCheck, errorX_expected, errorY_expected, detectionErrorElipseA_expected, detectionErrorElipseB_expected, detectionErrorElipseAngle_expected

    def calculateAnglesForFusion(self, x_detect, y_detect, x_sens, y_sens, sensor_angle):
        delta_x = x_detect - x_sens
        delta_y = y_detect - y_sens
        target_angle = math.atan2(delta_y, delta_x)
        distance = math.hypot(delta_x, delta_y)
        # We need to do some funny conversions here to account for the coordinate system of SUMO
        relative_angle = target_angle - sensor_angle
        return delta_x, delta_y, target_angle, distance, relative_angle

    def selectSensorsFromSet(self, relative_angle, distance, horizontalCrossSection):
        sensorRadialError_expected = 99
        radialSensorOfChoice = ""
        sensorVelocityError_expected = 99
        velocitySensorOfChoice = ""
        sensorDistanceError_expected = 99
        distanceSensorOfChoice = ""

        singleSensorMagnitudeRadians = 0.0
        singleSensorMagnitudeDistance = 0.0

        # print(" Detected from sensor angle: " + str(math.degrees(sensor_angle)))
        # print(" Detected from target angle: " + str(math.degrees(target_angle)))
        # print(" Detected from relative angle: " + str(math.degrees(relative_angle)))

        # Iterate through our sensor package and select the lowest error rate sensor
        for sensor in self.sensorList:
            # distance < maxRange and angle < centerAngle + horizontalFOV/2 and angle > centerAngle - horizontalFOV/2
            # print(distance, sensor.maxRange, sensor.centerAngle, sensor.horizontalFOV)
            if sensor.checkInRangeAndFOV(relative_angle, distance):
                # (2*distance*ATAN(RADIANS(horizontalResolution/horizontalFOV))
                # print ( "sensorRadialError_expected = " , sensorRadialError_expected, " new sensor = " , sensor.calculateRadialError(distance) )
                checkRadialError = sensor.getRadialErrorAtDistance(distance, horizontalCrossSection)
                if sensorRadialError_expected > checkRadialError:
                    sensorRadialError_expected = checkRadialError
                    radialSensorOfChoice = sensor.name
                    singleSensorMagnitudeRadians = sensor.errorMagnitudeRadians
                # print ( "sensorVelocityError_expected = " , sensorVelocityError_expected, " new sensor = " , sensor[7] )
                checkDistanceError = sensor.getDistanceErrorAtDistance(distance, horizontalCrossSection)
                if sensorDistanceError_expected > checkDistanceError:
                    sensorDistanceError_expected = checkDistanceError
                    singleSensorMagnitudeDistance = sensor.errorMagnitudeDistance
                    velocitySensorOfChoice = sensor.name
                # print ( "sensorDistanceError_expected = " , sensorDistanceError_expected, " new sensor = " , sensor[6] )
                if (sensor.velocityError_expected != -1) and (
                        sensorVelocityError_expected > sensor.velocityError_expected):
                    sensorVelocityError_expected = sensor.velocityError_expected
                    distanceSensorOfChoice = sensor.name
                    # Calculate our expected errors in x,y coordinates

        # Error check here to make sure we have a sensor with the correct angle
        if sensorRadialError_expected == -99 or sensorDistanceError_expected == -99:
            print("radialSensorOfChoice ", self.name, relative_angle, distance)
            return False, -1, -1, -1
        else:
            return True, sensorRadialError_expected, sensorDistanceError_expected, sensorVelocityError_expected, singleSensorMagnitudeRadians, singleSensorMagnitudeDistance

    def calculateDetectionErrorAtAngle(self, vehId, x_detect, y_detect, x_sens, y_sens, sensor_angle, horizontalCrossSection, step, simulationInjectErrors = True):
    
        try:

            # Get the correct track for this vehicle sensor fusion package
            trackIndex = self.getTrackIndex(vehId)

            # Build the object where we store all the accuracy stats
            trackAccuracy = TrackAccuracyObject()
            trackAccuracy.trackingId = vehId
            trackAccuracy.horizontalCrossSection = horizontalCrossSection

            # Calculate the necessary angles
            delta_x, delta_y, target_angle, trackAccuracy.detectionDistance, relative_angle = self.calculateAnglesForFusion(x_detect, y_detect, x_sens, y_sens, sensor_angle)
        
            # Now we need to select the best sensor candidates for sensor fusion
            valid, sensorRadialError_expected, sensorDistanceError_expected, sensorVelocityError_expected, singleSensorMagnitudeRadians, singleSensorMagnitudeDistance = self.selectSensorsFromSet(relative_angle, trackAccuracy.detectionDistance, horizontalCrossSection)

            # Sanity check, this should never hit
            if not valid:
                return

            # This statement determines if these are real inputs with error already attached, or
            # if we should generate the error according to the params because this is a simulation
            if simulationInjectErrors:
                # Always inject the normal error since that is enabled by the outer if
                # We need to create the real error to get our measurement values
                # Also we will add in the overall error here that wasnt added on a per sensor basis
                actualRadialError = np.random.normal(0, sensorRadialError_expected, 1)[0] + self.errorMagnitudeRadians + singleSensorMagnitudeRadians
                actualDistanceError = np.random.normal(0, sensorDistanceError_expected, 1)[0] + self.errorMagnitudeDistance + singleSensorMagnitudeDistance

                # Make sure we have generated error for this frame, this should be called by the exterior loop but this checks anywys
                if self.locStep != step:
                    self.generate_locatization_error_frame(sensor_angle, x_sens, y_sens, step)

                # Check if we chould make a detection error
                if 'veh' in vehId:
                    checkId = float(vehId.replace('veh', ''))
                else:
                    checkId = 1
                if False: #checkId%28 == 0:
                    # Make a vehicle sensor error
                    # Arbitrarily shift by 3 degrees for now
                    trackAccuracy.errorX_actual = ((trackAccuracy.detectionDistance + actualDistanceError) * math.cos(
                        target_angle + actualRadialError + math.radians(3.0))) - delta_x
                    trackAccuracy.errorY_actual = ((trackAccuracy.detectionDistance + actualDistanceError) * math.sin(
                        target_angle + actualRadialError + math.radians(3.0))) - delta_y
                else:
                    # Calculate our actual errors in x,y coordinates
                    trackAccuracy.errorX_actual = ((trackAccuracy.detectionDistance + actualDistanceError) * math.cos(target_angle + actualRadialError)) - delta_x
                    trackAccuracy.errorY_actual = ((trackAccuracy.detectionDistance + actualDistanceError) * math.sin(target_angle + actualRadialError)) - delta_y
                # Add in localization error
                trackAccuracy.errorX_actual += self.locErrorX_actual
                trackAccuracy.errorY_actual += self.locErrorY_actual
            else:
                # Error is already present so no need to calculate
                trackAccuracy.errorX_actual = 0
                trackAccuracy.errorY_actual = 0

            # For debugging
            # if (abs(trackAccuracy.errorX_actual) > 3 ) or (abs(trackAccuracy.errorY_actual) > 3 ):
            #     print ( "distance ", distance, " actualDistanceError ", actualDistanceError )
            #     print ( "  target_angle ", target_angle, " actualRadialError ", actualRadialError, " delta_x ", delta_x )
            #     print ( "  x_detect ", x_detect, " x_sens ", x_sens, " y_detect ", y_detect, " y_sens ", y_sens )
            #     print ( "  actualDistanceError ", actualDistanceError, " actualRadialError " , actualRadialError )
            #     print ( "  x ", ((distance + actualDistanceError) * math.cos(target_angle + actualRadialError)), " x_l " , delta_x )
            #     print ( "  y ", ((distance + actualDistanceError) * math.sin(target_angle + actualRadialError)), " y_l " , delta_y )
            #     print ( "  trackAccuracy.errorX_actual ", trackAccuracy.errorX_actual, " trackAccuracy.errorY_actual " , trackAccuracy.errorY_actual )
            #     print ( "  self.locErrorX_actual ", self.locErrorX_actual, " self.locErrorY_actual " , self.locErrorY_actual )
            #print ( "Target Angle: " , str(math.degrees(target_angle)))

            # Calculate our expected elipse error bounds
            detectionErrorElipseWidth_expected = 2 * (trackAccuracy.detectionDistance * math.sin(sensorRadialError_expected / 2))
            detectionErrorElipseHeight_expected = sensorDistanceError_expected / 2
            detectionErrorElipseAngle_expected = target_angle

            # We need to still worry about conventions of A and B, the birvariate matrix is rotated during creation
            if detectionErrorElipseWidth_expected >= detectionErrorElipseHeight_expected:
                sensingGaussian = BivariateGaussian(detectionErrorElipseWidth_expected,
                                                    detectionErrorElipseHeight_expected,
                                                    detectionErrorElipseAngle_expected)
            else:
                sensingGaussian = BivariateGaussian(detectionErrorElipseHeight_expected,
                                                    detectionErrorElipseWidth_expected,
                                                    detectionErrorElipseAngle_expected + math.radians(90))

            # Store the last error gaussian if we had one
            #if self.detectionsList[trackIndex].noStoredData == False and ((step - self.detectionsList[trackIndex].last_step) == 1):
            #    trackAccuracy.expectedErrorGaussian

            # Now we will add the 2 elipse expected errors of the detection and localization
            # errors to create the overall error range
            trackAccuracy.expectedErrorGaussian = addBivariateGaussians(self.locationGaussian, sensingGaussian)

            # Now we can calculate the theta and velocity error
            #trackAccuracy.expectedAngleErrorGaussian = (trackAccuracy.expectedErrorGaussian, trackAccuracy.expectedErrorGaussian)
            trackAccuracy.errorAngle_expected = 0
            trackAccuracy.errorVelocity_expected = 0

            # Time to calculate actual errors
            if self.detectionsList[trackIndex].noStoredData == False and ((step - self.detectionsList[trackIndex].last_step) == 1):
                # Actual velocity - measured
                x_detect_e = x_detect + trackAccuracy.errorX_actual
                x_detect_l_e = self.detectionsList[trackIndex].x_detect_last + self.detectionsList[trackIndex].xErrorActual_last
                y_detect_e = y_detect + trackAccuracy.errorY_actual
                y_detect_l_e = self.detectionsList[trackIndex].y_detect_last + self.detectionsList[trackIndex].yErrorActual_last
                trackAccuracy.errorVelocity_actual = math.hypot(x_detect_e - x_detect_l_e, y_detect_e - y_detect_l_e) - math.hypot(x_detect - self.detectionsList[trackIndex].x_detect_last, y_detect - self.detectionsList[trackIndex].y_detect_last)
                # Actual angle - measured
                trackAccuracy.errorAngle_actual = math.atan2(y_detect_e - y_detect_l_e,x_detect_e - x_detect_l_e) - math.atan2(y_detect - self.detectionsList[trackIndex].y_detect_last, x_detect - self.detectionsList[trackIndex].x_detect_last)
            else:
                trackAccuracy.errorAngle_actual = 0
                trackAccuracy.errorVelocity_actual = 0
            
            # # print ("Iteration: ", step )
            # # print (" Expected: Radial error, velocity error, distance error: ", sensorRadialError_expected, sensorVelocityError_expected, sensorDistanceError_expected )
            # # print (" Actual: Radial error, velocity error, distance error: ", actualRadialError, self.velocityErrorActual, actualDistanceError )
            # # print (" errorRectangleWidth, errorRectangleHeight, rectangle alpha: ", errorRectangleWidth, errorRectangleHeight, alpha )   
            # # print (" Expected: ", self.xErrorExpected, self.yErrorExpected, self.thetaErrorExpected, self.velocityErrorExpected )
            # # print (" Actual: ", self.xError_actual, self.yError_actual, self.thetaErrorActual, self.velocityErrorActual )
            
            # Store out x,y values for next iteration
            self.detectionsList[trackIndex].x_detect_last = x_detect
            self.detectionsList[trackIndex].y_detect_last = y_detect
            self.detectionsList[trackIndex].noStoredData = False
            self.detectionsList[trackIndex].last_step = step
            # Make sure our measured position is the mu
            trackAccuracy.expectedErrorGaussian.mu = [x_detect + trackAccuracy.errorX_actual, y_detect + trackAccuracy.errorY_actual]
            self.detectionsList[trackIndex].lastExpectedErrorGaussian = trackAccuracy.expectedErrorGaussian
            
            # if (abs(trackAccuracy.errorX_actual) > 3 ) or (abs(trackAccuracy.errorY_actual) > 3 ):
            #     print ( "  " , trackAccuracy.errorX_actual, trackAccuracy.errorY_actual )
        
        except Exception as e:
            print( "Error: calculateDetectionErrorAtAngle " , str(e)) 

        #print ( " returning" )
        
        return True, trackAccuracy

  
# Test add sensors  
# sensors = SensorPackage()
# sensors.addLIDAR("VLP64E", 0, 100, 360, .16, .02, -1)
# sensors.addCamera("Cam1", 0, 100, 21, 1920, 1, -1)
# sensors.addRadar("Radar1LR", 0, 250, 18, .1, .4, .1)
# sensors.addRadar("Radar1SRN", 0, 20, 90, 1, .1, .1)
# sensors.addRadar("Radar1SRW", 0, 20, 120, 5, .1, .1)


# Test set 1
# output = []
# for count in range(10000):
    # # (step, x_detect, y_detect, x_sens, y_sens, sensor_angle, x_sens_loc_err, y_sens_loc_err
    # print ( 
    # returned = sensors.calculateDetectionErrorAtAngle(count, 10, 1, 0, 0, 0, 0, 0)
    # if returned[0]:
        # output.append([returned[1],returned[2],returned[3],returned[4]])

# x_val = [x[0] for x in output]
# y_val = [x[1] for x in output]
# x_std = statistics.stdev(x_val)
# y_std = statistics.stdev(y_val)
# t_val = [x[2] for x in output]
# v_val = [x[3] for x in output]
# t_std = statistics.stdev(t_val)
# v_std = statistics.stdev(v_val)

# print ( "expected: ", returned[5], returned[6], returned[7], returned[8], returned[9])
# print ( "actual: ", x_std, y_std, t_std, v_std)

# plt.figure(1)
# plt.plot(x_val,y_val,'or', markersize=.1)
# plt.plot(x_std,y_std,'om')
# plt.plot(x_std,-y_std,'om')
# plt.plot(-x_std,-y_std,'om')
# plt.plot(-x_std,y_std,'om')
# # For whatever reason this doesnt print right unless you multiply the a and b by 2
# # Also we must change angle back to degrees before sending
# ellipse = Ellipse(xy=(0, 0), width=2*returned[5], height=2*returned[6], angle=math.degrees(returned[7]), edgecolor='c', fc='None', lw=4)
# ax = plt.gca()
# ax.add_patch(ellipse)
# plt.show()

# plt.figure(2)
# plt.plot(t_val,'or', markersize=.5)
# #plt.plot(v_val,'or', markersize=.5)
# plt.plot(returned[8],'ob')
# #plt.plot(returned[7],'om')
# plt.plot(500,t_std,'om')
# #plt.plot(500,v_std,'+m')
# plt.show()

# plt.figure(3)
# plt.plot(v_val,'or', markersize=.5)
# plt.plot(returned[9],'ob')
# plt.plot(500,v_std,'om')
# plt.show()

# Test set 2
# output2 = []
# for circleCount in range(0,360):
    # print ( circleCount )
    # output = []
    # for count in range(100):
        # # (step, x_detect, y_detect, x_sens, y_sens, sensor_angle, x_sens_loc_err, y_sens_loc_err
        # returned = sensors.calculateDetectionErrorAtAngle((circleCount*10)+count, 25*math.cos(math.radians(circleCount)), 25*math.sin(math.radians(circleCount)), 0, 0, 0, 0, 0)
        # if returned[0]:
            # output.append([returned[1],returned[2],returned[3],returned[4]])

    # x_val = [x[0] for x in output]
    # y_val = [y[1] for y in output]
    # x_std = statistics.stdev(x_val)
    # y_std = statistics.stdev(y_val)
    # t_val = [t[2] for t in output]
    # v_val = [v[3] for v in output]
    # t_std = statistics.stdev(t_val)
    # v_std = statistics.stdev(v_val)
    # output2.append([circleCount,x_std,y_std,t_std,v_std,returned[5], returned[6], returned[8], returned[9]])
    
    # #output2.append([circleCount,returned[1],returned[2],returned[3],returned[4],returned[5], returned[6], returned[8], returned[9]])

# #print ( "expected: ", returned[5], returned[6], returned[7], returned[8])
# #print ( "actual: ", x_std, y_std, t_std, v_std)

# cir_val = [x[0] for x in output2]
# exp_x_val = [x[5] for x in output2]
# x_val = [x[1] for x in output2]

# plt.figure(1)
# plt.plot(cir_val,exp_x_val,'r', markersize=.5)
# plt.plot(cir_val,x_val,'m')

# plt.show()

# cir_val = [x[0] for x in output2]
# exp_x_val = [x[6] for x in output2]
# x_val = [x[2] for x in output2]

# plt.figure(2)
# plt.plot(cir_val,exp_x_val,'r', markersize=.5)
# plt.plot(cir_val,x_val,'m')

# plt.show()

# cir_val = [x[0] for x in output2]
# exp_x_val = [x[7] for x in output2]
# x_val = [x[3] for x in output2]

# plt.figure(3)
# plt.plot(cir_val,exp_x_val,'r', markersize=.5)
# plt.plot(cir_val,x_val,'m')

# plt.show()

# cir_val = [x[0] for x in output2]
# exp_x_val = [x[8] for x in output2]
# x_val = [x[4] for x in output2]

# plt.figure(4)
# plt.plot(cir_val,exp_x_val,'r', markersize=.5)
# plt.plot(cir_val,x_val,'m')

# plt.show()