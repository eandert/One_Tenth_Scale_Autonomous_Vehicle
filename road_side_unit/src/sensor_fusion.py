import numpy as np
np.set_printoptions(threshold=3)
np.set_printoptions(suppress=True)

import math
import operator

method_noMethod = 0
method_mostAccurateSingleDetection = 1
method_averageOfAllDetections = 2
method_wieghtedAverageOfAllDetections = 3
method_dxdyKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections = 4
method_dxdyKalmanFilterBestThreeSensors = 5
method_dvdthetaKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections = 6
method_dxdyKalmanFilterBestFiveSensors = 9
method_dxdyKalmanFilterBestTwoSensorsWeightedAfter = 8
method_dxdyKalmanFilterBestSensorAndAverage = 7
method_dvdthetaKalmanFilterOfAllDetections = 10
method_dxdyKalmanFilterSingleSensor_OnMostAccurateDetection = 11

# This list corresponds to the above definitions and gives them human readable names for printing
fusion_names = ["Individual vehicle sensing",
                "Most Accurate Single Detection",
                "Average",
                "Weighted Average",
                "Kalman dxdy on Weighted Average",
                "Kalman dxdy on Three Most Accurate",
                "Kalman dvdtheta on Weighted Average",
                "Kalman dxdy on Most Accurate and Average",
                "Kalman dxdy on 2 Most Accurate, Weighted Average After",
                "Kalman dxdy on Five Most Accurate",
                "Kalman dxdy on 2 Most Accurate, Weighted Average After",
                "Kalman dvdtheta on All Detections - Not working"]
                
fusion_storage = [[0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]],
                    [0,[],[]]]

class TrackAccuracyObject:
    def __init__(self):
        self.errorX_actual = 0
        self.errorY_actual = 0
        self.errorX_expected = 0
        self.errorY_expected = 0
        self.errorA_expected = 0
        self.errorB_expected = 0
        self.errorPhi_expected = 0
        self.errorAngle_actual = 0
        self.errorVelocity_actual = 0
        self.errorAngle_expected = 0
        self.errorVelocity_expected = 0
        self.detectionDistance = 0

class ErrorPacket:
    def __init__(self):
        self.vehID = None
        self.x_actual = 0 
        self.y_actual = 0
        self.a_actual = 0 
        self.v_actual = 0
        self.errorPos_actual = 0
        self.errorX_actual = 0
        self.errorY_actual = 0
        self.errorA_actual = 0
        self.errorV_actual = 0
        self.errorPos_expected = 0
        self.errorA_expected = 0
        self.errorB_expected = 0
        self.errorPhi_expected = 0
        self.errorV_expected = 0
        self.errorX_expected = 0
        self.errorY_expected = 0
        self.numTracking = 0
        self.method = 0

class SensorFusion:
    def __init__(self, method):
        # Define methods of sensor fusion
    
        # Variables used by all methods
        self.acceleration = 0
        self.delta_t = .1 #second
        self.lastUpdate = -99
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.theta = 0
        self.timeOfLife = 0
        self.timeOfLifeInherited = 0
        self.method = method
        
        # Define variables specific to certain methods
        if self.method == method_dxdyKalmanFilterSingleSensor_OnMostAccurateDetection:
            #Transition matrix
            self.F_t=np.array([ [1,0,self.delta_t,0], [0,1,0,self.delta_t], [0,0,1,0], [0,0,0,1] ])
            #Initial State cov
            self.P_t= np.identity(4)
            #Process cov
            self.Q_t= np.identity(4)
            #Control matrix
            self.B_t=np.array( [ [0] , [0] , [0] , [0] ])
            #Control vector
            self.U_t=self.acceleration
            #Measurment Matrix
            self.H_t = np.array([[1, 0, 0, 0], [ 0, 1, 0, 0]])
            #Measurment cov
            self.R_t= np.identity(2)
        elif self.method == method_dxdyKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections:
            #Transition matrix
            self.F_t=np.array([ [1 ,0,self.delta_t,0] , [0,1,0,self.delta_t] , [0,0,1,0] , [0,0,0,1] ])
            #Initial State cov
            self.P_t= np.identity(4)
            #Process cov
            self.Q_t= np.identity(4)
            #Control matrix
            self.B_t=np.array( [ [0] , [0] , [0] , [0] ])
            #Control vector
            self.U_t=self.acceleration
            #Measurment Matrix
            self.H_t = np.array([[1, 0, 0, 0], [ 0, 1, 0, 0]])
            #Measurment cov
            self.R_t= np.identity(2)
        elif self.method == method_dvdthetaKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections:
            #Transition matrix
            self.F_t_warmedUp = False
            #Initial State cov
            self.P_t= np.identity(6)
            #Process cov
            self.Q_t= np.identity(6)
            #Control matrix
            self.B_t=np.array( [ [0] , [0] , [0] , [0] , [0] , [0] ])
            #Control vector
            self.U_t=self.acceleration
            #Measurment Matrix
            self.H_t = np.array([ [1, 0, 0, 0, 0, 0], 
                             [0, 1, 0, 0, 0, 0], 
                             [0, 0, 1, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0], 
                             [0, 0, 0, 0, 1, 0], 
                             [0, 0, 0, 0, 0, 0]])
            #Measurment cov
            self.R_t= np.identity(6)
        elif self.method == method_dxdyKalmanFilterBestThreeSensors:
            #Transition matrix
            self.F_t=np.array([ [1,0,0,0,0,0,self.delta_t,0]
                    , [0,1,0,0,0,0,0,self.delta_t]
                    , [0,0,1,0,0,0,self.delta_t,0]
                    , [0,0,0,1,0,0,0,self.delta_t]
                    , [0,0,0,0,1,0,self.delta_t,0]
                    , [0,0,0,0,0,1,0,self.delta_t]
                    , [0,0,0,0,0,0,1,0]
                    , [0,0,0,0,0,0,0,1] ])
            # if fusion_storage[self.method][0] >= 5:
            #     # If there is enough learned, lets use their learnings when we start up
            #     self.timeOfLifeInherited = fusion_storage[self.method][0]
            #     self.P_t = fusion_storage[self.method][1]
            #     self.Q_t = fusion_storage[self.method][2]
            # else:
            #Initial State cov
            self.P_t= np.identity(8)
            #Process cov
            self.Q_t= np.identity(8)
            # End if it not commented
            #Control matrix
            self.B_t=np.array( [ [0] , [0] , [0] , [0] , [0] , [0], [0] , [0] ])
            #Control vector
            self.U_t=self.acceleration
            #Measurment Matrix
            self.H_t = np.array([[1, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0]])
            self.H_t_2 = np.array([[1, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0]])
            self.H_t_3 = np.array([[1, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 1, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 1, 0, 0]])
            #Measurment cov
            self.R_t= np.identity(6)
            pass
        elif self.method == method_dxdyKalmanFilterBestSensorAndAverage:
            #Transition matrix
            self.F_t=np.array([ [1,0,0,0,self.delta_t,0]
                    , [0,1,0,0,0,self.delta_t]
                    , [0,0,1,0,self.delta_t,0]
                    , [0,0,0,1,0,self.delta_t]
                    , [0,0,0,0,1,0]
                    , [0,0,0,0,0,1] ])
            # Initial State cov
            self.P_t = np.identity(6)
            # Process cov
            self.Q_t = np.identity(6)
            # End if it not commented
            #Control matrix
            self.B_t=np.array( [ [0] , [0] , [0] , [0] , [0] , [0] ])
            #Control vector
            self.U_t=self.acceleration
            #Measurment Matrix
            self.H_t = np.array([[1, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0]])
            #Measurment cov
            self.R_t= np.identity(4)
            pass
        elif self.method == method_dxdyKalmanFilterBestFiveSensors:
            #Transition matrix
            self.F_t=np.array([ [1,0,0,0,0,0,0,0,0,0,self.delta_t,0]
                    , [0,1,0,0,0,0,0,0,0,0,0,self.delta_t]
                    , [0,0,1,0,0,0,0,0,0,0,self.delta_t,0]
                    , [0,0,0,1,0,0,0,0,0,0,0,self.delta_t]
                    , [0,0,0,0,1,0,0,0,0,0,self.delta_t,0]
                    , [0,0,0,0,0,1,0,0,0,0,0,self.delta_t]
                    , [0,0,0,0,0,0,1,0,0,0,self.delta_t,0]
                    , [0,0,0,0,0,0,0,1,0,0,0,self.delta_t]
                    , [0,0,0,0,0,0,0,0,1,0,self.delta_t,0]
                    , [0,0,0,0,0,0,0,0,0,1,0,self.delta_t]
                    , [0,0,0,0,0,0,0,0,0,0,1,0]
                    , [0,0,0,0,0,0,0,0,0,0,0,1] ])
            if fusion_storage[self.method][0] >= 5:
                # If there is enough learned, lets use their learnings when we start up
                self.timeOfLifeInherited = fusion_storage[self.method][0]
                self.P_t = fusion_storage[self.method][1]
                self.Q_t = fusion_storage[self.method][2]
            else:
                #Initial State cov
                self.P_t= np.identity(12)
                #Process cov
                self.Q_t= np.identity(12)
            #Control matrix
            self.B_t=np.array( [ [0] , [0] , [0] , [0] , [0] , [0], [0] , [0], [0] , [0], [0] , [0] ])
            #Control vector
            self.U_t=self.acceleration
            #Measurment Matrix
            self.H_t = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
            self.H_t_2 = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
            self.H_t_3 = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
            self.H_t_4 = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
            self.H_t_5 = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
                                , [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]])
            #Measurment cov
            self.R_t= np.identity(10)
            pass
        elif self.method == method_dxdyKalmanFilterBestTwoSensorsWeightedAfter:
            #Transition matrix
            self.F_t=np.array([ [1,0,0,0,self.delta_t,0]
                    , [0,1,0,0,0,self.delta_t]
                    , [0,0,1,0,self.delta_t,0]
                    , [0,0,0,1,0,self.delta_t]
                    , [0,0,0,0,1,0]
                    , [0,0,0,0,0,1] ])
            if fusion_storage[self.method][0] >= 5:
                # If there is enough learned, lets use their learnings when we start up
                self.timeOfLifeInherited = fusion_storage[self.method][0]
                self.P_t = fusion_storage[self.method][1]
                self.Q_t = fusion_storage[self.method][2]
            else:
                #Initial State cov
                self.P_t= np.identity(6)
                #Process cov
                self.Q_t= np.identity(6)
            #Control matrix
            self.B_t=np.array( [ [0] , [0] , [0] , [0] , [0] , [0] ])
            #Control vector
            self.U_t=self.acceleration
            #Measurment Matrix
            self.H_t = np.array([[1, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0]])
            self.H_t_2 = np.array([[1, 0, 0, 0, 0, 0]
                                , [ 0, 1, 0, 0, 0, 0]
                                , [ 0, 0, 1, 0, 0, 0]
                                , [ 0, 0, 0, 1, 0, 0]])
            #Measurment cov
            self.R_t= np.identity(6)
            pass
        
            
    def runFusionOnDataset(self, dataset, x, y, a, v, step):
        try:
            if self.method == method_mostAccurateSingleDetection or self.method == method_noMethod:
                return self.mostAccurateSingleDetection(dataset, x, y, a, v, step)
            elif self.method == method_averageOfAllDetections:
                return self.averageOfAllDetections(dataset, x, y, a, v, step)
            elif self.method == method_wieghtedAverageOfAllDetections:
                return self.wieghtedAverageOfAllDetections(dataset, x, y, a, v, step)
            elif self.method == method_dxdyKalmanFilterSingleSensor_OnMostAccurateDetection:
                return self.dxdyKalmanFilterSingleSensor_OnMostAccurateDetection(dataset, x, y, a, v, step)
            elif self.method == method_dxdyKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections:
                return self.dxdyKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections(dataset, x, y, a, v, step)
            elif self.method == method_dvdthetaKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections:
                return self.dvdthetaKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections(dataset, x, y, a, v, step)
            elif self.method == method_dxdyKalmanFilterBestThreeSensors:
                return self.dxdyKalmanFilterBestThreeSensors(dataset, x, y, a, v, step)
            elif self.method == method_dxdyKalmanFilterBestSensorAndAverage:
                return self.dxdyKalmanFilterBestSensorAndAverage(dataset, x, y, a, v, step)
            elif self.method == method_dxdyKalmanFilterBestFiveSensors:
                return self.dxdyKalmanFilterBestFiveSensors(dataset, x, y, a, v, step)
            elif self.method == method_dxdyKalmanFilterBestTwoSensorsWeightedAfter:
                return self.dxdyKalmanFilterBestTwoSensorsWeightedAfter(dataset, x, y, a, v, step)
            else:
                return ErrorPacket()
        except Exception as e:
            print( "Error runFusionOnDataset: " , str(e))
            
    def mostAccurateSingleDetection(self, dataset, x, y, a, v, step):
        errorPacket = ErrorPacket()
        errorPacket.numTracking = len(dataset)
        rmse = None
        for observation in dataset:
            a, b, phi = observation.expectedErrorGaussian.extractErrorElipseParamsFromBivariateGaussian()
            rmse_distance_current = math.hypot(a,b)
            if (rmse == None) or (rmse_distance_current < rmse):
                rmse = rmse_distance_current
                errorPacket.errorX_actual = observation.errorX_actual
                errorPacket.errorY_actual = observation.errorY_actual
                errorPacket.errorA_actual = observation.errorAngle_actual
                errorPacket.errorV_actual = observation.errorVelocity_actual
                errorPacket.errorA_expected = a
                errorPacket.errorB_expected = b
                errorPacket.errorPhi_expected = observation.errorAngle_expected
                errorPacket.errorV_expected = observation.errorVelocity_expected
                errorPacket.errorX_expected = observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(0)
                errorPacket.errorY_expected = observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(90)

        errorPacket.method = self.method
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorA_expected, errorPacket.errorB_expected)
        return errorPacket

    def averageOfAllDetections(self, dataset, x, y, a, v, step):
        errorPacket = ErrorPacket()
        errorPacket.numTracking = len(dataset)
        for observation in dataset:
            errorPacket.errorX_actual += observation.errorX_actual
            errorPacket.errorY_actual += observation.errorY_actual
            errorPacket.errorA_actual += observation.errorAngle_actual
            errorPacket.errorV_actual += observation.errorVelocity_actual
            # We can get the x compnent by finding the error gaussian radius at 0, same for y at 90
            errorPacket.errorX_expected += observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(0)
            errorPacket.errorY_expected += observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(90)
            #errorPacket.errorA_expected += observation.errorAngle_expected
            #errorPacket.errorV_expected += observation.errorVelocity_expected
        
        errorPacket.errorX_actual = errorPacket.errorX_actual / errorPacket.numTracking
        errorPacket.errorY_actual = errorPacket.errorY_actual / errorPacket.numTracking
        errorPacket.errorA_actual = errorPacket.errorA_actual / errorPacket.numTracking
        errorPacket.errorV_actual = errorPacket.errorV_actual / errorPacket.numTracking
        errorPacket.errorX_expected = errorPacket.errorX_expected / errorPacket.numTracking
        errorPacket.errorY_expected = errorPacket.errorY_expected / errorPacket.numTracking
        #errorPacket.errorA_expected = errorPacket.errorA_expected / errorPacket.numTracking
        #errorPacket.errorV_expected = errorPacket.errorV_expected / errorPacket.numTracking
        
        errorPacket.method = self.method
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorA_expected, errorPacket.errorB_expected)
        return errorPacket
        
    def wieghtedAverageOfAllDetections(self, dataset, x, y, a, v, step):
        errorPacket = ErrorPacket()
        totalWeight = 0
        errorPacket.numTracking = len(dataset)
        if errorPacket.numTracking >= 1:
            for observation in dataset:
                a, b, phi = observation.expectedErrorGaussian.extractErrorElipseParamsFromBivariateGaussian()
                distance_current =  1.0/math.hypot(a,b)
                totalWeight = totalWeight + distance_current
                errorPacket.errorX_actual += observation.errorX_actual * distance_current
                errorPacket.errorY_actual += observation.errorY_actual * distance_current
                errorPacket.errorA_actual += observation.errorAngle_actual * distance_current
                errorPacket.errorV_actual += observation.errorVelocity_actual * distance_current
                # We can get the x compnent by finding the error gaussian radius at 0, same for y at 90
                errorPacket.errorX_expected += observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(0) * distance_current
                errorPacket.errorY_expected += observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(90) * distance_current
                #errorPacket.errorA_expected += observation.errorAngle_expected * distance_current
                #errorPacket.errorV_expected += observation.errorVelocity_expected * distance_current
            
            errorPacket.errorX_actual = errorPacket.errorX_actual / totalWeight
            errorPacket.errorY_actual = errorPacket.errorY_actual / totalWeight
            errorPacket.errorA_actual = errorPacket.errorA_actual / totalWeight
            errorPacket.errorV_actual = errorPacket.errorV_actual / totalWeight
            errorPacket.errorX_expected = errorPacket.errorX_expected / totalWeight
            errorPacket.errorY_expected = errorPacket.errorY_expected / totalWeight
            #errorPacket.errorA_expected = errorPacket.errorA_expected / totalWeight
            #errorPacket.errorV_expected = errorPacket.errorV_expected / totalWeight
            
        errorPacket.method = self.method
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorA_expected, errorPacket.errorB_expected)
        return errorPacket

    def wieghtedAverageOfAllDetectionsWCovariance(self, dataset, x, y, a, v, step):
        errorPacket = ErrorPacket()
        totalWeight = 0
        errorPacket.numTracking = len(dataset)
        expectedCov = dataset[0].expectedErrorGaussian
        if errorPacket.numTracking >= 1:
            for idx, observation in enumerate(dataset):
                a, b, phi = observation.expectedErrorGaussian.extractErrorElipseParamsFromBivariateGaussian()
                distance_current = 1.0 / math.hypot(a, b)
                totalWeight = totalWeight + distance_current
                errorPacket.errorX_actual += observation.errorX_actual * distance_current
                errorPacket.errorY_actual += observation.errorY_actual * distance_current
                errorPacket.errorA_actual += observation.errorAngle_actual * distance_current
                errorPacket.errorV_actual += observation.errorVelocity_actual * distance_current
                # We can get the x compnent by finding the error gaussian radius at 0, same for y at 90
                errorPacket.errorX_expected += observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(
                    0) * distance_current
                errorPacket.errorY_expected += observation.expectedErrorGaussian.calcSelfRadiusAtAnlge(
                    90) * distance_current
                # errorPacket.errorA_expected += observation.errorAngle_expected * distance_current
                # errorPacket.errorV_expected += observation.errorVelocity_expected * distance_current
                # Calculate the sensor fusion expected from these gaussians
                if idx > 0:
                    expectedCov.covariance = np.add(expectedCov.covariance, observation.expectedErrorGaussian.covariance)

            errorPacket.errorX_actual = errorPacket.errorX_actual / totalWeight
            errorPacket.errorY_actual = errorPacket.errorY_actual / totalWeight
            errorPacket.errorA_actual = errorPacket.errorA_actual / totalWeight
            errorPacket.errorV_actual = errorPacket.errorV_actual / totalWeight
            errorPacket.errorX_expected = errorPacket.errorX_expected / totalWeight
            errorPacket.errorY_expected = errorPacket.errorY_expected / totalWeight
            # errorPacket.errorA_expected = errorPacket.errorA_expected / totalWeight
            # errorPacket.errorV_expected = errorPacket.errorV_expected / totalWeight
            expectedCov.covariance = expectedCov.covariance/4

        errorPacket.method = self.method
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorA_expected, errorPacket.errorB_expected)
        return errorPacket, expectedCov

    def dxdyKalmanFilterSingleSensor_OnMostAccurateDetection(self, dataset, x, y, a, v, step):
        errorPacket = self.mostAccurateSingleDetection(dataset, x, y, a, v, step)
        
        # Now we have the most accurate detection, do the kalman thing!
        if self.lastUpdate != (step-1):
            # We have no prior detection so we need to just ouput what we have but store for later
            self.X_hat_t = np.array( [[x + errorPacket.errorX_actual],[y + errorPacket.errorY_actual],[0],[0]] )
            self.lastUpdate = step
        else:
            # We have valid data
            X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
            measure_with_error = np.array( [x + errorPacket.errorX_actual,y + errorPacket.errorY_actual] )
            self.R_t = np.array([[covariance[0][0], covariance[0][1]],
                                [covariance[1][0], covariance[1][1]]])
            Z_t=(measure_with_error).transpose()
            Z_t=Z_t.reshape(Z_t.shape[0],-1)
            X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,self.H_t)
            self.X_hat_t=X_t
            self.P_hat_t=self.P_t
            errorPacket.errorX_actual = X_t[0][0] - x
            errorPacket.errorY_actual = X_t[1][0] - y
            self.lastUpdate = step
            self.timeOfLife = self.timeOfLife + 1

        errorPacket.numTracking = len(dataset)
        errorPacket.method = self.method
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorX_expected, errorPacket.errorY_expected)
        return errorPacket
        
    def dxdyKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections(self, dataset, x, y, a, v, step):
        errorPacket, expectedCov = self.wieghtedAverageOfAllDetectionsWCovariance(dataset, x, y, a, v, step)
        
        # Now we have the most accurate detection, do the kalman thing!
        if errorPacket.numTracking > 0:
            if self.lastUpdate != (step-1):
                # We have no prior detection so we need to just ouput what we have but store for later
                self.X_hat_t = np.array( [[x + errorPacket.errorX_actual],[y + errorPacket.errorY_actual],[0],[0]] )
                self.lastUpdate = step
            else:
                # We have valid data
                X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
                measure_with_error = np.array( [x + errorPacket.errorX_actual,y + errorPacket.errorY_actual] )
                self.R_t = np.array([[expectedCov.covariance[0][0], expectedCov.covariance[0][1]],
                                     [expectedCov.covariance[1][0], expectedCov.covariance[1][1]]])
                Z_t=(measure_with_error).transpose()
                Z_t=Z_t.reshape(Z_t.shape[0],-1)
                X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,self.H_t)
                self.X_hat_t=X_t
                self.P_hat_t=self.P_t
                errorPacket.errorX_actual = X_t[0][0] - x
                errorPacket.errorY_actual = X_t[1][0] - y
                self.lastUpdate = step
                self.timeOfLife = self.timeOfLife + 1

        errorPacket.method = self.method
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorX_expected, errorPacket.errorY_expected)
        return errorPacket
        
    def dxdyKalmanFilterBestThreeSensors(self, dataset, x, y, a, v, step):
        # Sort by distance
        errorPacket = ErrorPacket()
        errorPacket.numTracking = len(dataset)
        dataset2 = []
        try:
            if errorPacket.numTracking >= 1:
                for observation in dataset:
                    a, b, phi = observation.expectedErrorGaussian.extractErrorElipseParamsFromBivariateGaussian()
                    distance_current = math.hypot(a,b)

                    packet = []
                    packet.append(observation.errorX_actual)
                    packet.append(observation.errorY_actual)
                    packet.append(observation.errorAngle_actual)
                    packet.append(observation.errorVelocity_actual)
                    packet.append(observation.expectedErrorGaussian)
                    packet.append(observation.errorAngle_expected)
                    packet.append(observation.errorVelocity_expected)
                    packet.append(distance_current)
                    dataset2.append(packet)

                dataset2 = sorted(dataset2, key=operator.itemgetter(7))

                # Now we have the most accurate detection, do the kalman thing!
                if self.lastUpdate != (step-1):
                    # We have no prior detection so we need to just ouput what we have but store for later
                    self.X_hat_t = np.array( [[x + dataset2[0][0]],[y + dataset2[0][1]],[0],[0],[0],[0],[0],[0]] )
                    self.lastUpdate = step
                else:
                    # We have valid data
                    X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
                    if errorPacket.numTracking > 2:
                        tempeH_t = self.H_t_3
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + dataset2[1][0],y + dataset2[1][1],x + dataset2[2][0],y + dataset2[2][1]] )
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4].covariance[0][0], dataset2[0][4].covariance[0][1], 0, 0, 0, 0],
                                             [dataset2[0][4].covariance[1][0], dataset2[0][4].covariance[1][1], 0, 0, 0, 0],
                                             [0, 0, dataset2[1][4].covariance[0][0], dataset2[1][4].covariance[0][1], 0, 0],
                                             [0, 0, dataset2[1][4].covariance[1][0], dataset2[1][4].covariance[1][1], 0, 0],
                                             [0, 0, 0, 0, dataset2[2][4].covariance[0][0], dataset2[2][4].covariance[0][1]],
                                             [0, 0, 0, 0, dataset2[2][4].covariance[1][0], dataset2[2][4].covariance[1][1]]])
                    elif errorPacket.numTracking > 1:
                        tempeH_t = self.H_t_2
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + dataset2[1][0],y + dataset2[1][1],0,0] )
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4].covariance[0][0], dataset2[0][4].covariance[0][1], 0, 0, 0, 0],
                                             [dataset2[0][4].covariance[1][0], dataset2[0][4].covariance[1][1], 0, 0, 0, 0],
                                             [0, 0, dataset2[1][4].covariance[0][0], dataset2[1][4].covariance[0][1], 0, 0],
                                             [0, 0, dataset2[1][4].covariance[1][0], dataset2[1][4].covariance[1][1], 0, 0],
                                             [0, 0, 0, 0, 1, 0],
                                             [0, 0, 0, 0, 0, 1]])
                    else:
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],0,0,0,0] )
                        tempeH_t = self.H_t
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4].covariance[0][0], dataset2[0][4].covariance[0][1], 0., 0., 0., 0.],
                                             [dataset2[0][4].covariance[1][0], dataset2[0][4].covariance[1][1], 0., 0., 0., 0.],
                                             [0., 0., 1., 0., 0., 0.],
                                             [0., 0., 0., 1., 0., 0.],
                                             [0., 0., 0., 0., 1., 0.],
                                             [0., 0., 0., 0., 0., 1.]])
                    Z_t=(measure_with_error).transpose()
                    Z_t=Z_t.reshape(Z_t.shape[0],-1)
                    X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,tempeH_t)
                    self.X_hat_t=X_t
                    self.P_hat_t=self.P_t
                    errorPacket.errorX_actual = X_t[0][0] - x
                    errorPacket.errorY_actual = X_t[1][0] - y
                    self.lastUpdate = step
                    self.timeOfLife = self.timeOfLife + 1

                errorPacket.method = self.method
        except Exception as e:
            print( " Error ", str(e))

        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorX_expected, errorPacket.errorY_expected)
        return errorPacket

    def dxdyKalmanFilterBestSensorAndAverage(self, dataset, x, y, a, v, step):
        # Sort by distance
        errorPacket = ErrorPacket()
        errorPacket.numTracking = len(dataset)
        dataset2 = []
        try:
            if errorPacket.numTracking >= 1:
                for observation in dataset:
                    a, b, phi = observation.expectedErrorGaussian.extractErrorElipseParamsFromBivariateGaussian()
                    distance_current = math.hypot(a,b)

                    packet = []
                    packet.append(observation.errorX_actual)
                    packet.append(observation.errorY_actual)
                    packet.append(observation.errorAngle_actual)
                    packet.append(observation.errorVelocity_actual)
                    packet.append(observation.expectedErrorGaussian)
                    packet.append(observation.errorAngle_expected)
                    packet.append(observation.errorVelocity_expected)
                    packet.append(distance_current)
                    dataset2.append(packet)

                dataset2 = sorted(dataset2, key=operator.itemgetter(7))

                errorPacket2, expectedCov = self.wieghtedAverageOfAllDetectionsWCovariance(dataset, x, y, a, v, step)

                # Now we have the most accurate detection, do the kalman thing!
                if self.lastUpdate != (step-1):
                    # We have no prior detection so we need to just ouput what we have but store for later
                    self.X_hat_t = np.array( [[x + dataset2[0][0]],[y + dataset2[0][1]],[x + errorPacket2.errorX_actual],[y + errorPacket2.errorY_actual],[0],[0]] )
                    self.lastUpdate = step
                else:
                    # We have valid data
                    X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
                    measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + errorPacket2.errorX_actual,y + errorPacket2.errorY_actual] )
                    #Measurment cov
                    self.R_t = np.array([[dataset2[0][4].covariance[0][0], dataset2[0][4].covariance[0][1], 0, 0],
                                         [dataset2[0][4].covariance[1][0], dataset2[0][4].covariance[1][1], 0, 0],
                                         [0, 0, expectedCov.covariance[0][0], expectedCov.covariance[0][1]],
                                         [0, 0, expectedCov.covariance[1][0], expectedCov.covariance[1][1]]])
                    Z_t=(measure_with_error).transpose()
                    Z_t=Z_t.reshape(Z_t.shape[0],-1)
                    X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,self.H_t)
                    self.X_hat_t=X_t
                    self.P_hat_t=self.P_t
                    errorPacket.errorX_actual = X_t[0][0] - x
                    errorPacket.errorY_actual = X_t[1][0] - y
                    self.lastUpdate = step
                    self.timeOfLife = self.timeOfLife + 1

                errorPacket.method = self.method
        except Exception as e:
            print( " Error ", str(e))

        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorX_expected, errorPacket.errorY_expected)
        return errorPacket
        
    def dxdyKalmanFilterBestFiveSensors(self, dataset, x, y, a, v, step):
        # Sort by distance
        errorPacket = ErrorPacket()
        errorPacket.numTracking = len(dataset)
        dataset2 = []
        try:
            if errorPacket.numTracking >= 1:
                for observation in dataset:
                    rmse_distance_current = math.sqrt(observation.errorX_expected**2 + observation.errorY_expected**2)
                    
                    packet = []
                    packet.append(observation.errorX_actual)
                    packet.append(observation.errorY_actual)
                    packet.append(observation.errorAngle_actual)
                    packet.append(observation.errorVelocity_actual)
                    packet.append(observation.errorX_expected)
                    packet.append(observation.errorY_expected)
                    packet.append(observation.errorAngle_expected)
                    packet.append(observation.errorVelocity_expected)
                    packet.append(rmse_distance_current)
                    dataset2.append(packet)

                dataset2 = sorted(dataset2, key=operator.itemgetter(8))

                # Now we have the most accurate detection, do the kalman thing!
                if self.lastUpdate != (step-1):
                    # We have no prior detection so we need to just ouput what we have but store for later
                    self.X_hat_t = np.array( [[x + dataset2[0][0]],[y + dataset2[0][1]],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]] )
                    self.lastUpdate = step
                else:
                    # We have valid data
                    X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
                    if errorPacket.numTracking > 4:
                        tempeH_t = self.H_t_5
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + dataset2[1][0],y + dataset2[1][1],x + dataset2[2][0],y + dataset2[2][1],x + dataset2[3][0],y + dataset2[3][1],x + dataset2[4][0],y + dataset2[4][1]] )
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4], 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, dataset2[0][5], 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, 0, dataset2[1][4], 0, 0, 0, 0, 0, 0, 0],
                                             [0, 0, 0, dataset2[1][5], 0, 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, dataset2[2][4], 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, 0, dataset2[2][5], 0, 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., dataset2[3][4], 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, dataset2[3][5], 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, dataset2[4][4], 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, 0, dataset2[4][5]]])
                    elif errorPacket.numTracking > 3:
                        tempeH_t = self.H_t_4
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + dataset2[1][0],y + dataset2[1][1],x + dataset2[2][0],y + dataset2[2][1],x + dataset2[3][0],y + dataset2[3][1],0,0] )
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4], 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, dataset2[0][5], 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, 0, dataset2[1][4], 0, 0, 0, 0, 0, 0, 0],
                                             [0, 0, 0, dataset2[1][5], 0, 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, dataset2[2][4], 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, 0, dataset2[2][5], 0, 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., dataset2[3][4], 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, dataset2[3][5], 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, 1, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, 0, 1]])
                    elif errorPacket.numTracking > 2:
                        tempeH_t = self.H_t_3
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + dataset2[1][0],y + dataset2[1][1],x + dataset2[2][0],y + dataset2[2][1],0,0,0,0] )
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4], 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, dataset2[0][5], 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, 0, dataset2[1][4], 0, 0, 0, 0, 0, 0, 0],
                                             [0, 0, 0, dataset2[1][5], 0, 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, dataset2[2][4], 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, 0, dataset2[2][5], 0, 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 1, 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 1, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, 1, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, 0, 1]])
                    elif errorPacket.numTracking > 1:
                        tempeH_t = self.H_t_2
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + dataset2[1][0],y + dataset2[1][1],0,0,0,0,0,0] )
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4], 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, dataset2[0][5], 0, 0, 0, 0, 0, 0, 0, 0], 
                                             [0, 0, dataset2[1][4], 0, 0, 0, 0, 0, 0, 0],
                                             [0, 0, 0, dataset2[1][5], 0, 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
                                             [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                             [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                             [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
                    else:
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],0,0,0,0,0,0,0,0] )
                        tempeH_t = self.H_t
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4], 0., 0., 0., 0., 0., 0, 0, 0, 0], 
                                             [0., dataset2[0][5], 0., 0., 0., 0., 0, 0, 0, 0], 
                                             [0., 0., 1., 0., 0., 0., 0, 0, 0, 0],
                                             [0., 0., 0., 1., 0., 0., 0, 0, 0, 0], 
                                             [0., 0., 0., 0., 1., 0., 0, 0, 0, 0], 
                                             [0., 0., 0., 0., 0., 1., 0, 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 1, 0, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 1, 0, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, 1, 0],
                                             [0., 0., 0., 0., 0., 0., 0, 0, 0, 1]])
                    Z_t=(measure_with_error).transpose()
                    Z_t=Z_t.reshape(Z_t.shape[0],-1)
                    X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,tempeH_t)
                    self.X_hat_t=X_t
                    self.P_hat_t=self.P_t
                    errorPacket.errorX_actual = X_t[0][0] - x
                    errorPacket.errorY_actual = X_t[1][0] - y
                    self.lastUpdate = step
                    self.timeOfLife = self.timeOfLife + 1
                    if fusion_storage[self.method][0] < self.timeOfLife:
                        # We are the oldest kalman filter, lets load our values into storage
                        fusion_storage[self.method][1] = self.P_t
                        fusion_storage[self.method][2] = self.Q_t
                        fusion_storage[self.method][0] = self.timeOfLife
                        #print ( fusion_storage[self.method] )
                errorPacket.method = self.method
        except Exception as e:
            print( " Error ", str(e))     
            
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorX_expected, errorPacket.errorY_expected)
        return errorPacket
        
    def dvdthetaKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections(self, dataset, x, y, a, v, step):
        #print ( "   1" )
        errorPacket, expectedCov = self.wieghtedAverageOfAllDetectionsWCovariance(dataset, x, y, a, v, step)

        #print("   3")
        
        #print ( " self.lastUpdate " , self.lastUpdate, "step", step)
        # Now we have the most accurate detection, do the kalman thing!
        if self.lastUpdate != (step-1):
            # We have no prior detection so we need to just ouput what we have but store for later
            #print ( " 0_0 " )
            self.X_hat_t = np.array( [[x + errorPacket.errorX_actual],[y + errorPacket.errorY_actual],[a + errorPacket.errorA_actual],[0],[v + errorPacket.errorV_actual],[0]] )
            #print ( " 0_1 " )
            self.lastUpdate = step
            #print ( " 0_2 " )
            self.F_t_warmedUp = False
        else:
            #Measurment cov
            self.R_t = np.array([ [expectedCov.covariance[0][0],expectedCov.covariance[0][1],1,1,0,0] ,
                                   [expectedCov.covariance[1][0],expectedCov.covariance[1][1],1,1,0,0] ,
                                   [0,0,1,0,0,0] ,
                                   [0,0,0,1,0,0] ,
                                   [0,0,0,0,1,0] ,
                                   [0,0,0,0,0,1]])
            # We have valid data
            #print ( " 0" )
            if self.F_t_warmedUp:
                #self.F_t=np.array([ [1,0, - self.delta_t * self.X_t[4][0] * math.sin(self.X_t[2][0]), 0, self.delta_t * math.cos(self.X_t[2][0]), 0] ,
                #   [0,1, self.delta_t * self.X_t[4][0] * math.cos(self.X_t[2][0]), 0, self.delta_t * math.sin(self.X_t[2][0]), 0] ,
                self.F_t=np.array([ [1,0, self.delta_t * self.X_t[4][0] * math.cos(self.X_t[2][0]), 0, self.delta_t * math.cos(self.X_t[2][0]), 0] ,
                   [0,1, self.delta_t * self.X_t[4][0] * math.sin(self.X_t[2][0]), 0, self.delta_t * math.sin(self.X_t[2][0]), 0] ,
                   [0,0,1,self.delta_t,0,0] ,
                   [0,0,0,1,0,0] ,
                   [0,0,0,0,1,self.delta_t] ,
                   [0,0,0,0,0,1]])
            else:
                self.F_t=np.array([ [1,0,0,0,0,0],
                   [0,1,0,0,0,0],
                   [0,0,1,self.delta_t,0,0],
                   [0,0,0,1,0,0],
                   [0,0,0,0,1,self.delta_t],
                   [0,0,0,0,0,1] ])
            #print ( " 1" )
            #print ( self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t )
            X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
            #print ( " 2" )
            measure_with_error = np.array( [x + errorPacket.errorX_actual,y + errorPacket.errorY_actual,a + errorPacket.errorA_actual,0,v + errorPacket.errorV_actual,0] )
            #print ( " 3" )
            Z_t=(measure_with_error).transpose()
            #print ( " 4" )
            Z_t=Z_t.reshape(Z_t.shape[0],-1)
            #print ( " 5" )
            self.X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,self.H_t)
            #print ( " 6" )
            self.X_hat_t=self.X_t
            #print ( " 7" )
            self.P_hat_t=self.P_t
            #print ( self.X_t )
            errorPacket.errorX_actual = self.X_t[0][0] - x
            errorPacket.errorY_actual = self.X_t[1][0] - y
            #print ( math.degrees(self.X_t[2][0]), a )
            errorPacket.errorA_actual = math.degrees(self.X_t[2][0]) - a
            errorPacket.errorV_actual = self.X_t[4][0] - v
            self.lastUpdate = step
            self.F_t_warmedUp = True
            self.timeOfLife = self.timeOfLife + 1
            if fusion_storage[self.method][0] < self.timeOfLife:
                # We are the oldest kalman filter, lets load our values into storage
                fusion_storage[self.method][1] = self.P_t
                fusion_storage[self.method][2] = self.Q_t
                fusion_storage[self.method][0] = self.timeOfLife
                #print ( fusion_storage[self.method] )

        #print("   end")
        errorPacket.numTracking = len(dataset)
        errorPacket.method = self.method
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        #print ( self.method, errorPacket.errorPos_actual )
        #errorPacket.errorPos_expected = math.hypot(errorPacket.errorX_expected, errorPacket.errorY_expected)
        return errorPacket
        
    def dxdyKalmanFilterBestTwoSensorsWeightedAfter(self, dataset, x, y, a, v, step):
        # Sort by distance
        errorPacket = ErrorPacket()
        errorPacket.numTracking = len(dataset)
        dataset2 = []
        try:
            if errorPacket.numTracking >= 1:
                for observation in dataset:
                    distance_current = math.sqrt(observation.errorX_expected**2 + observation.errorY_expected**2)
                    
                    packet = []
                    packet.append(observation.errorX_actual)
                    packet.append(observation.errorY_actual)
                    packet.append(observation.errorAngle_actual)
                    packet.append(observation.errorVelocity_actual)
                    packet.append(observation.errorX_expected)
                    packet.append(observation.errorY_expected)
                    packet.append(observation.errorAngle_expected)
                    packet.append(observation.errorVelocity_expected)
                    packet.append(distance_current)
                    dataset2.append(packet)

                dataset2 = sorted(dataset2, key=operator.itemgetter(8))

                # Now we have the most accurate detection, do the kalman thing!
                if self.lastUpdate != (step-1):
                    # We have no prior detection so we need to just ouput what we have but store for later
                    self.X_hat_t = np.array( [[x + dataset2[0][0]],[y + dataset2[0][1]],[0],[0],[0],[0]] )
                    self.lastUpdate = step
                else:
                    # We have valid data
                    X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
                    if errorPacket.numTracking > 1:
                        tempeH_t = self.H_t_2
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],x + dataset2[1][0],y + dataset2[1][1]] )
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4], 0, 0, 0], 
                                             [0, dataset2[0][5], 0, 0], 
                                             [0, 0, dataset2[1][4], 0],
                                             [0, 0, 0, dataset2[1][5]]])
                    else:
                        measure_with_error = np.array( [x + dataset2[0][0],y + dataset2[0][1],0,0] )
                        tempeH_t = self.H_t
                        #Measurment cov
                        self.R_t = np.array([[dataset2[0][4], 0, 0, 0], 
                                             [0, dataset2[0][5], 0, 0], 
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]])
                    Z_t=(measure_with_error).transpose()
                    Z_t=Z_t.reshape(Z_t.shape[0],-1)
                    X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,tempeH_t)
                    self.X_hat_t=X_t
                    self.P_hat_t=self.P_t
                    # If we have more than 2 detections use a weighted average result
                    if errorPacket.numTracking > 2:
                        errorPacket = self.wieghtedAverageOfAllDetections(dataset, x, y, a, v, step)
                    else:
                        errorPacket.errorX_actual = X_t[0][0] - x
                        errorPacket.errorY_actual = X_t[1][0] - y
                    self.lastUpdate = step
                    self.timeOfLife = self.timeOfLife + 1
                    if fusion_storage[self.method][0] < self.timeOfLife:
                        # We are the oldest kalman filter, lets load our values into storage
                        fusion_storage[self.method][1] = self.P_t
                        fusion_storage[self.method][2] = self.Q_t
                        fusion_storage[self.method][0] = self.timeOfLife
                        #print ( fusion_storage[self.method] )
                errorPacket.method = self.method
        except Exception as e:
            print( " Error ", str(e))     
            
        errorPacket.errorPos_actual = math.hypot(errorPacket.errorX_actual, errorPacket.errorY_actual)
        errorPacket.errorPos_expected = math.hypot(errorPacket.errorX_expected, errorPacket.errorY_expected)
        return errorPacket
        
    # def dvdthetaKalmanFilterSingleSensor_OnWieghtedAverageOfAllDetections(self, dataset, x, y, a, v, step):
        # errorPacket = self.wieghtedAverageOfAllDetections(dataset, x, y, a, v, step)
        
        # # Now we have the most accurate detection, do the kalman thing!
        # if self.lastUpdate != step:
            # # We have no prior detection so we need to just ouput what we have but store for later
            # self.X_hat_t = np.array( [[x + errorPacket.errorX_actual],[y + errorPacket.errorY_actual],[0],[0]] )
            # self.lastUpdate = step
        # else:
            # # We have valid data, lettttsss gooooo
            # X_hat_t,self.P_hat_t = self.prediction(self.X_hat_t,self.P_t,self.F_t,self.B_t,self.U_t,self.Q_t)
            # measure_with_error = np.array( [[x + errorPacket.errorX_actual],[y + errorPacket.errorY_actual]] )
            # Z_t=(measure_with_error).transpose()
            # Z_t=Z_t.reshape(Z_t.shape[0],-1)
            # X_t,self.P_t=self.update(X_hat_t,self.P_hat_t,Z_t,self.R_t,self.H_t)
            # self.X_hat_t=X_t
            # self.P_hat_t=self.P_t
            # errorPacket.errorX_actual = X_t[0][0] - x
            # errorPacket.errorY_actual = X_t[1][1] - y

        # errorPacket.numTracking = len(dataset)
        # errorPacket.method = self.method
        # return errorPacket

    def prediction(self,X_hat_t_1,P_t_1,F_t,B_t,U_t,Q_t):
        X_hat_t=F_t.dot(X_hat_t_1)+(B_t.dot(U_t).reshape(B_t.shape[0],-1) )
        P_t=np.diag(np.diag(F_t.dot(P_t_1).dot(F_t.transpose())))+Q_t
        return X_hat_t,P_t
        

    def update(self,X_hat_t,P_t,Z_t,R_t,H_t):
        K_prime=P_t.dot(H_t.transpose()).dot( np.linalg.inv ( H_t.dot(P_t).dot(H_t.transpose()) + R_t ) )  
        #print("K:\n",K_prime)
        #print("X_hat:\n",X_hat_t)
        X_t=X_hat_t+K_prime.dot(Z_t-H_t.dot(X_hat_t))
        P_t=P_t-K_prime.dot(H_t).dot(P_t)
        
        return X_t,P_t

    def convert_coordinates(theta):
        if theta >= 0 and theta <= 90:
            return 90 - theta
        elif theta <= 270:
            return -(theta - 90)
        else:
            return (360+90) - theta
                
# #Observations: position_X, position_Y
# measurmens = genfromtxt('data/measurmens_real.csv', delimiter=',',skip_header=1)

# #Transition matrix
# F_t=np.array([ [1 ,0,delta_t,0] , [0,1,0,delta_t] , [0,0,1,0] , [0,0,0,1] ])

# #Initial State cov
# P_t= np.identity(4)*0.2

# #Process cov
# Q_t= np.identity(4)

# #Control matrix
# B_t=np.array( [ [0] , [0], [0] , [0] ])

# #Control vector
# U_t=acceleration

# #Measurment Matrix
# H_t = np.array([ [1, 0, 0, 0], [ 0, 1, 0, 0]])

# #Measurment cov
# R_t= np.identity(2)

# # Initial State
# X_hat_t = np.array( [[measurmens[0][0]],[measurmens[0][1]],[0],[0]] )
# print("X_hat_t",X_hat_t.shape)
# print("P_t",P_t.shape)
# print("F_t",F_t.shape)
# print("B_t",B_t.shape)
# print("Q_t",Q_t.shape)
# print("R_t",R_t.shape)
# print("H_t",H_t.shape)

# output = []
# measured_error = []
# measured_error_1 = []
# measured_error_2 = []

# for i in range(measurmens.shape[0]):
    # X_hat_t,P_hat_t = prediction(X_hat_t,P_t,F_t,B_t,U_t,Q_t)
    # print("Prediction:")
    # print("X_hat_t:\n",X_hat_t,"\nP_t:\n",P_t)
    
    # measure_with_error = np.add(measurmens[i],np.asarray(generate_localization_error(2)))
    # measure_with_error_1 = np.add(measurmens[i],np.asarray(generate_localization_error(2)))
    # measure_with_error_2 = np.add(measurmens[i],np.asarray(generate_localization_error(2)))
        
    # print ( " --------------- " , measurmens[i], " vs ",  measure_with_error )
    # measured_error.append(measure_with_error)
    # measured_error_1.append(measure_with_error_1)
    # measured_error_2.append(measure_with_error_2)
    
    # Z_t=(measure_with_error).transpose()
    # Z_t=Z_t.reshape(Z_t.shape[0],-1)
    
    # print(Z_t.shape)
    
    # X_t,P_t=update(X_hat_t,P_hat_t,Z_t,R_t,H_t)
    # print("Update:")
    # print("X_t:\n",X_t,"\nP_t:\n",P_t)
    # #output.append(X_t[0],X_t[1])
    # suboutput = [X_t[0][0],X_t[1][0]]
    # output.append(suboutput)
    # X_hat_t=X_t
    # P_hat_t=P_t

# print (measurmens)
# print ( " ----- " )
# print (output)

# output_0 = np.asarray(output)
# measured_error_0 = np.asarray(measured_error)
# measured_error_1 = np.asarray(measured_error_1)
# measured_error_2 = np.asarray(measured_error_2)