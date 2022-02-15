import time
import math
import sys
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import numpy as np
from sklearn.neighbors import NearestNeighbors
import threading

# For simulation
# from connected_autonomous_vehicle.src import lidar_recognition
# import local_fusion, sensor, shared_math
# import global_fusion
from gui.src import communication
from road_side_unit.src import mapGenerator
from shared_library import shared_math

# Defines the colors for various elements of the GUI
brush_color = {
    "vehicle": Qt.darkBlue,
    "camera": Qt.darkBlue,
    "camera_fov": Qt.lightGray,
    "traffic_light_green": Qt.green,
    "traffic_light_yellow": Qt.yellow,
    "traffic_light_red": Qt.red,
    "waypoint": Qt.darkGray,
    "target_point": Qt.darkRed,
    "wheel_angle": Qt.red,
    "bounding_box": Qt.gray,
    "lidar_detection_centroid": Qt.cyan,
    "lidar_detection_error": Qt.cyan,
    "lidar_detection_raw": Qt.lightGray,
    "camera_detection_centroid": Qt.darkYellow,
    "camera_detection_error": Qt.darkYellow,
    "sensor_fusion_centroid": Qt.red,
    "localization": Qt.yellow,
    "localization_error": Qt.yellow,
    "sensor_fusion_error_ellipse": Qt.green,
    "global_sensor_fusion_centroid": Qt.blue,
    "global_sensor_fusion_error_ellipse": Qt.darkGreen
}

class MainWindow(QMainWindow):
    def __init__(self, config):
        print ( " GUI Started ")

        self.rsu_connection = communication.connectServer(config.rsu_ip)
        init_response = self.rsu_connection.getGuiValues(True)

        self.mapSpecs = mapGenerator.MapSpecs(init_response['map_specs'][0], init_response['map_specs'][1])
        self.vehicles = init_response['vehicle']
        self.camera_fov = init_response['camera_fov']
        self.camera_center = init_response['camera_center']
        self.lidar_detection_centroid = init_response['lidar_detection_centroid']
        self.camera_detection_centroid = init_response['camera_detection_centroid']
        self.sensor_fusion_centroid = init_response['sensor_fusion_centroid']
        self.localization_error = init_response['localization_error']
        self.global_sensor_fusion_centroid = init_response['global_sensor_fusion_centroid']
        self.trafficLightArray = init_response['traffic_light']

        # self.vehicles = cav
        # self.cis = cis
        # self.trafficLightArray = trafficLightArray
        # self.pause_simulation = True
        # self.vehiclesLock = vehiclesLock

        # # Set to engage a full simulation world w/ no real vehicles and fake time
        # self.full_simulation = False

        # # Check the fusion mode from unit tests
        # if unitTestEnable:
        #     # self.unitTest = [[0,0,1],
        #     #                  [0,0,0],
        #     #                  [0,0,0],
        #     #                  [0,0,1]]
        #     self.unitTest = [[2,2,0],
        #                      [2,2,1],
        #                      [1,1,0],
        #                      [1,1,1],
        #                      [0,0,0],
        #                      [0,0,1],
        #                      #[1,1,0],
        #                      #[1,1,1],
        #                      #[2,2,0],
        #                      #[2,2,1],
        #                      #[0,2,1],
        #                      [0,0,0]]
        #     # self.unitTest = [[0,0],
        #     #             [1,1],
        #     #             [2,2]]
        #     # self.unitTest = [[0,0],
        #     #             [1,0],
        #     #             [2,0],
        #     #             [0,1],
        #     #             [1,1],
        #     #             [2,1],
        #     #             [0,2],
        #     #             [1,2],
        #     #             [2,2]]
        #     self.local_fusion_mode = self.unitTest[0][0]
        #     self.global_fusion_mode = self.unitTest[0][1]
        # else:
        #     # Default to 1
        #     # TODO: add a button for this
        #     self.local_fusion_mode = 0
        #     self.global_fusion_mode = 0

        # # Create the simulated LIDARs, planner, etc.
        # self.localizationsList = []
        # self.lidarRecognitionList = []
        # self.localFusionCAV = []
        # self.localFusionCIS = []
        # self.globalFusionList = []
        # self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
        # # Add in the arrays for local fusion storage for CAV vehicles
        # for idx, veh in self.vehicles.items():
        #     if veh.simVehicle:
        #         self.lidarRecognitionList.append(lidar_recognition.LIDAR(0.0))
        #         self.localFusionCAV.append(local_fusion.FUSION(self.local_fusion_mode, idx))
        #     else:
        #         self.lidarRecognitionList.append(None)
        #         self.localFusionCAV.append(None)
        # # Add in the arrays for local fusion storage for CIS sensors
        # for idx, sens in self.cis.items():
        #     if sens.simCIS:
        #         # 1000 is to make sure our sensor ids are different for CIS vs CAV
        #         self.localFusionCIS.append(local_fusion.FUSION(self.local_fusion_mode, 1000 + idx))
        #     else:
        #         self.localFusionCIS.append(None)

        # # Keep track of stats if this is a simulation
        # self.unit_test_state = 0
        # self.unit_test_idx = 0
        # self.unit_test_local_over_detection_miss_results = []
        # self.unit_test_local_under_detection_miss_results = []
        # self.unit_test_local_rmse_results = []
        # self.unit_test_local_variance_results = []
        # self.local_over_detection_miss = 0
        # self.local_under_detection_miss = 0
        # self.local_differences = []

        # # localization stats
        # self.unit_test_localization_rmse_results = []
        # self.unit_test_localization_variance_results = []
        # self.localization_differences = []

        # # Global stats
        # self.unit_test_global_over_detection_miss_results = []
        # self.unit_test_global_under_detection_miss_results = []
        # self.unit_test_global_rmse_results = []
        # self.unit_test_global_variance_results = []
        # self.global_over_detection_miss = 0
        # self.global_under_detection_miss = 0
        # self.global_differences = []

        # self.localization_differences = []
        # self.localization_velocity = []

        # self.real_lidar = True

        # # Parameters of test
        # self.mapSpecs = mapSpecs

        # # Time params
        # self.time = 0
        # self.lastPositionUpdate = 0
        # self.lastLocalizationUpdate = 0
        # self.lightTime = 0

        self.end_simulation = False
        self.pause_simulation = True
        self.last_pause_simulation = True

        # Draw params
        self.drawTrafficLight = False
        self.display_localization = True

        QMainWindow.__init__(self)

        self.setMinimumSize(QSize(1280, 1000))
        self.setWindowTitle("Demo GUI")

        self.labelName = QLabel(self)
        self.labelName.setText('Settings:')
        self.labelName.move(1000, 20)

        self.sensorsButton = QPushButton('Quick Sensor Sim', self)
        self.sensorsButton.resize(140, 32)
        self.sensorsButton.move(1000, 310)
        self.sensorsButton.clicked.connect(self.on_sensors_clicked)

        self.lidarButton = QPushButton('LIDAR Debug Off', self)
        self.lidarButton.resize(140, 32)
        self.lidarButton.move(1000, 350)
        self.lidarButton.clicked.connect(self.on_lidar_clicked)
        self.lidar_debug = False

        self.cameraButton = QPushButton('Camera Debug Off', self)
        self.cameraButton.resize(140, 32)
        self.cameraButton.move(1000, 390)
        self.cameraButton.clicked.connect(self.on_camera_clicked)
        self.camera_debug = False

        self.fusionButton = QPushButton('Fusion Local Debug Off', self)
        self.fusionButton.resize(140, 32)
        self.fusionButton.move(1000, 430)
        self.fusionButton.clicked.connect(self.on_fusion_clicked)
        self.fusion_debug = False

        self.pathButton = QPushButton('Path Debug Off', self)
        self.pathButton.resize(140, 32)
        self.pathButton.move(1000, 470)
        self.pathButton.clicked.connect(self.on_path_clicked)
        self.path_debug = False

        self.errorButton = QPushButton('Simulate Error Off', self)
        self.errorButton.resize(140, 32)
        self.errorButton.move(1000, 510)
        self.errorButton.clicked.connect(self.on_simulate_error_clicked)
        self.simulate_error = False

        self.covarianceButton = QPushButton('Estimate Local Covariance Off', self)
        self.covarianceButton.resize(140, 32)
        self.covarianceButton.move(1000, 550)
        self.covarianceButton.clicked.connect(self.on_estimate_covariance_clicked)
        self.estimate_covariance = False

        self.covarianceDisplayButton = QPushButton('Display Covariance Off', self)
        self.covarianceDisplayButton.resize(140, 32)
        self.covarianceDisplayButton.move(1000, 590)
        self.covarianceDisplayButton.clicked.connect(self.on_display_covariance_clicked)
        self.display_covariance = False

        # self.covarianceDisplayButton = QPushButton('Display Covariance Off', self)
        # self.covarianceDisplayButton.resize(140, 32)
        # self.covarianceDisplayButton.move(1000, 590)
        # self.covarianceDisplayButton.clicked.connect(self.on_display_covariance_clicked)
        self.display_global_fusion = True

        self.testGroup = QButtonGroup(self)  # Radio button group

        self.radioTrafficLight = QRadioButton("Traffic Light", self)
        self.radioTrafficLight.resize(200, 32)
        self.radioTrafficLight.move(1000, 630)
        # self.radioTrafficLight.clicked.connect(self.showCustomOptions)
        self.radioTrafficLight.toggle()  # start in traffic test
        self.testGroup.addButton(self.radioTrafficLight)

        self.radioAutonomousIntersection = QRadioButton("Autonomous Intersection", self)
        self.radioAutonomousIntersection.resize(200, 32)
        self.radioAutonomousIntersection.move(1000, 660)
        # self.radioAutonomousIntersection.clicked.connect(self.showCustomOptions)
        self.testGroup.addButton(self.radioAutonomousIntersection)

        self.startButton = QPushButton('Start Test', self)
        self.startButton.resize(140, 32)
        self.startButton.move(1000, 700)

        self.startButton.clicked.connect(self.on_start_clicked)

        self.pauseButton = QPushButton('Pause Test', self)
        self.pauseButton.setEnabled(False)
        self.pauseButton.resize(140, 32)
        self.pauseButton.move(1000, 740)

        self.pauseButton.clicked.connect(self.on_pause_clicked)

        self.endButton = QPushButton('End Test', self)
        self.endButton.setEnabled(False)
        self.endButton.resize(140, 32)
        self.endButton.move(1000, 780)

        self.endButton.clicked.connect(self.on_end_clicked)

        self.unitTestButton = QPushButton('Unit Test Off', self)
        self.unitTestButton.setEnabled(True)
        self.unitTestButton.resize(140, 32)
        self.unitTestButton.move(1000, 820)

        self.unitTestButton.clicked.connect(self.on_unit_test_clicked)
        self.unit_test = False

        self.drawIntersection = True

        # Set this to true after we have set the coordinates set
        # as this will enable the GUI to draw.
        self.drawCoordinates = True

        self.labelVehicleSpeed = []
        self.lineVehicleSpeed = []
        self.labelVehicleSpeedActual = []
        self.labelVehicleSpeedTarget = []
        self.labelVehicleAcceleration = []
        self.last_line_vehicle_speed = []

        for idx, vehicle in enumerate(self.vehicles):
            self.labelVehicleSpeed.append(QLabel(self))
            self.labelVehicleSpeed[idx].setText('Speed Vehicle ' + str(idx) + ':')
            self.labelVehicleSpeed[idx].move(1000, 80 + 80 * idx)

            self.lineVehicleSpeed.append(QLineEdit(self))
            self.lineVehicleSpeed[idx].move(1000, 120 + 80 * idx)
            self.lineVehicleSpeed[idx].resize(100, 32)
            self.lineVehicleSpeed[idx].setText("0")
            self.last_line_vehicle_speed.append(0.0)

            self.labelVehicleSpeedActual.append(QLabel(self))
            self.labelVehicleSpeedActual[idx].setText('VA=0')
            self.labelVehicleSpeedActual[idx].move(1100, 120 + 80 * idx)

            self.labelVehicleSpeedTarget.append(QLabel(self))
            self.labelVehicleSpeedTarget[idx].setText('VT=0')
            self.labelVehicleSpeedTarget[idx].move(1150, 120 + 80 * idx)

            self.labelVehicleAcceleration.append(QLabel(self))
            self.labelVehicleAcceleration[idx].setText('AA=0')
            self.labelVehicleAcceleration[idx].move(1200, 120 + 80 * idx)

        self.labelList = []
        self.labelList.append(QLabel(self))
        self.labelList[0].setText("Legend:")
        self.labelList[0].move(5, 5)
        for idx, brush in enumerate(brush_color):
            self.labelList.append(QLabel(self))
            # self.labelList[idx].setAutoFillBackground(True) # This is important!!
            # color  = brush_color[brush]
            # print(color)
            # alpha  = 140
            # values = "{r}, {g}, {b}, {a}".format(r = color.red(),
            #                                     g = color.green(),
            #                                     b = color.blue(),
            #                                     a = alpha
            #                                     )
            # self.labelList[idx].setStyleSheet("QLabel { background-color: rgba("+values+"); }")
            self.labelList[idx].setText(brush)
            #self.labelList[idx].setStyleSheet("background-color: " + str(brush_color[brush]))
            self.labelList[idx].move(5, 15 + (idx * 10))


        self.setVelocity = True

        self.drawVehicle = True

        self.drawCamera = True

        print ( " GUI Init end ")

        # Do this forevvvvveerrrrr!!!
        timer = QtCore.QTimer(self, timeout=self.stepTime, interval=config.gui_interval)
        timer.start()

    def on_start_clicked(self):
        self.endButton.setEnabled(True)
        self.pauseButton.setEnabled(True)
        self.startButton.setEnabled(False)
        self.pause_simulation = False

    def on_pause_clicked(self):
        self.pauseButton.setEnabled(False)
        self.startButton.setEnabled(True)
        self.pause_simulation = True

    def on_sensors_clicked(self):
        if not self.unit_test:
            if self.sensorsButton.text() == 'Full Sensor Sim':
                self.full_simulation = False
                self.sensorsButton.setText('Quick Sensor Sim')
            else:
                self.full_simulation = True
                self.sensorsButton.setText('Full Sensor Sim')

    def on_lidar_clicked(self):
        if self.lidarButton.text() == 'LIDAR Debug Off':
            self.lidar_debug = True
            self.lidarButton.setText('LIDAR Debug On')
        else:
            self.lidar_debug = False
            self.lidarButton.setText('LIDAR Debug Off')

    def on_camera_clicked(self):
        if self.cameraButton.text() == 'Camera Debug Off':
            self.camera_debug = True
            self.cameraButton.setText('Camera Debug On')
        else:
            self.camera_debug = False
            self.cameraButton.setText('Camera Debug Off')

    def on_fusion_clicked(self):
        if self.fusionButton.text() == 'Fusion Local Debug Off':
            self.fusion_debug = True
            self.fusionButton.setText('Fusion Local Debug On')
        else:
            self.fusion_debug = False
            self.fusionButton.setText('Fusion Local Debug Off')

    def on_path_clicked(self):
        if self.pathButton.text() == 'Path Debug Off':
            self.path_debug = True
            self.pathButton.setText('Path Debug On')
        else:
            self.path_debug = False
            self.pathButton.setText('Path Debug Off')

    def on_simulate_error_clicked(self):
        if not self.unit_test:
            if self.errorButton.text() == 'Simulate Error Off':
                self.simulate_error = True
                self.errorButton.setText('Simulate Error On')
            else:
                self.simulate_error = False
                self.errorButton.setText('Simulate Error Off')

    def on_estimate_covariance_clicked(self):
        if not self.unit_test:
            if self.covarianceButton.text() == 'Estimate Local Covariance Off':
                self.estimate_covariance = True
                self.covarianceButton.setText('Estimate Local Covariance On')
            else:
                self.estimate_covariance = False
                self.covarianceButton.setText('Estimate Local Covariance Off')

    def on_display_covariance_clicked(self):
        if self.covarianceDisplayButton.text() == 'Display Covariance Off':
            self.display_covariance = True
            self.covarianceDisplayButton.setText('Display Covariance On')
        else:
            self.display_covariance = False
            self.covarianceDisplayButton.setText('Display Covariance Off')

    def on_unit_test_clicked(self):
        if self.unitTestButton.text() == 'Unit Test Off':
            if self.time == 0.0:
                self.unit_test = True
                self.unitTestButton.setText('Unit Test On')

    def on_end_clicked(self):
        self.end_simulation = True

    # def updateVehicleInSim(self, idx, full_simulation, vehicleList, thread_idx):
    #     # Update ourself
    #     vehicle = self.vehicles[idx]
    #     vehicle.update_localization()
    #     vehicle.recieve_coordinate_group_commands(self.trafficLightArray)
    #     vehicle.pure_pursuit_control()

    #     # Filter out ourself
    #     tempList = vehicleList.copy()
    #     tempList.pop(idx)

    #     localization_error = [0.0, 0.0]

    #     if full_simulation:
    #         # Create that fake LIDAR
    #         if self.lidarRecognitionList[idx] != None:
    #             localization_error_gaussian, localization_error = vehicle.localization.getErrorParamsAtVelocity(abs(vehicle.velocity), vehicle.theta)
    #             if self.estimate_covariance:
    #                 temp_covariance = localization_error_gaussian
    #             else:
    #                 temp_covariance = sensor.BivariateGaussian(0.175, 0.175, 0)
    #             point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(vehicle, tempList, [], 15.0, 15.0, 0.0, 160.0, l_error = localization_error, l_error_gauss = temp_covariance)
    #             if self.simulate_error:
    #                 vehicle.cameraDetections = camera_error_array
    #                 vehicle.localizationError = localization_error_gaussian
    #                 if self.real_lidar:
    #                     lidarcoordinates, lidartimestamp = self.lidarRecognitionList[idx].processLidarFrame(point_cloud_error, self.time/1000.0,
    #                         vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, vehicle.lidarSensor)
    #                     vehicle.rawLidarDetections = point_cloud_error
    #                     vehicle.lidarDetections = lidarcoordinates
    #                 else:
    #                     vehicle.lidarDetections = lidar_detected_error
    #             else:
    #                 vehicle.cameraDetections = camera_array
    #                 lidarcoordinates, lidartimestamp = self.lidarRecognitionList[idx].processLidarFrame(point_cloud, self.time/1000.0,
    #                     vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, vehicle.lidarSensor)
    #                 vehicle.rawLidarDetections = point_cloud
    #                 vehicle.lidarDetections = lidarcoordinates
    #             vehicle.groundTruth = camera_array

    #             # Vehicle position can be the map centroid in sim
    #             # because we are generating the detection WRT the centroid
    #             #pos = [vehicle.localizationPositionX - vehicle.positionX_offset, vehicle.localizationPositionY - vehicle.positionY_offset, vehicle.theta - vehicle.theta_offset]
    #             #pos = [0,0,0]

    #             # Lets add the detections to the vehicle class
    #             # vehicle.lidarDetections = []
    #             # for each in lidarcoordinates:
    #             #     new = rotate((0, 0), (float(each[1]), float(each[2])), pos[2])
    #             #     sensed_x = new[0] + pos[0]
    #             #     sensed_y = new[1] + pos[1]
    #             #     vehicle.lidarDetections.append((sensed_x, sensed_y, each[6]))

    #             # Raw LIDAR for debug
    #             vehicle.lidarPoints = point_cloud

    #             # Do the local fusion like we would on the vehicle
    #             self.localFusionCAV[idx].processDetectionFrame(local_fusion.CAMERA, self.time/1000.0, vehicle.cameraDetections, .25, self.estimate_covariance)
    #             self.localFusionCAV[idx].processDetectionFrame(local_fusion.LIDAR, self.time/1000.0, vehicle.lidarDetections, .25, self.estimate_covariance)
    #             results = self.localFusionCAV[idx].fuseDetectionFrame(self.estimate_covariance, vehicle)

    #             # Add to the GUI
    #             vehicle.fusionDetections = []
    #             for each in results:
    #                 sensed_x = each[1]
    #                 sensed_y = each[2]
    #                 vehicle.fusionDetections.append((sensed_x, sensed_y, each[3], each[4], each[5], each[0]))
    #             # Add ourself to this
    #             self.localizationsList.append((vehicle.localizationPositionX + localization_error[0],
    #                                             vehicle.localizationPositionY + localization_error[1],
    #                                             localization_error_gaussian.covariance, 0, 0, -1))

    #     else:
    #         # Quick fake of sensor values
    #         vehicle.fusionDetections = []
    #         for each in tempList:
    #             sensed_x = each[0]
    #             sensed_y = each[1]
    #             vehicle.fusionDetections.append((sensed_x, sensed_y, np.array([[1.0, 0.0],[0.0, 1.0]]), 0, 0, each[5]))

    #     # Now update our current PID with respect to other vehicles
    #     vehicle.check_positions_of_other_vehicles_adjust_velocity(tempList)

    #     # We can't update the PID controls until after all positions are known
    #     vehicle.update_pid()

    #     # Ground truth to the original dataset
    #     testSet = []
    #     groundTruth = []
    #     for each in vehicle.fusionDetections:
    #         sensed_x = each[0]
    #         sensed_y = each[1]
    #         testSet.append([sensed_x, sensed_y])
    #     for each in vehicle.groundTruth:
    #         sensed_x = each[0]
    #         sensed_y = each[1]
    #         groundTruth.append([sensed_x, sensed_y])

    #     local_differences = []
    #     local_over_detection_miss = 0
    #     local_under_detection_miss = 0

    #     if len(testSet) >= 1 and len(groundTruth) >= 1:
    #         nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(testSet))
    #         distances, indices = nbrs.kneighbors(np.array(groundTruth))

    #         # Now calculate the score
    #         for dist in distances:
    #             local_differences.append(dist)

    #     # Check how much large the test set is from the ground truth and add that as well
    #     if len(testSet) > len(groundTruth):
    #         # Overdetection case
    #         local_over_detection_miss += len(testSet) - len(groundTruth)
    #     elif len(testSet) < len(groundTruth):
    #         # Underdetection case, we count this differently because it may be from obstacle blocking
    #         local_under_detection_miss += len(groundTruth) - len(testSet)

    #     if self.useThreading:
    #         self.threads_results[thread_idx] = [local_differences, local_over_detection_miss, local_under_detection_miss, localization_error]
    #     else:
    #         return local_differences, local_over_detection_miss, local_under_detection_miss, localization_error

    # def updateCISInSim(self, idx, full_simulation, vehicleList, thread_idx):
    #     cis = self.cis[idx]
    #     # Don't filter the list at all
    #     tempList = vehicleList.copy()
    #     if full_simulation:
    #         # Create that fake camera
    #         if self.lidarRecognitionList[idx] != None:
    #             point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(cis, tempList, [], 15.0, 15.0, 0.0, 160.0)
    #             if self.simulate_error:
    #                 cis.cameraDetections = camera_error_array
    #             else:
    #                 cis.cameraDetections = camera_array
    #             cis.groundTruth = camera_array
                
    #             # f = open("data_" + str(idx) + ".txt", "a")
    #             # f.write(str(idx) + "," + str(self.time))
    #             # for each in cis.cameraDetections:
    #             #     f.write("," + str(each[0]) + "," + str(each[1]))
    #             # f.write("\n")
    #             # f.close()

    #             # CIS position can be the map centroid in sim
    #             # because we are generating the detection WRT the centroid
    #             # pos = [cis.localizationPositionX - cis.positionX_offset, cis.localizationPositionY - cis.positionY_offset, cis.theta - cis.theta_offset]
    #             pos = [0, 0, 0]

    #             # Fusion detection frame is the same as single camera (for now)
    #             # Add to the GUI
    #             # Do the local fusion like we would on the vehicle
    #             self.localFusionCIS[idx].processDetectionFrame(local_fusion.CAMERA, self.time/1000.0, cis.cameraDetections, .25, self.estimate_covariance)
    #             results = self.localFusionCIS[idx].fuseDetectionFrame(self.estimate_covariance, cis)

    #             # Add to the GUI
    #             cis.fusionDetections = []
    #             for each in results:
    #                 sensed_x = each[1]
    #                 sensed_y = each[2]
    #                 cis.fusionDetections.append((sensed_x, sensed_y, each[3], each[4], each[5], each[0]))
    #     else:
    #         # Quick fake of sensor values
    #         cis.fusionDetections = []
    #         cis.groundTruth = []
    #         for each in vehicleList:
    #             sensed_x = each[0]
    #             sensed_y = each[1]
    #             cis.fusionDetections.append((sensed_x, sensed_y, np.array([[1.0, 0.0],[0.0, 1.0]]), 0, 0, each[5]))

    #     # Ground truth to the original dataset
    #     testSet = []
    #     groundTruth = []
    #     for each in cis.fusionDetections:
    #         sensed_x = each[0]
    #         sensed_y = each[1]
    #         testSet.append([sensed_x, sensed_y])
    #     for each in cis.groundTruth:
    #         sensed_x = each[0]
    #         sensed_y = each[1]
    #         groundTruth.append([sensed_x, sensed_y])

    #     local_differences = []
    #     local_over_detection_miss = 0
    #     local_under_detection_miss = 0

    #     if len(testSet) >= 1 and len(groundTruth) >= 1:
    #         nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(testSet))
    #         distances, indices = nbrs.kneighbors(np.array(groundTruth))

    #         # Now calculate the score
    #         for dist in distances:
    #             local_differences.append(dist)

    #     # Check how much large the test set is from the ground truth and add that as well
    #     if len(testSet) > len(groundTruth):
    #         # Overdetection case
    #         local_over_detection_miss += len(testSet) - len(groundTruth)
    #     elif len(testSet) < len(groundTruth):
    #         # Underdetection case, we count this differently because it may be from obstacle blocking
    #         local_under_detection_miss += len(groundTruth) - len(testSet)

    #     if self.useThreading:
    #         self.threads_results[thread_idx] = [local_differences, local_over_detection_miss, local_under_detection_miss, [0.0, 0.0]]
    #     else:
    #         return local_differences, local_over_detection_miss, local_under_detection_miss

    def calcResultsOfUnitTest(self):
        # Calculate the prior results
        # Localization
        differences_squared_l = np.array(self.localization_differences) ** 2
        mean_of_differences_squared_l = differences_squared_l.mean()
        rmse_val_l = np.sqrt(mean_of_differences_squared_l)
        variance_l = np.var(self.localization_differences,ddof=1)

        self.unit_test_localization_rmse_results.append(rmse_val_l)
        self.unit_test_localization_variance_results.append(variance_l)

        # Onboard
        differences_squared = np.array(self.local_differences) ** 2
        mean_of_differences_squared = differences_squared.mean()
        rmse_val = np.sqrt(mean_of_differences_squared)
        variance = np.var(self.local_differences,ddof=1)

        self.unit_test_local_rmse_results.append(rmse_val)
        self.unit_test_local_variance_results.append(variance)
        self.unit_test_local_under_detection_miss_results.append(self.local_under_detection_miss)
        self.unit_test_local_over_detection_miss_results.append(self.local_over_detection_miss)

        # Global
        differences_squared_g = np.array(self.global_differences) ** 2
        mean_of_differences_squared_g = differences_squared_g.mean()
        rmse_val_g = np.sqrt(mean_of_differences_squared_g)
        variance_g = np.var(self.global_differences,ddof=1)

        self.unit_test_global_rmse_results.append(rmse_val_g)
        self.unit_test_global_variance_results.append(variance_g)
        self.unit_test_global_under_detection_miss_results.append(self.global_under_detection_miss)
        self.unit_test_global_over_detection_miss_results.append(self.global_over_detection_miss)

    def printUnitTestStats(self):
        idx = 0
        fails = 0
        for l_rmse, l_var, o_rmse, o_var, o_u_miss, o_o_miss, g_rmse, g_var, g_u_miss, g_o_miss in zip(self.unit_test_localization_rmse_results, self.unit_test_localization_variance_results, 
            self.unit_test_local_rmse_results, self.unit_test_local_variance_results,
            self.unit_test_local_under_detection_miss_results, self.unit_test_local_over_detection_miss_results,
            self.unit_test_global_rmse_results, self.unit_test_global_variance_results,
            self.unit_test_global_under_detection_miss_results, self.unit_test_global_over_detection_miss_results):
            print( "Test: ", idx, " g_mode:", self.unitTest[idx][0], " l_mode:", self.unitTest[idx][1], " est_cov:", self.unitTest[idx][2] )
            print( "  localization_rmse_val: ", l_rmse, " variance: ", l_var)
            print( "  onboard_rmse_val: ", o_rmse, " variance: ", o_var, " over misses: ", o_o_miss, " under misses: ", o_u_miss)
            print( "  global_rmse_val: ", g_rmse, " variance: ", g_var, " over misses: ", g_o_miss, " under misses: ", g_u_miss)
            # if idx == 0:
            #     if rmse < .18 or rmse > 20 or O_miss > (50 * test_time):
            #         fails += 1
            # elif idx == 1:
            #     if rmse < .18 or rmse > 20 or O_miss > (50 * test_time):
            #         fails += 1
            idx += 1

    def resetUnitTestStats(self):
        # Reset the stats
        self.localization_differences = []
        self.local_over_detection_miss = 0
        self.local_under_detection_miss = 0
        self.local_differences = []
        self.global_over_detection_miss = 0
        self.global_under_detection_miss = 0
        self.global_differences = []

    def resetTest(self):
        # Reset vehicle positions and filters
        for idx, vehicle in self.vehicles.items():
            # Clear sensors
            vehicle.cameraDetections = []
            vehicle.lidarDetections = []
            vehicle.fusionDetections = []
            vehicle.fusionDetectionsCovariance = []
            vehicle.rawLidarDetections = []
            vehicle.groundTruth = []

            # Raw LIDAR for gui debug
            vehicle.lidarPoints = []

            # Move back to start location
            reverse_theta = vehicle.theta_offset-math.radians(180)
            vehicle.rearAxlePositionX = vehicle.positionX_offset + (vehicle.axleFromCenter * math.cos(reverse_theta))
            vehicle.rearAxlePositionY = vehicle.positionY_offset + (vehicle.axleFromCenter * math.sin(reverse_theta))
            vehicle.localizationPositionX = vehicle.positionX_offset
            vehicle.localizationPositionY = vehicle.positionY_offset
            vehicle.positionX_sim = vehicle.rearAxlePositionX
            vehicle.positionY_sim = vehicle.rearAxlePositionY
            vehicle.velocity = 0
            vehicle.theta = vehicle.theta_offset
            vehicle.lastPointIndex = None

        # Reset detections from CIS sensors
        # Reset vehicle positions and filters
        for idx, cis in self.cis.items():
            # Clear sensors
            cis.cameraDetections = []
            cis.fusionDetections = []
            cis.fusionDetectionsCovariance = []
            cis.groundTruth = []

        # Clear the global fusion
        self.globalFusion.trackedList = []

        # Reset the light
        self.trafficLightArray = [0, 2, 0]

    def stepTime(self):
        # if self.unit_test:
        #     test_time = 60000
        #     test_time_print = 10000
        #     if self.time % test_time_print == 0:
        #         print("Test: ", 100 * (self.time % test_time)/test_time, "% num:", self.unit_test_idx)
        #     if self.time % test_time == 0:
        #         # Reset the map, unit testing has been selected
        #         self.resetTest()

        #         # Determing mode
        #         if self.unit_test_state == 0:
        #             for idx, vehicle in self.vehicles.items():
        #                 self.lineVehicleSpeed[idx].setText("0.5")
        #                 self.lineVehicleSpeed[idx].setReadOnly(True)
        #             self.sensorsButton.setEnabled(False)
        #             self.errorButton.setEnabled(False)
        #             self.covarianceButton.setEnabled(False)
        #             self.endButton.setEnabled(True)
        #             self.pauseButton.setEnabled(False)
        #             self.startButton.setEnabled(False)
        #             self.unitTestButton.setEnabled(False)
        #             self.full_simulation = True
        #             self.simulate_error = True
        #             self.estimate_covariance = False
        #             self.pause_simulation = False
        #             self.real_lidar = False
        #             self.unit_test_idx = 0

        #             # Set the fusion modes
        #             self.local_fusion_mode = self.unitTest[self.unit_test_idx][0]
        #             self.global_fusion_mode = self.unitTest[self.unit_test_idx][1]
        #             self.estimate_covariance = self.unitTest[self.unit_test_idx][2]
        #             self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
        #             for idx, veh in self.vehicles.items():
        #                 if veh.simVehicle:
        #                     self.localFusionCAV[idx].fusion_mode = self.local_fusion_mode
        #             for idx, sens in self.cis.items():
        #                 if sens.simCIS:
        #                     self.localFusionCIS[idx].fusion_mode = self.local_fusion_mode

        #             # Reset the stats
        #             self.resetUnitTestStats()
    
        #             # Increment the unit test counter for those long tests
        #             self.unit_test_state = 1
        #             self.unit_test_idx += 1
        #         elif len(self.unitTest) <= self.unit_test_idx:
        #             # Calculate the prior results
        #             self.calcResultsOfUnitTest()
        #             self.resetUnitTestStats()
        #             self.printUnitTestStats()
                    
        #             # Set everythign back to normal
        #             self.real_lidar = True

        #             for idx, vehicle in self.vehicles.items():
        #                 self.lineVehicleSpeed[idx].setText("0.0")
        #                 self.lineVehicleSpeed[idx].setReadOnly(False)

        #             # Test over
        #             self.sensorsButton.setEnabled(True)
        #             self.errorButton.setEnabled(True)
        #             self.covarianceButton.setEnabled(True)
        #             self.endButton.setEnabled(True)
        #             self.pauseButton.setEnabled(False)
        #             self.startButton.setEnabled(True)
        #             self.unit_test_state = 0
        #             self.full_simulation = False
        #             self.simulate_error = False
        #             self.estimate_covariance = False
        #             self.pause_simulation = True
        #             self.unitTestButton.setEnabled(True)
        #             self.unit_test = False
        #             self.unitTestButton.setText('Unit Test Off')
        #             self.unitTestButton.setEnabled(True)

        #             sys.exit()
        #         else:
        #             # Calculate the prior results
        #             self.calcResultsOfUnitTest()
        #             self.resetUnitTestStats()

        #             for idx, vehicle in self.vehicles.items():
        #                 self.lineVehicleSpeed[idx].setText("0.5")
        #             self.full_simulation = True
        #             self.simulate_error = True
        #             self.pause_simulation = False

        #             # Set the fusion modes
        #             self.local_fusion_mode = self.unitTest[self.unit_test_idx][0]
        #             self.global_fusion_mode = self.unitTest[self.unit_test_idx][1]
        #             self.estimate_covariance = self.unitTest[self.unit_test_idx][2]
        #             self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
        #             for idx, veh in self.vehicles.items():
        #                 if veh.simVehicle:
        #                     self.localFusionCAV[idx].fusion_mode = self.local_fusion_mode
        #             for idx, sens in self.cis.items():
        #                 if sens.simCIS:
        #                     self.localFusionCIS[idx].fusion_mode = self.local_fusion_mode

        #             # Incrememt the unit test state
        #             self.unit_test_idx += 1


        # if not self.pause_simulation:
        #     if self.full_simulation:
        #             self.time += 125
        #     else:
        #         self.time = round(time.time() * 1000)

        # # 8HZ
        # if self.full_simulation:
        #     if (self.time - self.lastPositionUpdate) >= 125:
        #         self.lastPositionUpdate = self.time
        #         self.vehiclesLock.acquire()
        #         for key, vehicle in self.vehicles.items():
        #             # Update vehicle position based on physics
        #             if vehicle.simVehicle:
        #                 vehicle.updatePosition(.125)
        #         self.vehiclesLock.release()
        #         self.drawTrafficLight = True

        # # 8HZ
        # if (self.time - self.lastLocalizationUpdate) >= 125:
        #     # print ( self.time )
        #     self.lastLocalizationUpdate = self.time
        #     # Traffic light update sequence
        #     if self.lightTime > self.mapSpecs.lightTimePeriod:
        #         self.lightTime = 0
        #         if self.trafficLightArray[1] == 2:
        #             self.trafficLightArray[1] = 1
        #             self.trafficLightArray[2] = 0
        #             lightTimePeriod = 0 * 8
        #         elif self.trafficLightArray[2] == 2:
        #             self.trafficLightArray[1] = 0
        #             self.trafficLightArray[2] = 1
        #             lightTimePeriod = 0 * 8
        #         elif self.trafficLightArray[1] == 1:
        #             self.trafficLightArray[1] = 0
        #             self.trafficLightArray[2] = 2
        #             lightTimePeriod = 5 * 8
        #         elif self.trafficLightArray[2] == 1:
        #             self.trafficLightArray[1] = 2
        #             self.trafficLightArray[2] = 0
        #             lightTimePeriod = 5 * 8
        #     else:
        #         self.lightTime += 1

        #     #self.vehiclesLock.acquire()

        #     start_vehicles = time.time()

        #     # Make the vehicle list before we move the vehicles
        #     # Get the last known location of all other vehicles
        #     vehicleList = []
        #     for otherIdx, otherVehicle in self.vehicles.items():
        #         vehicleList.append(otherVehicle.get_location())

        #     self.localizationsList = []
        #     for idx, vehicle in self.vehicles.items():
        #         if self.pause_simulation:
        #             # Make sure we relay the pause to our vehicles
        #             vehicle.targetVelocityGeneral = 0.0
        #         elif self.lineVehicleSpeed[idx].text() == "" or self.lineVehicleSpeed[idx].text() == ".":
        #             # Need to pass here in case the user is still typing
        #             pass
        #         else:
        #             vehicle.targetVelocityGeneral = float(self.lineVehicleSpeed[idx].text())
        #         if vehicle.simVehicle:
        #             if self.pause_simulation:
        #                 vehicle.update_localization()
        #                 vehicle.distance_pid_control_overide = True
        #                 vehicle.targetVelocity = 0.0
        #                 vehicle.update_pid()
        #             else:
        #                 if self.useThreading:
        #                     self.threads[idx] = threading.Thread(target=self.updateVehicleInSim, args=(idx, self.full_simulation, vehicleList, idx))      
        #                     self.threads[idx].start() # start the thread we just created
        #                 else:
        #                     local_differences_c, local_over_detection_miss_c, local_under_detection_miss_c, localization_error_c = self.updateVehicleInSim(idx, self.full_simulation, vehicleList, idx)
        #                     self.local_under_detection_miss += local_under_detection_miss_c
        #                     self.local_over_detection_miss += local_over_detection_miss_c
        #                     self.local_differences += local_differences_c
        #                     self.localization_differences.append(math.hypot(localization_error_c[0], localization_error_c[1]))
        #                 # self.localization_velocity.append(vehicle.velocity)

        #     for idx, cis in self.cis.items():
        #         #print ( " CIS:", idx )
        #         # Do this if we are not in a full sim
        #         if not self.full_simulation and cis.simCIS:
        #             # CISs should not move but we can do this anyway just in case
        #             cis.updatePosition(.125)
        #         if cis.simCIS:
        #             if self.pause_simulation:
        #                 cis.update_localization()
        #             else:
        #                 # Update ourself
        #                 cis.update_localization()

        #                 if self.useThreading:
        #                     thread_num = len(self.vehicles) + idx
        #                     self.threads[thread_num] = threading.Thread(target=self.updateCISInSim, args=(idx, self.full_simulation, vehicleList, thread_num))
        #                     self.threads[thread_num].start() # start the thread we just created  
        #                 else:
        #                     local_differences_c, local_over_detection_miss_c, local_under_detection_miss_c = self.updateCISInSim(idx, self.full_simulation, vehicleList, idx)

        #                     self.local_under_detection_miss += local_under_detection_miss_c
        #                     self.local_over_detection_miss += local_over_detection_miss_c
        #                     self.local_differences += local_differences_c

        #     if self.useThreading:
        #         # wait for all threads to finish                                            
        #         for idx, t in enumerate(self.threads):                                                           
        #             t.join()
        #             if self.threads_results[idx] != None:
        #                 local_differences_c, local_over_detection_miss_c, local_under_detection_miss_c, localization_error_c = self.threads_results[idx]
        #                 self.local_under_detection_miss += local_under_detection_miss_c
        #                 self.local_over_detection_miss += local_over_detection_miss_c
        #                 self.local_differences += local_differences_c

        #     #print ( "v:", time.time() - start_vehicles )

        #     #self.vehiclesLock.release()

        #     start_global = time.time()

        #     # First we need to add the localization frame, since it should be the basis
        #     self.globalFusion.processDetectionFrame(-1, self.time/1000.0, self.localizationsList, .25, self.estimate_covariance)

        #     for idx, vehicle in self.vehicles.items():
        #         # Add to the global sensor fusion
        #         self.globalFusion.processDetectionFrame(idx, self.time/1000.0, vehicle.fusionDetections, .25, self.estimate_covariance)

        #     for idx, cis in self.cis.items(): 
        #         # Add to the global sensor fusion
        #         self.globalFusion.processDetectionFrame(idx, self.time/1000.0, cis.fusionDetections, .25, self.estimate_covariance)

        #     self.globalFusionList = self.globalFusion.fuseDetectionFrame(self.estimate_covariance)

        #     # Ground truth to the original dataset
        #     testSetGlobal = []
        #     groundTruthGlobal = []
        #     for each in self.globalFusionList:
        #         sensed_x = each[1]
        #         sensed_y = each[2]
        #         testSetGlobal.append([sensed_x, sensed_y])
        #     for each in vehicleList:
        #         sensed_x = each[0]
        #         sensed_y = each[1]
        #         groundTruthGlobal.append([sensed_x, sensed_y])
        #     if len(testSetGlobal) >= 1 and len(groundTruthGlobal) >= 1:
        #         nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(testSetGlobal))
        #         distances, indices = nbrs.kneighbors(np.array(groundTruthGlobal))

        #         # Now calculate the score
        #         for dist in distances:
        #             if dist > 1.0:
        #                 # Too far away to be considered a match, add as a miss instead
        #                 self.global_under_detection_miss += len(groundTruthGlobal) - len(testSetGlobal)
        #             else:
        #                 self.global_differences.append(dist)
        #     # Check how much large the test set is from the ground truth and add that as well
        #     if len(testSetGlobal) > len(groundTruthGlobal):
        #         # Overdetection case
        #         self.global_over_detection_miss += len(testSetGlobal) - len(groundTruthGlobal)
        #     elif len(testSetGlobal) < len(groundTruthGlobal):
        #         # Underdetection case, we count this differently because it may be from obstacle blocking
        #         self.global_under_detection_miss += len(groundTruthGlobal) - len(testSetGlobal)

        #     #print ( "g:", time.time() - start_global )

        #     #print ( " over misses: ", self.local_over_detection_miss, " under misses: ", self.local_under_detection_miss )
        #     #print ( " over misses: ", self.global_over_detection_miss, " under misses: ", self.global_under_detection_miss )

        response = self.rsu_connection.getGuiValues(True)

        if len(response) > 5:

            self.vehicles = response['vehicle']
            self.camera_fov = response['camera_fov']
            self.camera_center = response['camera_center']
            self.lidar_detection_centroid = response['lidar_detection_centroid']
            self.camera_detection_centroid = response['camera_detection_centroid']
            self.sensor_fusion_centroid = response['sensor_fusion_centroid']
            #self.sensor_fusion_error = response['sensor_fusion_error']
            self.localization_error = response['localization_error']
            self.global_sensor_fusion_centroid = response['global_sensor_fusion_centroid']
            self.trafficLightArray = response['traffic_light']
            self.lidar_detection_raw = response['lidar_detection_raw']

            self.drawIntersection = True
            self.drawCoordinates = True
            self.drawTrafficLight = True

        self.drawTrafficLight = True

        new_speed_target = False
        for idx, vehicle in enumerate(self.vehicles):
            if not (self.lineVehicleSpeed[idx].text() == "" or self.lineVehicleSpeed[idx].text() == "."):
                if float(self.lineVehicleSpeed[idx].text()) != self.last_line_vehicle_speed[idx]:
                    new_speed_target = True
                    self.last_line_vehicle_speed[idx] = float(self.lineVehicleSpeed[idx].text())

        if self.pause_simulation != self.last_pause_simulation or self.end_simulation or new_speed_target:
            self.rsu_connection.sendGuiValues(self.last_line_vehicle_speed, self.pause_simulation, self.end_simulation, None)
            self.last_pause_simulation = self.pause_simulation
            if self.end_simulation:
                sys.exit()

        QApplication.processEvents()
        self.update()

    def translateX(self, x):
        return self.mapSpecs.centerX + x

    def translateY(self, y):
        return self.mapSpecs.centerY - y

    def translateDetections(self, x, y, theta, sens_x, sens_y, sens_theta):
        # Have to do this to convert to gui coordinate system from actual
        sens_theta = sens_theta - math.radians(180)
        return ( x * math.cos(theta + sens_theta)) - (y * math.sin(theta + sens_theta)) + sens_x , ( x * math.sin(theta + sens_theta)) + (y * math.cos(theta + sens_theta )) + sens_y

    def drawTargetArc(self, x0, y0, x1, y1, x2, y2, painter):
        r = math.hypot(x1 - x0, y1 - y0)
        x = x0 - r
        y = y0 + r
        width = 2 * r
        height = 2 * r
        startAngle = math.atan2(y1 - y0, x1 - x0)
        endAngle = math.atan2(y2 - y0, x2 - x0)
        angleLen = math.degrees(shared_math.angleDifference(startAngle, endAngle))
        startAngle = math.degrees(startAngle)
        painter.drawArc(self.translateX(x * self.mapSpecs.meters_to_print_scale),
                        self.translateY(y * self.mapSpecs.meters_to_print_scale), width * self.mapSpecs.meters_to_print_scale,
                        height * self.mapSpecs.meters_to_print_scale, startAngle * 16, angleLen * 16)

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.drawIntersection:
            painter.setPen(Qt.black)

            # Draw top section
            painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) - (
                                         self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
            painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) - (
                                         self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

            # Draw bottom section
            painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) + (
                                         self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
            painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) + (
                                         self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

            # Draw left section
            painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) - (
                                         self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
            painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) - (
                                         self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

            # Draw right section
            painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) + (
                                         self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
            painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) + (
                                         self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

            # Draw top right 3/4 circle
            painter.drawArc(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                            self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) - (
                                        self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2),
                            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2,
                            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2, -90 * 16, 270 * 16)
            painter.drawArc(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                            self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 1.5) - (
                                        self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2),
                            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2 + (
                                        self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 2),
                            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2 + (
                                        self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 2), -90 * 16, 270 * 16)

            # Draw top right 3/4 circle
            painter.drawArc(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                            self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) + (
                                        self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2),
                            -self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2,
                            -self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2, 90 * 16, 270 * 16)
            painter.drawArc(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                            self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 1.5) + (
                                        self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2),
                            -self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2 - (
                                        self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 2),
                            -self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2 - (
                                        self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 2), 90 * 16, 270 * 16)

            # self.mapSpecs.drawIntersection = False

        if self.drawCoordinates:
            pen = QPen()
            pen.setWidth(2)

            for x, y, intersection in zip(self.mapSpecs.xCoordinates, self.mapSpecs.yCoordinates, self.mapSpecs.vCoordinates):
                # painter.translate(self.mapSpecs.centerX-15, self.mapSpecs.centerY-30);
                # painter.rotate(90);
                if intersection == 0:
                    # No inersection here
                    pen.setBrush(brush_color['waypoint'])
                    pen.setWidth(2)
                elif self.trafficLightArray[intersection] == 2:
                    pen.setWidth(4)
                    pen.setBrush(brush_color['traffic_light_green'])
                elif self.trafficLightArray[intersection] == 1:
                    pen.setWidth(4)
                    pen.setBrush(brush_color['traffic_light_yellow'])
                else:
                    pen.setWidth(4)
                    pen.setBrush(brush_color['traffic_light_red'])
                painter.setPen(pen)
                painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * x),
                                  self.translateY(self.mapSpecs.meters_to_print_scale * y))


        if self.drawVehicle:
            for idx, vehicle in enumerate(self.vehicles):
                pen = QPen()
                pen.setWidth(4)
                pen.setBrush(brush_color['vehicle'])
                painter.setPen(pen)

                localizationPositionX = vehicle[0]
                localizationPositionY = vehicle[1]
                theta = vehicle[2]
                velocity = vehicle[3]
                wheelbaseWidth = vehicle[4]
                wheelbaseLength = vehicle[5]
                steeringAcceleration = vehicle[6]
                targetIndexX = vehicle[7]
                targetIndexY = vehicle[8]
                rearAxlePositionX = vehicle[9]
                rearAxlePositionY = vehicle[10]
                targetVelocity = vehicle[11]
                motorAcceleration = vehicle[12]
                width = vehicle[13]
                length = vehicle[14]

                # Draw the vehicle position
                # Front
                front_axle_offset_x = (wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta)
                front_axle_offset_y = (wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta)
                rear_axle_offset_x = (wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(180))
                rear_axle_offset_y = (wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(180))
                painter.drawLine(self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90))) + front_axle_offset_x,
                                 self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + front_axle_offset_y),
                                 self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90))) + front_axle_offset_x,
                                 self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + front_axle_offset_y))
                # Middle
                painter.drawLine(self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(180))),
                                 self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(180))),
                                 self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta)),
                                 self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta)))
                # Back
                painter.drawLine(self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90))) + rear_axle_offset_x,
                                 self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + rear_axle_offset_y),
                                 self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90))) + rear_axle_offset_x,
                                 self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + rear_axle_offset_y))

                # Wheels
                driver_front_center_x = localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90)) + front_axle_offset_x
                driver_front_center_y =  localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + front_axle_offset_y
                passenger_front_center_x = localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90)) + front_axle_offset_x
                passenger_front_center_y =  localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + front_axle_offset_y
                driver_rear_center_x = localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90)) + rear_axle_offset_x
                driver_rear_center_y =  localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + rear_axle_offset_y
                passenger_rear_center_x = localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + math.radians(90)) + rear_axle_offset_x
                passenger_rear_center_y =  localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + math.radians(90)) + rear_axle_offset_y

                # Driver side front
                painter.drawLine(self.translateX(driver_front_center_x + 5 * math.cos(theta + steeringAcceleration)),
                                 self.translateY(driver_front_center_y + 5 * math.sin(theta + steeringAcceleration)),
                                 self.translateX(driver_front_center_x - 5 * math.cos(theta + steeringAcceleration)),
                                 self.translateY(driver_front_center_y - 5 * math.sin(theta + steeringAcceleration)))
                # Passenger side front
                painter.drawLine(self.translateX(passenger_front_center_x + 5 * math.cos(theta + steeringAcceleration)),
                                 self.translateY(passenger_front_center_y + 5 * math.sin(theta + steeringAcceleration)),
                                 self.translateX(passenger_front_center_x - 5 * math.cos(theta + steeringAcceleration)),
                                 self.translateY(passenger_front_center_y - 5 * math.sin(theta + steeringAcceleration)))
                # Driver side rear
                painter.drawLine(self.translateX(driver_rear_center_x + 5 * math.cos(theta)),
                                 self.translateY(driver_rear_center_y + 5 * math.sin(theta)),
                                 self.translateX(driver_rear_center_x - 5 * math.cos(theta)),
                                 self.translateY(driver_rear_center_y - 5 * math.sin(theta)))
                # Passenger side rear
                painter.drawLine(self.translateX(passenger_rear_center_x + 5 * math.cos(theta)),
                                 self.translateY(passenger_rear_center_y + 5 * math.sin(theta)),
                                 self.translateX(passenger_rear_center_x - 5 * math.cos(theta)),
                                 self.translateY(passenger_rear_center_y - 5 * math.sin(theta)))

                if self.path_debug:
                    # Draw the target point
                    pen.setBrush(brush_color['target_point'])
                    painter.setPen(pen)
                    painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * targetIndexX),
                                    self.translateY(self.mapSpecs.meters_to_print_scale * targetIndexY))
                    pen.setBrush(brush_color['wheel_angle'])
                    pen.setWidth(1)
                    painter.setPen(pen)
                    self.drawTargetArc(localizationPositionX, localizationPositionY, (rear_axle_offset_x+driver_rear_center_x)/2, 
                    (rear_axle_offset_y+driver_rear_center_y)/2, targetIndexX, targetIndexY, painter)

                self.labelVehicleSpeedActual[idx].setText('VA=' + str(round(velocity, 2)))
                self.labelVehicleSpeedTarget[idx].setText('VT=' + str(round(targetVelocity, 2)))
                self.labelVehicleAcceleration[idx].setText('AA=' + str(round(motorAcceleration, 2)))

                if self.path_debug:
                    pen.setWidth(.5)
                    pen.setBrush(brush_color['bounding_box'])
                    pen.setWidth(4)
                    painter.setPen(pen)
                    buffer = 0.1
                    x1 = localizationPositionX + (
                                (width/2.0 + buffer) * math.cos(theta + math.radians(90)) + (
                                    (length/2.0 + buffer) * math.cos(theta - math.radians(180))))
                    y1 = localizationPositionY + (
                                (width/2.0 + buffer) * math.sin(theta + math.radians(90)) + (
                                    (length/2.0 + buffer) * math.sin(theta - math.radians(180))))
                    x2 = localizationPositionX + (
                                (width/2.0 + buffer) * math.cos(theta - math.radians(90)) + (
                                    (length/2.0 + buffer) * math.cos(theta - math.radians(180))))
                    y2 = localizationPositionY + (
                                (width/2.0 + buffer) * math.sin(theta - math.radians(90)) + (
                                    (length/2.0 + buffer) * math.sin(theta - math.radians(180))))
                    x3 = localizationPositionX + (
                                (width/2.0 + buffer) * math.cos(theta - math.radians(90)) + (
                                    (length/2.0 + buffer) * math.cos(theta)))
                    y3 = localizationPositionY + (
                                (width/2.0 + buffer) * math.sin(theta - math.radians(90)) + (
                                    (length/2.0 + buffer) * math.sin(theta)))
                    x4 = localizationPositionX + (
                                (width/2.0 + buffer) * math.cos(theta + math.radians(90)) + (
                                    (length/2.0 + buffer) * math.cos(theta)))
                    y4 = localizationPositionY + (
                                (width/2.0 + buffer) * math.sin(theta + math.radians(90)) + (
                                    (length/2.0 + buffer) * math.sin(theta)))

                    # print (localizationPositionX, localizationPositionY, x1, y1, x2, y2)
                    painter.drawPoint(self.translateX(x1 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y1 * self.mapSpecs.meters_to_print_scale))
                    painter.drawPoint(self.translateX(x2 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y2 * self.mapSpecs.meters_to_print_scale))
                    painter.drawPoint(self.translateX(x3 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y3 * self.mapSpecs.meters_to_print_scale))
                    painter.drawPoint(self.translateX(x4 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y4 * self.mapSpecs.meters_to_print_scale))

                if self.camera_debug:
                    # Draw the FOV of cam
                    pen.setWidth(.5)
                    pen.setBrush(brush_color['camera_fov'])
                    painter.setPen(pen)

                    camera_center = math.degrees(self.camera_center[idx])
                    camera_fov = math.degrees(self.camera_fov[idx])

                    painter.drawLine(self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                    self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                            1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                        theta + math.radians(camera_center + (camera_fov/2)))),
                                    self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                            1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                        theta + math.radians(camera_center + (camera_fov/2)))))

                    painter.drawLine(self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                    self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                            1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                        theta + math.radians(camera_center + -(camera_fov/2)))),
                                    self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                            1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                        theta + math.radians(camera_center + -(camera_fov/2)))))

                    # Now draw the vehicle camera detections
                    pen.setBrush(brush_color['camera_detection_centroid'])
                    pen.setWidth(4)
                    painter.setPen(pen)
                    for each in self.camera_detection_centroid[idx]:
                        # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), localizationPositionX, localizationPositionY, theta)
                        painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                        self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

                    # Time for displaying the covaraince
                    if self.display_covariance:
                        pen.setBrush(brush_color['camera_detection_error'])
                        pen.setWidth(.5)
                        painter.setPen(pen)
                        for each in self.camera_detection_centroid[idx]:
                            # Make sure covariance parameters have been added
                            if len(each) >= 3:
                                pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                        self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
                                a, b, phi = shared_math.ellipsify(each[2], 3.0)
                                a = a * self.mapSpecs.meters_to_print_scale
                                b = b * self.mapSpecs.meters_to_print_scale
                                # Save the previous painter envinronment so we don't mess up the other things
                                painter.save()
                                # get the x and y components of the ellipse position
                                ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                                ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                                # translate the center to where our ellipse should be
                                painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                                # Rotate by phi to turn the ellipse the correct way
                                painter.rotate(math.degrees(phi))
                                # Draw the ellipse at 0.0
                                painter.drawEllipse(0, 0, a, b)
                                # Restore the environment to what it was before
                                painter.restore()

                if self.lidar_debug:
                    # Now draw the vehicle lidar detections
                    pen.setBrush(brush_color['lidar_detection_centroid'])
                    pen.setWidth(4)
                    painter.setPen(pen)
                    for each in self.lidar_detection_centroid[idx]:
                        painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                        self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

                    # Draw raw lidar
                    pen.setBrush(brush_color['lidar_detection_raw'])
                    pen.setWidth(2)
                    painter.setPen(pen)
                    for each in self.lidar_detection_raw[idx]:
                        painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                        self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

                    # Time for displaying the covaraince
                    if self.display_covariance:
                        pen.setBrush(brush_color['lidar_detection_error'])
                        pen.setWidth(.5)
                        painter.setPen(pen)
                        for each in self.lidar_detection_centroid[idx]:
                            # Make sure covariance parameters have been added
                            if len(each) >= 3:
                                pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                        self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
                                a, b, phi = shared_math.ellipsify(each[2], 3.0)
                                a = a * self.mapSpecs.meters_to_print_scale
                                b = b * self.mapSpecs.meters_to_print_scale
                                # Save the previous painter envinronment so we don't mess up the other things
                                painter.save()
                                # get the x and y components of the ellipse position
                                ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                                ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                                # translate the center to where our ellipse should be
                                painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                                # Rotate by phi to turn the ellipse the correct way
                                painter.rotate(math.degrees(phi))
                                # Draw the ellipse at 0.0
                                painter.drawEllipse(0, 0, a, b)
                                # Restore the environment to what it was before
                                painter.restore()

                if self.display_localization:
                    pen.setBrush(brush_color['localization_error'])
                    pen.setWidth(.5)
                    painter.setPen(pen)
                    if self.localization_error != []:
                        pos = ( self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale) )
                        a, b, phi = shared_math.ellipsify(self.localization_error[idx], 3.0)
                        a = a * self.mapSpecs.meters_to_print_scale
                        b = b * self.mapSpecs.meters_to_print_scale
                        # Save the previous painter envinronment so we don't mess up the other things
                        painter.save()
                        # get the x and y components of the ellipse position
                        ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                        ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                        # translate the center to where our ellipse should be
                        painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                        # Rotate by phi to tunr the ellipse the correct way
                        painter.rotate(math.degrees(phi))
                        # Draw the ellipse at 0.0
                        painter.drawEllipse(0, 0, a, b)
                        # Restore the environment to what it was before
                        painter.restore()

            if self.fusion_debug:
                # Now draw the vehicle fusion detections
                pen.setBrush(brush_color['sensor_fusion_centroid'])
                pen.setWidth(6)
                painter.setPen(pen)
                for each in self.sensor_fusion_centroid[idx]:
                    painter.drawPoint(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
                pen.setWidth(.5)
                painter.setPen(pen)
                for each in self.sensor_fusion_centroid[idx]:
                    painter.drawLine(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[2] * self.mapSpecs.meters_to_print_scale),
                                    self.translateX((each[1] + (8.0*each[4])) * self.mapSpecs.meters_to_print_scale),
                                    self.translateY((each[2] + (8.0*each[5])) * self.mapSpecs.meters_to_print_scale))

                # if self.display_covariance:
                #     pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
                #     pen.setWidth(.5)
                #     painter.setPen(pen)
                #     for each in self.sensor_fusion_error:
                #         # Make sure covariance parameters have been added
                #         if len(each) >= 3:
                #             pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                #                     self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
                #             a, b, phi = shared_math.ellipsify(each[2], 3.0)
                #             a = a * self.mapSpecs.meters_to_print_scale
                #             b = b * self.mapSpecs.meters_to_print_scale

                #             if not math.isnan(a) and not math.isnan(b):
                #                 # Save the previous painter envinronment so we don't mess up the other things
                #                 painter.save()
                #                 # get the x and y components of the ellipse position
                #                 ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                #                 ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                #                 # translate the center to where our ellipse should be
                #                 painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                #                 # Rotate by phi to tunr the ellipse the correct way
                #                 painter.rotate(math.degrees(phi))
                #                 # Draw the ellipse at 0.0
                #                 painter.drawEllipse(0, 0, a, b)
                #                 # Restore the environment to what it was before
                #                 painter.restore()

        # if self.drawCamera:
        #     for idx, cis in self.cis.items():
        #         pen = QPen()
        #         pen.setWidth(4)
        #         pen.setBrush(brush_color['camera'])
        #         painter.setPen(pen)
        #         # Draw the camera position
        #         painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale),
        #                         self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
        #                                     cis.wheelbaseLength * .5 * self.mapSpecs.meters_to_print_scale) * math.cos(
        #                             cis.theta)),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
        #                                     cis.wheelbaseLength * .5 * self.mapSpecs.meters_to_print_scale) * math.sin(
        #                             cis.theta)))
        #         painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
        #                     (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
        #             cis.theta + math.radians(90))),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
        #                                     (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
        #                             cis.theta + math.radians(90))),
        #                         self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
        #                                     (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
        #                             cis.theta + math.radians(90))),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
        #                                     (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
        #                             cis.theta + math.radians(90))))

        #     for idx, cis in self.cis.items():
        #         # Draw the FOV
        #         pen = QPen()
        #         pen.setWidth(.5)
        #         pen.setBrush(brush_color['camera_fov'])
        #         painter.setPen(pen)

        #         painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale),
        #                         self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
        #                                 1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
        #                             cis.theta + math.radians(180 + 80))),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
        #                                 1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
        #                             cis.theta + math.radians(180 + 80))))

        #         painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale),
        #                         self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
        #                                 1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
        #                             cis.theta + math.radians(180 + -80))),
        #                         self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
        #                                 1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
        #                             cis.theta + math.radians(180 + -80))))

        #     # Now draw the camera detections
        #     for idx, cis in self.cis.items():
        #         pen.setBrush(brush_color['camera_detection_centroid'])
        #         pen.setWidth(4)
        #         painter.setPen(pen)
        #         for each in cis.cameraDetections:
        #             #print ( each )
        #             #transX, transY = self.translateDetections(each[2], -each[1], math.atan2(-each[1], each[2]), cis.localizationPositionX, cis.localizationPositionY, cis.theta)
        #             #print ( transX, transY )
        #             #print ( cis.localizationPositionX, cis.localizationPositionY, cis.theta )
        #             #painter.drawPoint(self.translateX(transX * self.mapSpecs.meters_to_print_scale),
        #             #                  self.translateY(transY * self.mapSpecs.meters_to_print_scale))
        #             painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
        #                             self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

        #     if self.display_covariance:
        #         pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
        #         pen.setWidth(.5)
        #         painter.setPen(pen)
        #         for each in cis.cameraDetections:
        #             # Make sure covariance parameters have been added
        #             if len(each) >= 3:
        #                 pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
        #                 a, b, phi = shared_math.ellipsify(each[2].covariance, 3.0)
        #                 a = a * self.mapSpecs.meters_to_print_scale
        #                 b = b * self.mapSpecs.meters_to_print_scale
        #                 # Save the previous painter envinronment so we don't mess up the other things
        #                 painter.save()
        #                 # get the x and y components of the ellipse position
        #                 ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
        #                 ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
        #                 # translate the center to where our ellipse should be
        #                 painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
        #                 # Rotate by phi to tunr the ellipse the correct way
        #                 painter.rotate(math.degrees(phi))
        #                 # Draw the ellipse at 0.0
        #                 painter.drawEllipse(0, 0, a, b)
        #                 # Restore the environment to what it was before
        #                 painter.restore()

        # if self.display_localization:
        #     pen.setBrush(brush_color['localization_error'])
        #     pen.setWidth(.5)
        #     painter.setPen(pen)
        #     for idx, vehicle in self.vehicles.items():
        #         if localizationError != None:
        #             pos = ( self.translateX(localizationPositionX * self.mapSpecs.meters_to_print_scale),
        #                     self.translateY(localizationPositionY * self.mapSpecs.meters_to_print_scale) )
        #             a, b, phi = shared_math.ellipsify(localizationError.covariance, 3.0)
        #             a = a * self.mapSpecs.meters_to_print_scale
        #             b = b * self.mapSpecs.meters_to_print_scale
        #             # Save the previous painter envinronment so we don't mess up the other things
        #             painter.save()
        #             # get the x and y components of the ellipse position
        #             ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
        #             ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
        #             # translate the center to where our ellipse should be
        #             painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
        #             # Rotate by phi to tunr the ellipse the correct way
        #             painter.rotate(math.degrees(phi))
        #             # Draw the ellipse at 0.0
        #             painter.drawEllipse(0, 0, a, b)
        #             # Restore the environment to what it was before
        #             painter.restore()

        

        #     for idx, cis in self.cis.items():
        #         # Now draw the camera fusion detections
        #         pen.setBrush(brush_color['sensor_fusion_centroid'])
        #         pen.setWidth(6)
        #         painter.setPen(pen)
        #         for each in cis.fusionDetections:
        #             # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), localizationPositionX, localizationPositionY, theta)
        #             painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
        #                             self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))
        #         pen.setWidth(.5)
        #         painter.setPen(pen)
        #         for each in cis.fusionDetections:
        #             # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), localizationPositionX, localizationPositionY, theta)
        #             painter.drawLine(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
        #                             self.translateY(each[1] * self.mapSpecs.meters_to_print_scale),
        #                             self.translateX((each[0] + (8.0*each[3])) * self.mapSpecs.meters_to_print_scale),
        #                             self.translateY((each[1] + (8.0*each[4])) * self.mapSpecs.meters_to_print_scale))

        #         if self.display_covariance:
        #             pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
        #             pen.setWidth(.5)
        #             painter.setPen(pen)
        #             for each in cis.fusionDetections:
        #                 # Make sure covariance parameters have been added
        #                 if len(each) >= 3:
        #                     pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
        #                             self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
        #                     a, b, phi = shared_math.ellipsify(each[2], 3.0)
        #                     a = a * self.mapSpecs.meters_to_print_scale
        #                     b = b * self.mapSpecs.meters_to_print_scale
        #                     if not math.isnan(a) and not math.isnan(b):
        #                         # Save the previous painter envinronment so we don't mess up the other things
        #                         painter.save()
        #                         # get the x and y components of the ellipse position
        #                         ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
        #                         ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
        #                         # translate the center to where our ellipse should be
        #                         painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
        #                         # Rotate by phi to tunr the ellipse the correct way
        #                         painter.rotate(math.degrees(phi))
        #                         # Draw the ellipse at 0.0
        #                         painter.drawEllipse(0, 0, a, b)
        #                         # Restore the environment to what it was before
        #                         painter.restore()

        # if self.display_global_fusion:
        #     # Now draw the camera fusion detections
        #     pen.setBrush(brush_color['global_sensor_fusion_centroid'])
        #     pen.setWidth(6)
        #     painter.setPen(pen)
        #     for fuse in self.globalFusionList:
        #         painter.drawPoint(self.translateX(fuse[1] * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY(fuse[2] * self.mapSpecs.meters_to_print_scale))
        #     pen.setWidth(.5)
        #     painter.setPen(pen)
        #     for fuse in self.globalFusionList:
        #         painter.drawLine(self.translateX(fuse[1] * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY(fuse[2] * self.mapSpecs.meters_to_print_scale),
        #                         self.translateX((fuse[1] + (8.0*fuse[4])) * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY((fuse[2] + (8.0*fuse[5])) * self.mapSpecs.meters_to_print_scale))

        #     if self.display_covariance:
        #         pen.setBrush(brush_color['global_sensor_fusion_error_ellipse'])
        #         pen.setWidth(.5)
        #         painter.setPen(pen)
        #         for fuse in self.globalFusionList:
        #             # Make sure covariance parameters have been added
        #             if len(fuse) >= 4:
        #                 pos = ( self.translateX(fuse[1] * self.mapSpecs.meters_to_print_scale),
        #                         self.translateY(fuse[2] * self.mapSpecs.meters_to_print_scale) )
        #                 a, b, phi = shared_math.ellipsify(fuse[3], 3.0)
        #                 a = a * self.mapSpecs.meters_to_print_scale
        #                 b = b * self.mapSpecs.meters_to_print_scale
        #                 if not math.isnan(a) and not math.isnan(b):
        #                     # Save the previous painter envinronment so we don't mess up the other things
        #                     painter.save()
        #                     # get the x and y components of the ellipse position
        #                     ellipse_x_offset = math.cos(phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
        #                     ellipse_y_offset = math.sin(phi)*(a/2.0) + math.cos(phi)*(b/2.0)
        #                     # translate the center to where our ellipse should be
        #                     painter.translate(pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
        #                     # Rotate by phi to tunr the ellipse the correct way
        #                     painter.rotate(math.degrees(phi))
        #                     # Draw the ellipse at 0.0
        #                     painter.drawEllipse(0, 0, a, b)
        #                     # Restore the environment to what it was before
        #                     painter.restore()

        if self.drawTrafficLight:
            pen = QPen()

            # N/S direction
            pen.setWidth(4)
            if self.trafficLightArray[2] == 2:
                pen.setBrush(Qt.green)
            elif self.trafficLightArray[2] == 1:
                pen.setBrush(Qt.yellow)
            else:
                pen.setBrush(Qt.red)
            painter.setPen(pen)
            painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
            painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

            # E/W direction
            pen.setWidth(4)
            if self.trafficLightArray[1] == 2:
                pen.setBrush(Qt.green)
            elif self.trafficLightArray[1] == 1:
                pen.setBrush(Qt.yellow)
            else:
                pen.setBrush(Qt.red)
            painter.setPen(pen)
            painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
            painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                             self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))