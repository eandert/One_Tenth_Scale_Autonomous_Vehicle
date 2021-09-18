import time
import math
import sys
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import numpy as np

# For simulation
from connected_autonomous_vehicle.src import lidar_recognition, local_fusion
from connected_infrastructure_sensor.src import local_fusion as cis_local_fusion
from road_side_unit.src import global_fusion

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
    "lidar_detection_raw": Qt.lightGray,
    "camera_detection_centroid": Qt.darkYellow,
    "sensor_fusion_centroid": Qt.red,
    "sensor_fusion_error_ellipse": Qt.green
}


def angleDifference( angle1, angle2 ):
    diff = ( angle2 - angle1 + math.pi ) % (2*math.pi) - math.pi
    if diff < -math.pi:
        return diff + (2*math.pi)
    else:
        return diff

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def ellipsify(mu, covariance, meters_to_print_scale, num = 50, multiplier = 3.0):
    # Eigenvalue and eigenvector computations
    w, v = np.linalg.eig(covariance)

    # Use the eigenvalue to figure out which direction is larger
    if abs(w[0]) > abs(w[1]):
        a = abs(w[0])
        b = abs(w[1])
        phi = math.atan2(v[0, 0], v[1, 0])
    else:
        a = abs(w[1])
        b = abs(w[0])
        phi = math.atan2(v[0, 1], v[1, 1])

    # Default multiplier is 3 because that should cover 99.7% of errors
    print("a (ellipsify) ", str(a))
    print("b (ellipsify)", str(b))
    print("phi (ellipsify) ", str(math.degrees(phi)))
    pointEvery = math.radians(360.0)/num
    ellipse = QPolygonF()
    cur_angle = 0
    for count in range(num + 1):
        denominator = math.sqrt( a**2 * math.sin(phi-cur_angle)**2 + b**2 * math.cos(phi-cur_angle)**2 )
        if denominator == 0.0:
            print ( "Warning: calculateEllipseRadius denom 0! - check localizer definitions " )
            range_val = 0
        else:
            range_val = ( a * b ) / denominator
        print ( " m ", count, math.degrees(cur_angle), range_val )
        x_val = mu[0] + range_val * math.cos(cur_angle) * meters_to_print_scale
        y_val = mu[1] + range_val * math.sin(cur_angle) * meters_to_print_scale
        ellipse.append(QPointF(x_val, y_val))
        cur_angle += pointEvery

    return ellipse

def calcSelfRadiusAtAnlge(self, angle):
        a, b, phi = self.extractErrorElipseParamsFromBivariateGaussian()
        return self.calculateRadiusAtAngle(a, b, phi, angle)

class MainWindow(QMainWindow):
    def __init__(self, mapSpecs, vehiclesLock, cav, cis, trafficLightArray):
        print ( " GUI Started ")

        self.vehicles = cav
        self.cis = cis
        self.trafficLightArray = trafficLightArray
        self.pause_simulation = True
        self.vehiclesLock = vehiclesLock

        # Set to engage a full simulation world w/ no real vehicles and fake time
        self.full_simulation = False

        # Create the simulated LIDARs, planner, etc.
        self.lidarRecognitionList = []
        self.localFusionCAV = []
        self.localFusionCIS = [] = []
        self.globalFusion = global_fusion.GlobalFUSION()
        # Add in the arrays for local fusion storage for CAV vehicles
        for idx, veh in self.vehicles.items():
            if veh.simVehicle:
                self.lidarRecognitionList.append(lidar_recognition.LIDAR(0.0))
                self.localFusionCAV.append(local_fusion.FUSION())
            else:
                self.lidarRecognitionList.append(None)
                self.localFusionCAV.append(cis_local_fusion.FUSION())
        # Add in the arrays for local fusion storage for CIS sensors
        for idx, sens in self.cis.items():
            if sens.simCIS:
                self.localFusionCIS.append(local_fusion.FUSION())
            else:
                self.localFusionCIS.append(cis_local_fusion.FUSION())

        # Parameters of test
        self.mapSpecs = mapSpecs

        # Time params
        self.time = 0
        self.lastPositionUpdate = 0
        self.lastLocalizationUpdate = 0
        self.lightTime = 0

        # Draw params
        self.drawTrafficLight = False

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

        self.drawIntersection = True

        # Set this to true after we have set the coordinates set
        # as this will enable the GUI to draw.
        self.drawCoordinates = True

        self.labelVehicleSpeed = []
        self.lineVehicleSpeed = []
        self.labelVehicleSpeedActual = []
        self.labelVehicleSpeedTarget = []
        self.labelVehicleAcceleration = []

        self.vehiclesLock.acquire()
        for idx, vehicle in self.vehicles.items():
            self.labelVehicleSpeed.append(QLabel(self))
            self.labelVehicleSpeed[idx].setText('Speed Vehicle ' + str(idx) + ':')
            self.labelVehicleSpeed[idx].move(1000, 80 + 80 * idx)

            self.lineVehicleSpeed.append(QLineEdit(self))
            self.lineVehicleSpeed[idx].move(1000, 120 + 80 * idx)
            self.lineVehicleSpeed[idx].resize(100, 32)
            self.lineVehicleSpeed[idx].setText("0")

            self.labelVehicleSpeedActual.append(QLabel(self))
            self.labelVehicleSpeedActual[idx].setText('VA=0')
            self.labelVehicleSpeedActual[idx].move(1100, 120 + 80 * idx)

            self.labelVehicleSpeedTarget.append(QLabel(self))
            self.labelVehicleSpeedTarget[idx].setText('VT=0')
            self.labelVehicleSpeedTarget[idx].move(1150, 120 + 80 * idx)

            self.labelVehicleAcceleration.append(QLabel(self))
            self.labelVehicleAcceleration[idx].setText('AA=0')
            self.labelVehicleAcceleration[idx].move(1200, 120 + 80 * idx)

        self.vehiclesLock.release()

        self.phaseTarget = 1

        self.setVelocity = True

        self.drawVehicle = True

        self.drawCamera = True

        print ( " GUI Init end ")

        # Do this forevvvvveerrrrr!!!
        timer = QtCore.QTimer(self, timeout=self.stepTime, interval=25)
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
        if self.errorButton.text() == 'Simulate Error Off':
            self.simulate_error = True
            self.errorButton.setText('Simulate Error On')
        else:
            self.simulate_error = False
            self.errorButton.setText('Simulate Error Off')

    def on_estimate_covariance_clicked(self):
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

    def on_end_clicked(self):
        sys.exit()

    def stepTime(self):
        if self.full_simulation:
            self.time += 125
        else:
            self.time = round(time.time() * 1000)

        # 8HZ
        if self.full_simulation:
            if (self.time - self.lastPositionUpdate) >= 125:
                self.lastPositionUpdate = self.time
                self.vehiclesLock.acquire()
                for key, vehicle in self.vehicles.items():
                    # Update vehicle position based on physics
                    if vehicle.simVehicle:
                        vehicle.updatePosition(.125)
                self.vehiclesLock.release()
                self.drawTrafficLight = True

        # 8HZ
        if (self.time - self.lastLocalizationUpdate) >= 125:
            # print ( self.time )
            self.lastLocalizationUpdate = self.time
            # Traffic light update sequence
            if self.lightTime > self.mapSpecs.lightTimePeriod:
                self.lightTime = 0
                if self.trafficLightArray[1] == 2:
                    self.trafficLightArray[1] = 1
                    self.trafficLightArray[2] = 0
                    lightTimePeriod = 0 * 8
                elif self.trafficLightArray[2] == 2:
                    self.trafficLightArray[1] = 0
                    self.trafficLightArray[2] = 1
                    lightTimePeriod = 0 * 8
                elif self.trafficLightArray[1] == 1:
                    self.trafficLightArray[1] = 0
                    self.trafficLightArray[2] = 2
                    lightTimePeriod = 5 * 8
                elif self.trafficLightArray[2] == 1:
                    self.trafficLightArray[1] = 2
                    self.trafficLightArray[2] = 0
                    lightTimePeriod = 5 * 8
            else:
                self.lightTime += 1
            self.vehiclesLock.acquire()

            # Make the vehicle list before we move the vehicles
            # Get the last known location of all other vehicles
            vehicleList = []
            for otherIdx, otherVehicle in self.vehicles.items():
                vehicleList.append(otherVehicle.get_location())

            for idx, vehicle in self.vehicles.items():
                #print ( " CAV:", idx )
                # Do this if we are not in a full sim
                if not self.full_simulation and vehicle.simVehicle:
                    vehicle.updatePosition(.125)
                if self.pause_simulation:
                    # Make sure we relay the pause to our vehicles
                    vehicle.targetVelocityGeneral = 0.0
                elif self.lineVehicleSpeed[idx].text() == "" or self.lineVehicleSpeed[idx].text() == ".":
                    # Need to pass here in case the user is still typing
                    pass
                else:
                    vehicle.targetVelocityGeneral = float(self.lineVehicleSpeed[idx].text())
                if vehicle.simVehicle:
                    if self.pause_simulation:
                        vehicle.update_localization()
                        vehicle.distance_pid_control_overide = True
                        vehicle.targetVelocity = 0.0
                        vehicle.update_pid()
                    else:
                        # Update ourself
                        vehicle.update_localization()
                        vehicle.recieve_coordinate_group_commands(self.trafficLightArray)
                        vehicle.pure_pursuit_control()

                        # Filter out ourself
                        tempList = vehicleList.copy()
                        tempList.pop(idx)

                        if self.full_simulation:
                            # Create that fake LIDAR
                            if self.lidarRecognitionList[idx] != None:
                                point_cloud, point_cloud_error, camera_array, camera_error_array = vehicle.fake_lidar_and_camera(tempList, [], 15.0, 15.0, 0.0, 160.0)
                                if self.simulate_error:
                                    vehicle.cameraDetections = camera_error_array
                                    lidarcoordinates, lidartimestamp = self.lidarRecognitionList[idx].processLidarFrame(point_cloud_error, self.time/1000.0,
                                        vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, vehicle.lidarSensor)
                                    vehicle.rawLidarDetections = point_cloud_error
                                    vehicle.lidarDetections = lidarcoordinates
                                else:
                                    vehicle.cameraDetections = camera_array
                                    lidarcoordinates, lidartimestamp = self.lidarRecognitionList[idx].processLidarFrame(point_cloud, self.time/1000.0,
                                        vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, vehicle.lidarSensor)
                                    vehicle.rawLidarDetections = point_cloud
                                    vehicle.lidarDetections = lidarcoordinates

                                # Vehicle position can be the map centroid in sim
                                # because we are generating the detection WRT the centroid
                                #pos = [vehicle.localizationPositionX - vehicle.positionX_offset, vehicle.localizationPositionY - vehicle.positionY_offset, vehicle.theta - vehicle.theta_offset]
                                #pos = [0,0,0]

                                # Lets add the detections to the vehicle class
                                # vehicle.lidarDetections = []
                                # for each in lidarcoordinates:
                                #     new = rotate((0, 0), (float(each[1]), float(each[2])), pos[2])
                                #     sensed_x = new[0] + pos[0]
                                #     sensed_y = new[1] + pos[1]
                                #     vehicle.lidarDetections.append((sensed_x, sensed_y, each[6]))

                                # Raw LIDAR for debug
                                vehicle.lidarPoints = point_cloud

                                # Do the local fusion like we would on the vehicle
                                self.localFusionCAV[idx].processDetectionFrame(0, self.time/1000.0, vehicle.cameraDetections, .25, self.estimate_covariance)
                                self.localFusionCAV[idx].processDetectionFrame(1, self.time/1000.0, vehicle.lidarDetections, .25, self.estimate_covariance)
                                results = self.localFusionCAV[idx].fuseDetectionFrame(self.estimate_covariance, vehicle)

                                # Add to the GUI
                                vehicle.fusionDetections = []
                                for each in results:
                                    sensed_x = each[1]
                                    sensed_y = each[2]
                                    vehicle.fusionDetections.append((sensed_x, sensed_y, each[4], each[5]))
                                    vehicle.fusionDetectionsCovariance.append(each[3])

                        else:
                            # Quick fake of sensor values
                            vehicle.fusionDetections = []
                            for each in tempList:
                                sensed_x = each[0]
                                sensed_y = each[1]
                                vehicle.fusionDetections.append((sensed_x, sensed_y, 0.0, 0.0))
                                vehicle.fusionDetectionsCovariance.append(np.array([[1.0, 0],[0, 1.0]]))

                        # Now update our current PID with respect to other vehicles
                        vehicle.check_positions_of_other_vehicles_adjust_velocity(tempList)

                        # We can't update the PID controls until after all positions are known
                        vehicle.update_pid()

                        # Add to the global sensor fusion
                        self.globalFusion.processDetectionFrame(idx, self.time/1000.0, vehicle.fusionDetections, vehicle.fusionDetectionsCovariance, .25)

            for idx, cis in self.cis.items():
                #print ( " CIS:", idx )
                # Do this if we are not in a full sim
                if not self.full_simulation and cis.simCIS:
                    # CISs should not move but we can do this anyway just in case
                    cis.updatePosition(.125)
                if cis.simCIS:
                    if self.pause_simulation:
                        cis.update_localization()
                    else:
                        # Update ourself
                        cis.update_localization()

                        if self.full_simulation:
                            # Create that fake camera
                            if self.lidarRecognitionList[idx] != None:
                                camera_array, camera_error_array = cis.fake_camera(vehicleList, [], 15.0, 0.0, 160.0)
                                if self.simulate_error:
                                    cis.cameraDetections = camera_error_array
                                else:
                                    cis.cameraDetections = camera_array
                                
                                # f = open("data_" + str(idx) + ".txt", "a")
                                # f.write(str(idx) + "," + str(self.time))
                                # for each in cis.cameraDetections:
                                #     f.write("," + str(each[0]) + "," + str(each[1]))
                                # f.write("\n")
                                # f.close()

                                # CIS position can be the map centroid in sim
                                # because we are generating the detection WRT the centroid
                                # pos = [cis.localizationPositionX - cis.positionX_offset, cis.localizationPositionY - cis.positionY_offset, cis.theta - cis.theta_offset]
                                pos = [0, 0, 0]

                                # Fusion detection frame is the same as single camera (for now)
                                # Add to the GUI
                                # Do the local fusion like we would on the vehicle
                                self.localFusionCIS[idx].processDetectionFrame(0, self.time/1000.0, cis.cameraDetections, .25, self.estimate_covariance)
                                results = self.localFusionCIS[idx].fuseDetectionFrame(self.estimate_covariance, cis)

                                # Add to the GUI
                                cis.fusionDetections = []
                                for each in results:
                                    sensed_x = each[1]
                                    sensed_y = each[2]
                                    cis.fusionDetections.append((sensed_x, sensed_y, each[4], each[5]))
                                    cis.fusionDetectionsCovariance.append(each[3])
                        else:
                            # Quick fake of sensor values
                            cis.fusionDetections = []
                            for each in vehicleList:
                                sensed_x = each[0]
                                sensed_y = each[1]
                                cis.fusionDetections.append((sensed_x, sensed_y, 0, 0))
                                cis.fusionDetectionsCovariance.append(np.array([[1.0, 0.0],[0.0, 1.0]]))

                        # Add to the global sensor fusion
                        self.globalFusion.processDetectionFrame(1000+idx, self.time/1000.0, cis.fusionDetections, cis.fusionDetectionsCovariance, .25)

            self.vehiclesLock.release()

            self.drawTrafficLight = True

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
        angleLen = math.degrees(angleDifference(startAngle, endAngle))
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

            # self.drawVehicle = False

        if self.drawVehicle:
            for idx, vehicle in self.vehicles.items():
                pen = QPen()
                pen.setWidth(4)
                pen.setBrush(brush_color['vehicle'])
                painter.setPen(pen)

                # Draw the vehicle position
                # Front
                front_axle_offset_x = (vehicle.wheelbaseLength * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta)
                front_axle_offset_y = (vehicle.wheelbaseLength * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta)
                painter.drawLine(self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_x),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_y),
                                 self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_x),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_y))
                # Middle
                painter.drawLine(self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                 self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             vehicle.wheelbaseLength * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta)),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             vehicle.wheelbaseLength * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta)))
                # Back
                painter.drawLine(self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90))),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90))),
                                 self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90))),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90))))
                # Wheels
                driver_front_center_x = vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_x
                driver_front_center_y =  vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_y
                passenger_front_center_x = vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_x
                passenger_front_center_y =  vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90)) + front_axle_offset_y
                driver_rear_center_x = vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90))
                driver_rear_center_y =  vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90))
                passenger_rear_center_x = vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90))
                passenger_rear_center_y =  vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             (vehicle.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90))
                # Driver side front
                painter.drawLine(self.translateX(driver_front_center_x + 5 * math.cos(vehicle.theta + vehicle.steeringAcceleration)),
                                 self.translateY(driver_front_center_y + 5 * math.sin(vehicle.theta + vehicle.steeringAcceleration)),
                                 self.translateX(driver_front_center_x - 5 * math.cos(vehicle.theta + vehicle.steeringAcceleration)),
                                 self.translateY(driver_front_center_y - 5 * math.sin(vehicle.theta + vehicle.steeringAcceleration)))
                # Passenger side front
                painter.drawLine(self.translateX(passenger_front_center_x + 5 * math.cos(vehicle.theta + vehicle.steeringAcceleration)),
                                 self.translateY(passenger_front_center_y + 5 * math.sin(vehicle.theta + vehicle.steeringAcceleration)),
                                 self.translateX(passenger_front_center_x - 5 * math.cos(vehicle.theta + vehicle.steeringAcceleration)),
                                 self.translateY(passenger_front_center_y - 5 * math.sin(vehicle.theta + vehicle.steeringAcceleration)))
                # Driver side rear
                painter.drawLine(self.translateX(driver_rear_center_x + 5 * math.cos(vehicle.theta)),
                                 self.translateY(driver_rear_center_y + 5 * math.sin(vehicle.theta)),
                                 self.translateX(driver_rear_center_x - 5 * math.cos(vehicle.theta)),
                                 self.translateY(driver_rear_center_y - 5 * math.sin(vehicle.theta)))
                # Passenger side rear
                painter.drawLine(self.translateX(passenger_rear_center_x + 5 * math.cos(vehicle.theta)),
                                 self.translateY(passenger_rear_center_y + 5 * math.sin(vehicle.theta)),
                                 self.translateX(passenger_rear_center_x - 5 * math.cos(vehicle.theta)),
                                 self.translateY(passenger_rear_center_y - 5 * math.sin(vehicle.theta)))

                if self.path_debug:
                    # Draw the target point
                    pen.setBrush(brush_color['target_point'])
                    painter.setPen(pen)
                    painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * vehicle.targetIndexX),
                                    self.translateY(self.mapSpecs.meters_to_print_scale * vehicle.targetIndexY))
                    pen.setBrush(brush_color['wheel_angle'])
                    pen.setWidth(1)
                    painter.setPen(pen)
                    self.drawTargetArc(vehicle.centerPointX, vehicle.centerPointY, vehicle.localizationPositionX,
                                    vehicle.localizationPositionY, vehicle.targetIndexX, vehicle.targetIndexY, painter)

                self.labelVehicleSpeedActual[idx].setText('VA=' + str(round(vehicle.velocity, 2)))
                self.labelVehicleSpeedTarget[idx].setText('VT=' + str(round(vehicle.targetVelocity, 2)))
                self.labelVehicleAcceleration[idx].setText('AA=' + str(round(vehicle.motorAcceleration, 2)))

                if self.path_debug:
                    pen.setWidth(.5)
                    pen.setBrush(brush_color['bounding_box'])
                    pen.setWidth(4)
                    painter.setPen(pen)
                    buffer = 0.1
                    x1 = vehicle.localizationPositionX + (
                                (vehicle.width / 2 + buffer) * math.cos(vehicle.theta + math.radians(90)) + (
                                    (vehicle.wheelbaseLengthFromRear + buffer) * math.cos(
                                vehicle.theta - math.radians(180))))
                    y1 = vehicle.localizationPositionY + (
                                (vehicle.width / 2 + buffer) * math.sin(vehicle.theta + math.radians(90)) + (
                                    (vehicle.wheelbaseLengthFromRear + buffer) * math.sin(
                                vehicle.theta - math.radians(180))))
                    x2 = vehicle.localizationPositionX + (
                                (vehicle.width / 2 + buffer) * math.cos(vehicle.theta - math.radians(90)) + (
                                    (vehicle.wheelbaseLengthFromRear + buffer) * math.cos(
                                vehicle.theta - math.radians(180))))
                    y2 = vehicle.localizationPositionY + (
                                (vehicle.width / 2 + buffer) * math.sin(vehicle.theta - math.radians(90)) + (
                                    (vehicle.wheelbaseLengthFromRear + buffer) * math.sin(
                                vehicle.theta - math.radians(180))))
                    x3 = vehicle.localizationPositionX + (
                                (vehicle.width / 2 + buffer) * math.cos(vehicle.theta - math.radians(90)) + (
                                    (vehicle.length - vehicle.wheelbaseLengthFromRear + buffer) * math.cos(vehicle.theta)))
                    y3 = vehicle.localizationPositionY + (
                                (vehicle.width / 2 + buffer) * math.sin(vehicle.theta - math.radians(90)) + (
                                    (vehicle.length - vehicle.wheelbaseLengthFromRear + buffer) * math.sin(vehicle.theta)))
                    x4 = vehicle.localizationPositionX + (
                                (vehicle.width / 2 + buffer) * math.cos(vehicle.theta + math.radians(90)) + (
                                    (vehicle.length - vehicle.wheelbaseLengthFromRear + buffer) * math.cos(vehicle.theta)))
                    y4 = vehicle.localizationPositionY + (
                                (vehicle.width / 2 + buffer) * math.sin(vehicle.theta + math.radians(90)) + (
                                    (vehicle.length - vehicle.wheelbaseLengthFromRear + buffer) * math.sin(vehicle.theta)))

                    # print (vehicle.localizationPositionX, vehicle.localizationPositionY, x1, y1, x2, y2)
                    painter.drawPoint(self.translateX(x1 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y1 * self.mapSpecs.meters_to_print_scale))
                    painter.drawPoint(self.translateX(x2 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y2 * self.mapSpecs.meters_to_print_scale))
                    painter.drawPoint(self.translateX(x3 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y3 * self.mapSpecs.meters_to_print_scale))
                    painter.drawPoint(self.translateX(x4 * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(y4 * self.mapSpecs.meters_to_print_scale))

        if self.drawCamera:
            for idx, cis in self.cis.items():
                pen = QPen()
                pen.setWidth(4)
                pen.setBrush(brush_color['camera'])
                painter.setPen(pen)
                # Draw the camera position
                painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                            cis.wheelbaseLength * .5 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                    cis.theta)),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                            cis.wheelbaseLength * .5 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                    cis.theta)))
                painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                            (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                    cis.theta + math.radians(90))),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                            (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                    cis.theta + math.radians(90))),
                                self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                            (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                    cis.theta + math.radians(90))),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                            (cis.wheelbaseWidth/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                    cis.theta + math.radians(90))))

        if self.camera_debug:
            for idx, vehicle in self.vehicles.items():
                # Draw the FOV of cam
                pen.setWidth(.5)
                pen.setBrush(brush_color['camera_fov'])
                painter.setPen(pen)

                painter.drawLine(self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                    vehicle.theta + math.radians(180 + 80))),
                                self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                    vehicle.theta + math.radians(180 + 80))))

                painter.drawLine(self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                    vehicle.theta + math.radians(180 + -80))),
                                self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                    vehicle.theta + math.radians(180 + -80))))

            for idx, cis in self.cis.items():
                # Draw the FOV
                pen = QPen()
                pen.setWidth(.5)
                pen.setBrush(brush_color['camera_fov'])
                painter.setPen(pen)

                painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                    cis.theta + math.radians(180 + 80))),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                    cis.theta + math.radians(180 + 80))))

                painter.drawLine(self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                self.translateX(cis.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                    cis.theta + math.radians(180 + -80))),
                                self.translateY(cis.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                        1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                    cis.theta + math.radians(180 + -80))))

            # Now draw the vehicle camera detections
            for idx, vehicle in self.vehicles.items():
                pen.setBrush(brush_color['camera_detection_centroid'])
                pen.setWidth(4)
                painter.setPen(pen)
                for each in vehicle.cameraDetections:
                    # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta)
                    painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

            # if self.display_covariance:
            #     pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
            #     pen.setWidth(.5)
            #     painter.setPen(pen)
            #     for each in cis.cameraDetections:
            #         # Make sure covariance parameters have been added
            #         if len(each) >= 3:
            #             pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
            #                     self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
            #             ellipse = ellipsify(pos, each[2], self.mapSpecs.meters_to_print_scale, 50, 3.0)
            #             painter.drawPolygon(ellipse)

            # Now draw the camera detections
            for idx, cis in self.cis.items():
                pen.setBrush(brush_color['camera_detection_centroid'])
                pen.setWidth(4)
                painter.setPen(pen)
                for each in cis.cameraDetections:
                    #print ( each )
                    #transX, transY = self.translateDetections(each[2], -each[1], math.atan2(-each[1], each[2]), cis.localizationPositionX, cis.localizationPositionY, cis.theta)
                    #print ( transX, transY )
                    #print ( cis.localizationPositionX, cis.localizationPositionY, cis.theta )
                    #painter.drawPoint(self.translateX(transX * self.mapSpecs.meters_to_print_scale),
                    #                  self.translateY(transY * self.mapSpecs.meters_to_print_scale))
                    painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

            # if self.display_covariance:
            #     pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
            #     pen.setWidth(.5)
            #     painter.setPen(pen)
            #     for each in cis.cameraDetections:
            #         # Make sure covariance parameters have been added
            #         if len(each) >= 3:
            #             pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
            #                     self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
            #             ellipse = ellipsify(pos, each[2], self.mapSpecs.meters_to_print_scale, 50, 3.0)
            #             painter.drawPolygon(ellipse)

        if self.lidar_debug:
            for idx, vehicle in self.vehicles.items():
                # Now draw the vehicle lidar detections
                pen.setBrush(brush_color['lidar_detection_centroid'])
                pen.setWidth(4)
                painter.setPen(pen)
                for each in vehicle.lidarDetections:
                    # print ( each )
                    # transX, transY = self.translateDetections(each[1], each[2], math.atan2(each[2], each[1]), vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta)
                    painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

                pen.setBrush(brush_color['lidar_detection_raw'])
                pen.setWidth(2)
                painter.setPen(pen)
                for each in vehicle.rawLidarDetections:
                    painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))

            if self.display_covariance:
                pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
                pen.setWidth(.5)
                painter.setPen(pen)
                for each in vehicle.lidarDetections:
                    # Make sure covariance parameters have been added
                    if len(each) >= 3:
                        print(each[2].covariance)
                        pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
                        ellipse = ellipsify(pos, each[2].covariance, self.mapSpecs.meters_to_print_scale, 50, 3.0)
                        painter.drawPolygon(ellipse)

        if self.fusion_debug:
            for idx, vehicle in self.vehicles.items():
                # Now draw the vehicle fusion detections
                pen.setBrush(brush_color['sensor_fusion_centroid'])
                pen.setWidth(6)
                painter.setPen(pen)
                for each in vehicle.fusionDetections:
                    # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta)
                    painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))
                pen.setWidth(.5)
                painter.setPen(pen)
                for each in vehicle.fusionDetections:
                    # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta)
                    painter.drawLine(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale),
                                    self.translateX((each[0] + (8.0*each[2])) * self.mapSpecs.meters_to_print_scale),
                                    self.translateY((each[1] + (8.0*each[3])) * self.mapSpecs.meters_to_print_scale))

                if self.display_covariance:
                    pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
                    pen.setWidth(.5)
                    painter.setPen(pen)
                    for covariance, each in zip(vehicle.fusionDetectionsCovariance, vehicle.fusionDetections):
                        print ( covariance )
                        pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
                        ellipse = ellipsify(pos, covariance, self.mapSpecs.meters_to_print_scale, 50, 3.0)
                        painter.drawPolygon(ellipse)

            for idx, cis in self.cis.items():
                # Now draw the camera fusion detections
                pen.setBrush(brush_color['sensor_fusion_centroid'])
                pen.setWidth(6)
                painter.setPen(pen)
                for each in cis.fusionDetections:
                    # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta)
                    painter.drawPoint(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale))
                pen.setWidth(.5)
                painter.setPen(pen)
                for each in cis.fusionDetections:
                    # transX, transY = self.translateDetections(each[1],  abs(each[2]), math.atan2(abs(each[2]), each[1]), vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta)
                    painter.drawLine(self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                    self.translateY(each[1] * self.mapSpecs.meters_to_print_scale),
                                    self.translateX((each[0] + (8.0*each[2])) * self.mapSpecs.meters_to_print_scale),
                                    self.translateY((each[1] + (8.0*each[3])) * self.mapSpecs.meters_to_print_scale))

                if self.display_covariance:
                    pen.setBrush(brush_color['sensor_fusion_error_ellipse'])
                    pen.setWidth(.5)
                    painter.setPen(pen)
                    for covariance, each in zip(cis.fusionDetectionsCovariance, cis.fusionDetections):
                        pos = ( self.translateX(each[0] * self.mapSpecs.meters_to_print_scale),
                                self.translateY(each[1] * self.mapSpecs.meters_to_print_scale) )
                        ellipse = ellipsify(pos, covariance, self.mapSpecs.meters_to_print_scale, 50, 3.0)
                        painter.drawPolygon(ellipse)

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