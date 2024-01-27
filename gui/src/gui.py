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
    "traffic_light_autonomous": Qt.lightGray,
    "waypoint": Qt.darkGray,
    "target_point": Qt.darkRed,
    "wheel_angle": Qt.red,
    "bounding_box": Qt.gray,
    "lidar_detection_centroid": Qt.cyan,
    "lidar_detection_error": Qt.cyan,
    "lidar_detection_raw": Qt.lightGray,
    "camera_detection_centroid": Qt.darkYellow,
    "camera_detection_error": Qt.darkYellow,
    "localization": Qt.yellow,
    "localization_error": Qt.yellow,
    "sensor_fusion_centroid": Qt.red,
    "sensor_fusion_error_ellipse": Qt.red,
    "global_sensor_fusion_centroid": Qt.darkGreen,
    "global_sensor_fusion_error_ellipse": Qt.darkGreen
}


class MainWindow(QMainWindow):
    def __init__(self, config):
        print(" GUI Started ")

        self.rsu_connection = communication.connectServer(config.rsu_ip)
        init_response = self.rsu_connection.getGuiValues(True)

        self.mapSpecs = mapGenerator.MapSpecs(
            init_response['map_specs'][0], init_response['map_specs'][1])

        self.vehicles = init_response['vehicle']
        self.camera_fov = init_response['camera_fov']
        self.camera_center = init_response['camera_center']
        self.lidar_detection_centroid = init_response['lidar_detection_centroid']
        self.camera_detection_centroid = init_response['camera_detection_centroid']
        self.sensor_fusion_centroid = init_response['sensor_fusion_centroid']
        self.localization_centroid = init_response['localization_centroid']
        self.localization_error = init_response['localization_error']

        self.sensors = init_response['sensor']
        self.sensor_camera_fov = init_response['sensor_camera_fov']
        self.sensor_camera_center = init_response['sensor_camera_center']
        self.sensor_camera_detection_centroid = init_response['sensor_camera_detection_centroid']
        self.sensor_sensor_fusion_centroid = init_response['sensor_sensor_fusion_centroid']
        self.sensor_localization_error = init_response['sensor_localization_error']

        self.global_sensor_fusion_centroid = init_response['global_sensor_fusion_centroid']
        self.trafficLightArray = init_response['traffic_light']

        # # Set to engage a full simulation world w/ no real vehicles and fake time
        self.full_simulation = False

        self.end_simulation = False
        self.pause_simulation = True
        self.last_pause_simulation = True

        # Draw params
        self.drawTrafficLight = False
        self.display_localization = True
        self.intersection_mode = 0

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

        self.errorButton = QPushButton('Simulate Error Off', self)
        self.errorButton.resize(140, 32)
        self.errorButton.move(1000, 350)
        self.errorButton.clicked.connect(self.on_simulate_error_clicked)
        self.simulate_error = False

        self.covarianceButton = QPushButton(
            'Estimate Local Covariance Off', self)
        self.covarianceButton.resize(140, 32)
        self.covarianceButton.move(1000, 390)
        self.covarianceButton.clicked.connect(
            self.on_parameterized_covariance_clicked)
        self.parameterized_covariance = False

        self.pathButton = QPushButton('Path Debug Off', self)
        self.pathButton.resize(140, 32)
        self.pathButton.move(1000, 430)
        self.pathButton.clicked.connect(self.on_path_clicked)
        self.path_debug = False

        self.lidarButton = QPushButton('LIDAR Debug Off', self)
        self.lidarButton.resize(140, 32)
        self.lidarButton.move(1000, 470)
        self.lidarButton.clicked.connect(self.on_lidar_clicked)
        self.lidar_debug = False

        self.cameraButton = QPushButton('Camera Debug Off', self)
        self.cameraButton.resize(140, 32)
        self.cameraButton.move(1000, 510)
        self.cameraButton.clicked.connect(self.on_camera_clicked)
        self.camera_debug = False

        self.fusionButton = QPushButton('Fusion Local Debug Off', self)
        self.fusionButton.resize(140, 32)
        self.fusionButton.move(1000, 550)
        self.fusionButton.clicked.connect(self.on_fusion_clicked)
        self.fusion_debug = False

        self.covarianceDisplayButton = QPushButton(
            'Display Covariance Off', self)
        self.covarianceDisplayButton.resize(140, 32)
        self.covarianceDisplayButton.move(1000, 590)
        self.covarianceDisplayButton.clicked.connect(
            self.on_display_covariance_clicked)
        self.display_covariance = False

        # self.covarianceDisplayButton = QPushButton('Display Global Fusion Off', self)
        # self.covarianceDisplayButton.resize(140, 32)
        # self.covarianceDisplayButton.move(1000, 590)
        # self.covarianceDisplayButton.clicked.connect(self.on_display_covariance_clicked)
        self.display_global_fusion = True

        self.testGroup = QButtonGroup(self)  # Radio button group

        self.radioTrafficLight = QRadioButton("Traffic Light", self)
        self.radioTrafficLight.resize(200, 32)
        self.radioTrafficLight.move(1000, 630)
        self.radioTrafficLight.clicked.connect(self.on_intersection_clicked)
        self.radioTrafficLight.toggle()  # start in traffic test
        self.testGroup.addButton(self.radioTrafficLight)

        self.radioAutonomousIntersection = QRadioButton(
            "Autonomous Intersection", self)
        self.radioAutonomousIntersection.resize(200, 32)
        self.radioAutonomousIntersection.move(1000, 660)
        self.radioAutonomousIntersection.clicked.connect(
            self.on_intersection_clicked)
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
        self.unit_test = config.unit_test

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
            self.labelVehicleSpeed[idx].setText(
                'Speed Vehicle ' + str(idx) + ':')
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

        # Print out the legend for all the colors used
        self.labelList = []
        self.labelList.append(QLabel(self))
        self.labelList[0].setText("Legend:")
        self.labelList[0].move(5, 5)
        for idx, brush in enumerate(brush_color):
            self.labelList.append(QLabel(self))
            self.labelList[idx].setAutoFillBackground(
                True)  # This is important!!
            color = QColor(brush_color[brush])
            alpha = 140
            values = "{r}, {g}, {b}, {a}".format(r=color.red(),
                                                 g=color.green(),
                                                 b=color.blue(),
                                                 a=alpha
                                                 )
            self.labelList[idx].setStyleSheet(
                "QLabel { background-color: rgba("+values+"); }")
            self.labelList[idx].setText(brush)
            self.labelList[idx].resize(200, 20)
            self.labelList[idx].move(5, 25 + (idx * 20))

        self.setVelocity = True

        self.drawVehicle = True

        self.drawCamera = True

        self.button_states = dict(
            full_simulation=self.full_simulation,
            simulate_error=self.simulate_error,
            parameterized_covariance=self.parameterized_covariance,
            path_debug=self.path_debug,
            lidar_debug=self.lidar_debug,
            camera_debug=self.camera_debug,
            fusion_debug=self.fusion_debug,
            display_covariance=self.display_covariance,
            unit_test=self.unit_test,
            intersection_mode=self.intersection_mode
        )

        if config.unit_test:
            print("Unit test in progress")
            for idx, vehicle in enumerate(self.vehicles):
                self.lineVehicleSpeed[idx].setText(
                    str(config.unit_test_speed_target))
                self.lineVehicleSpeed[idx].setReadOnly(True)
            self.sensorsButton.setEnabled(False)
            self.errorButton.setEnabled(False)
            self.covarianceButton.setEnabled(False)
            self.endButton.setEnabled(True)
            self.pauseButton.setEnabled(False)
            self.startButton.setEnabled(False)
            self.unitTestButton.setEnabled(False)

        print(" GUI Init end ")

        # Do this forevvvvveerrrrr!!!
        timer = QtCore.QTimer(self, timeout=self.stepTime,
                              interval=config.gui_interval)
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

    def on_parameterized_covariance_clicked(self):
        if not self.unit_test:
            if self.covarianceButton.text() == 'Estimate Local Covariance Off':
                self.parameterized_covariance = True
                self.covarianceButton.setText('Estimate Local Covariance On')
            else:
                self.parameterized_covariance = False
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

    def on_intersection_clicked(self):
        if self.radioAutonomousIntersection.isChecked():
            self.intersection_mode = 1
        else:
            self.intersection_mode = 0

    def stepTime(self):
        response = self.rsu_connection.getGuiValues(False)

        if len(response) > 5:

            self.vehicles = response['vehicle']
            self.camera_fov = response['camera_fov']
            self.camera_center = response['camera_center']
            self.lidar_detection_centroid = response['lidar_detection_centroid']
            self.camera_detection_centroid = response['camera_detection_centroid']
            self.sensor_fusion_centroid = response['sensor_fusion_centroid']
            self.localization_error = response['localization_error']

            self.sensors = response['sensor']
            self.sensor_camera_fov = response['sensor_camera_fov']
            self.sensor_camera_center = response['sensor_camera_center']
            self.sensor_camera_detection_centroid = response['sensor_camera_detection_centroid']
            self.sensor_sensor_fusion_centroid = response['sensor_sensor_fusion_centroid']
            self.localization_centroid = response['localization_centroid']
            self.sensor_localization_error = response['sensor_localization_error']

            self.global_sensor_fusion_centroid = response['global_sensor_fusion_centroid']
            self.trafficLightArray = response['traffic_light']
            self.lidar_detection_raw = response['lidar_detection_raw']

            if response['end_test'] == True:
                exit(0)

            self.drawIntersection = True
            self.drawCoordinates = True
            self.drawTrafficLight = True

        self.drawTrafficLight = True

        # Check if our button have chagned, and if so create a new packet
        button_states_changed = False
        if self.button_states['full_simulation'] != self.full_simulation or self.button_states['simulate_error'] != self.simulate_error or self.button_states['parameterized_covariance'] != self.parameterized_covariance or self.button_states['intersection_mode'] != self.intersection_mode:
            self.button_states = dict(
                full_simulation=self.full_simulation,
                simulate_error=self.simulate_error,
                parameterized_covariance=self.parameterized_covariance,
                path_debug=self.path_debug,
                lidar_debug=self.lidar_debug,
                camera_debug=self.camera_debug,
                fusion_debug=self.fusion_debug,
                display_covariance=self.display_covariance,
                unit_test=self.unit_test,
                intersection_mode=self.intersection_mode
            )
            button_states_changed = True

        # Chekc if our speed targets have chagned and if so create new packets
        new_speed_target = False
        for idx, vehicle in enumerate(self.vehicles):
            if not (self.lineVehicleSpeed[idx].text() == "" or self.lineVehicleSpeed[idx].text() == "."):
                if float(self.lineVehicleSpeed[idx].text()) != self.last_line_vehicle_speed[idx]:
                    new_speed_target = True
                    self.last_line_vehicle_speed[idx] = float(
                        self.lineVehicleSpeed[idx].text())

        # Check if the start, pause, end, or any other button has changed and if so send all new button states to the RSU
        if self.pause_simulation != self.last_pause_simulation or self.end_simulation or new_speed_target or button_states_changed:
            self.rsu_connection.sendGuiValues(
                self.last_line_vehicle_speed, self.pause_simulation, self.end_simulation, self.button_states)
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
        return (x * math.cos(theta + sens_theta)) - (y * math.sin(theta + sens_theta)) + sens_x, (x * math.sin(theta + sens_theta)) + (y * math.cos(theta + sens_theta)) + sens_y

    def drawTargetArc(self, x0, y0, x1, y1, x2, y2, painter):
        r = math.hypot(x1 - x0, y1 - y0)
        x = x0 - r
        y = y0 + r
        width = 2 * r
        height = 2 * r
        startAngle = math.atan2(y1 - y0, x1 - x0)
        endAngle = math.atan2(y2 - y0, x2 - x0)
        angleLen = math.degrees(
            shared_math.angleDifference(startAngle, endAngle))
        startAngle = math.degrees(startAngle)
        painter.drawArc(self.translateX(x * self.mapSpecs.meters_to_print_scale),
                        self.translateY(
                            y * self.mapSpecs.meters_to_print_scale), width * self.mapSpecs.meters_to_print_scale,
                        height * self.mapSpecs.meters_to_print_scale, startAngle * 16, angleLen * 16)

    def paintEvent(self, event):
        painter = QPainter(self)
        pen = QPen()

        try:

            ### DRAW INTERSECTION ###
            if self.drawIntersection:
                self.paint_lane_lines(pen, painter)

            ### DRAW TRAFFIC LIGHT STATE ###
            if self.drawCoordinates:
                self.draw_tfl_and_waypoints(pen, painter)

            ### DRAW CAVS ###
            if self.drawVehicle:
                self.paint_vehicles(pen, painter)

            ### DRAW CISS ###
            if self.drawCamera:
                self.paint_sensors(pen, painter)

            ### GLOBAL FUSION ###
            if self.display_global_fusion:
                self.paint_global_fusion(pen, painter)

            # if self.display_cooperative_monitoring:
            #     self.paint_cooperative_monitor(pen, painter)

        except Exception as e:
            print(" GUI render error: ", str(e))

    def draw_tfl_and_waypoints(self, pen, painter):
        pen.setWidth(2)

        for x, y, intersection in zip(self.mapSpecs.x_coordinates, self.mapSpecs.y_coordinates, self.mapSpecs.velocity_coordinates):
            # painter.translate(self.mapSpecs.centerX-15, self.mapSpecs.centerY-30);
            # painter.rotate(90);

            if intersection == 0:
                # No inersection here
                pen.setBrush(brush_color['waypoint'])
                pen.setWidth(2)
            elif self.trafficLightArray[intersection] == 2:
                pen.setWidth(4)
                if self.intersection_mode == 0:
                    pen.setBrush(brush_color['traffic_light_green'])
                else:
                    pen.setBrush(brush_color['traffic_light_autonomous'])
            elif self.trafficLightArray[intersection] == 1:
                pen.setWidth(4)
                if self.intersection_mode == 0:
                    pen.setBrush(brush_color['traffic_light_yellow'])
                else:
                    pen.setBrush(brush_color['traffic_light_autonomous'])
            else:
                pen.setWidth(4)
                if self.intersection_mode == 0:
                    pen.setBrush(brush_color['traffic_light_red'])
                else:
                    pen.setBrush(brush_color['traffic_light_autonomous'])
            painter.setPen(pen)
            painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * x),
                              self.translateY(self.mapSpecs.meters_to_print_scale * y))

    def paint_lane_lines(self, pen, painter):
        pen = QPen()

        # N/S direction
        pen.setWidth(4)
        if self.intersection_mode == 1:
            pen.setBrush(brush_color['traffic_light_autonomous'])
        elif self.trafficLightArray[2] == 2:
            pen.setBrush(brush_color['traffic_light_green'])
        elif self.trafficLightArray[2] == 1:
            pen.setBrush(brush_color['traffic_light_yellow'])
        else:
            pen.setBrush(brush_color['traffic_light_red'])
        painter.setPen(pen)
        painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
        painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

        # E/W direction
        pen.setWidth(4)
        if self.intersection_mode == 1:
            pen.setBrush(brush_color['traffic_light_autonomous'])
        elif self.trafficLightArray[1] == 2:
            pen.setBrush(brush_color['traffic_light_green'])
        elif self.trafficLightArray[1] == 1:
            pen.setBrush(brush_color['traffic_light_yellow'])
        else:
            pen.setBrush(brush_color['traffic_light_red'])
        painter.setPen(pen)
        painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
        painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

        painter.setPen(Qt.black)

        # Draw top section
        painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY - (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) - (
            self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
        painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY - (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) - (
            self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

        # Draw bottom section
        painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY + (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) + (
            self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
        painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY + (self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale) + (
            self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

        # Draw left section
        painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) - (
            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
            self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
        painter.drawLine(self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) - (
            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
            self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

        # Draw right section
        painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY -
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) + (
            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
            self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))
        painter.drawLine(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerY +
                         (self.mapSpecs.intersectionWidth *
                          self.mapSpecs.meters_to_print_scale / 2),
                         self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) + (
            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale),
            self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2))

        # Draw top right 3/4 circle
        painter.drawArc(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                        self.mapSpecs.centerY - (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2) - (
            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2),
            self.mapSpecs.intersectionStraightLength *
            self.mapSpecs.meters_to_print_scale * 2,
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
            -self.mapSpecs.intersectionStraightLength *
            self.mapSpecs.meters_to_print_scale * 2,
            -self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2, 90 * 16, 270 * 16)
        painter.drawArc(self.mapSpecs.centerX + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale / 2),
                        self.mapSpecs.centerY + (self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 1.5) + (
            self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2),
            -self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2 - (
            self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 2),
            -self.mapSpecs.intersectionStraightLength * self.mapSpecs.meters_to_print_scale * 2 - (
            self.mapSpecs.intersectionWidth * self.mapSpecs.meters_to_print_scale * 2), 90 * 16, 270 * 16)

    def paint_vehicles(self, pen, painter):
        for idx, vehicle in enumerate(self.vehicles):
            pen = QPen()
            pen.setWidth(4)
            pen.setBrush(brush_color['vehicle'])
            painter.setPen(pen)

            global_position_x = vehicle[0]
            global_position_y = vehicle[1]
            theta = vehicle[2]
            velocity = vehicle[3]
            wheelbase_width = vehicle[4]
            wheelbaseLength = vehicle[5]
            steering_acceleration = vehicle[6]
            target_index_x = vehicle[7]
            target_index_y = vehicle[8]
            rear_axle_x = vehicle[9]
            rear_axle_y = vehicle[10]
            target_velocity = vehicle[11]
            motor_acceleration = vehicle[12]
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
            painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale + (
                                            (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(90))) + front_axle_offset_x,
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale + (
                    (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                    theta + math.radians(90)) + front_axle_offset_y),
                self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale - (
                    (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                    theta + math.radians(90))) + front_axle_offset_x,
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale - (
                    (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                    theta + math.radians(90)) + front_axle_offset_y))
            # Middle
            painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale + (
                wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(180))),
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale + (
                    wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                    theta + math.radians(180))),
                self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale + (
                    wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                    theta)),
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale + (
                    wheelbaseLength/2.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                    theta)))
            # Back
            painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale + (
                                            (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(90))) + rear_axle_offset_x,
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale + (
                    (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                    theta + math.radians(90)) + rear_axle_offset_y),
                self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale - (
                    (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                    theta + math.radians(90))) + rear_axle_offset_x,
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale - (
                    (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                    theta + math.radians(90)) + rear_axle_offset_y))

            # Wheels
            driver_front_center_x = global_position_x * self.mapSpecs.meters_to_print_scale + (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(90)) + front_axle_offset_x
            driver_front_center_y = global_position_y * self.mapSpecs.meters_to_print_scale + (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                theta + math.radians(90)) + front_axle_offset_y
            passenger_front_center_x = global_position_x * self.mapSpecs.meters_to_print_scale - (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(90)) + front_axle_offset_x
            passenger_front_center_y = global_position_y * self.mapSpecs.meters_to_print_scale - (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                theta + math.radians(90)) + front_axle_offset_y
            driver_rear_center_x = global_position_x * self.mapSpecs.meters_to_print_scale - (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(90)) + rear_axle_offset_x
            driver_rear_center_y = global_position_y * self.mapSpecs.meters_to_print_scale - (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                theta + math.radians(90)) + rear_axle_offset_y
            passenger_rear_center_x = global_position_x * self.mapSpecs.meters_to_print_scale + (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(90)) + rear_axle_offset_x
            passenger_rear_center_y = global_position_y * self.mapSpecs.meters_to_print_scale + (
                (wheelbase_width/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                theta + math.radians(90)) + rear_axle_offset_y

            # Driver side front
            painter.drawLine(self.translateX(driver_front_center_x + 5 * math.cos(theta + steering_acceleration)),
                             self.translateY(
                                 driver_front_center_y + 5 * math.sin(theta + steering_acceleration)),
                             self.translateX(
                                 driver_front_center_x - 5 * math.cos(theta + steering_acceleration)),
                             self.translateY(driver_front_center_y - 5 * math.sin(theta + steering_acceleration)))
            # Passenger side front
            painter.drawLine(self.translateX(passenger_front_center_x + 5 * math.cos(theta + steering_acceleration)),
                             self.translateY(
                                 passenger_front_center_y + 5 * math.sin(theta + steering_acceleration)),
                             self.translateX(
                                 passenger_front_center_x - 5 * math.cos(theta + steering_acceleration)),
                             self.translateY(passenger_front_center_y - 5 * math.sin(theta + steering_acceleration)))
            # Driver side rear
            painter.drawLine(self.translateX(driver_rear_center_x + 5 * math.cos(theta)),
                             self.translateY(
                                 driver_rear_center_y + 5 * math.sin(theta)),
                             self.translateX(
                                 driver_rear_center_x - 5 * math.cos(theta)),
                             self.translateY(driver_rear_center_y - 5 * math.sin(theta)))
            # Passenger side rear
            painter.drawLine(self.translateX(passenger_rear_center_x + 5 * math.cos(theta)),
                             self.translateY(
                                 passenger_rear_center_y + 5 * math.sin(theta)),
                             self.translateX(
                                 passenger_rear_center_x - 5 * math.cos(theta)),
                             self.translateY(passenger_rear_center_y - 5 * math.sin(theta)))

            if self.path_debug:
                # Draw the target point
                pen.setBrush(brush_color['target_point'])
                painter.setPen(pen)
                painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * target_index_x),
                                  self.translateY(self.mapSpecs.meters_to_print_scale * target_index_y))
                pen.setBrush(brush_color['wheel_angle'])
                pen.setWidth(1)
                painter.setPen(pen)
                self.drawTargetArc(global_position_x, global_position_y, (rear_axle_offset_x+driver_rear_center_x)/2,
                                   (rear_axle_offset_y+driver_rear_center_y)/2, target_index_x, target_index_y, painter)

            self.labelVehicleSpeedActual[idx].setText(
                'VA=' + str(round(velocity, 2)))
            self.labelVehicleSpeedTarget[idx].setText(
                'VT=' + str(round(target_velocity, 2)))
            self.labelVehicleAcceleration[idx].setText(
                'AA=' + str(round(motor_acceleration, 2)))

            if self.path_debug:
                pen.setWidth(.5)
                pen.setBrush(brush_color['bounding_box'])
                pen.setWidth(4)
                painter.setPen(pen)
                buffer = 0.1
                x1 = global_position_x + (
                    (width/2.0 + buffer) * math.cos(theta + math.radians(90)) + (
                        (length/2.0 + buffer) * math.cos(theta - math.radians(180))))
                y1 = global_position_y + (
                    (width/2.0 + buffer) * math.sin(theta + math.radians(90)) + (
                        (length/2.0 + buffer) * math.sin(theta - math.radians(180))))
                x2 = global_position_x + (
                    (width/2.0 + buffer) * math.cos(theta - math.radians(90)) + (
                        (length/2.0 + buffer) * math.cos(theta - math.radians(180))))
                y2 = global_position_y + (
                    (width/2.0 + buffer) * math.sin(theta - math.radians(90)) + (
                        (length/2.0 + buffer) * math.sin(theta - math.radians(180))))
                x3 = global_position_x + (
                    (width/2.0 + buffer) * math.cos(theta - math.radians(90)) + (
                        (length/2.0 + buffer) * math.cos(theta)))
                y3 = global_position_y + (
                    (width/2.0 + buffer) * math.sin(theta - math.radians(90)) + (
                        (length/2.0 + buffer) * math.sin(theta)))
                x4 = global_position_x + (
                    (width/2.0 + buffer) * math.cos(theta + math.radians(90)) + (
                        (length/2.0 + buffer) * math.cos(theta)))
                y4 = global_position_y + (
                    (width/2.0 + buffer) * math.sin(theta + math.radians(90)) + (
                        (length/2.0 + buffer) * math.sin(theta)))

                # print (global_position_x, global_position_y, x1, y1, x2, y2)
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

                camera_center = self.camera_center[idx]
                camera_fov = self.camera_fov[idx]

                painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale),
                                 self.translateY(
                                     global_position_y * self.mapSpecs.meters_to_print_scale),
                                 self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + camera_center - (camera_fov/2))),
                                 self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + camera_center - (camera_fov/2))))

                painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale),
                                 self.translateY(
                                     global_position_y * self.mapSpecs.meters_to_print_scale),
                                 self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + camera_center + (camera_fov/2))),
                                 self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + camera_center + (camera_fov/2))))

                # Now draw the vehicle camera detections
                pen.setBrush(brush_color['camera_detection_centroid'])
                pen.setWidth(4)
                painter.setPen(pen)
                for each in self.camera_detection_centroid[idx]:
                    painter.drawPoint(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                      self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))

                # Time for displaying the covaraince
                if self.display_covariance:
                    pen.setBrush(brush_color['camera_detection_error'])
                    pen.setWidth(.5)
                    painter.setPen(pen)
                    for each in self.camera_detection_centroid[idx]:
                        # Make sure covariance parameters have been added
                        if len(each) >= 3:
                            pos = (self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                   self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
                            a, b, phi = shared_math.ellipsify(each[3], 3.0)
                            a = a * self.mapSpecs.meters_to_print_scale
                            b = b * self.mapSpecs.meters_to_print_scale
                            # Save the previous painter envinronment so we don't mess up the other things
                            painter.save()
                            # get the x and y components of the ellipse position
                            ellipse_x_offset = math.cos(
                                phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                            ellipse_y_offset = math.sin(
                                phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                            # translate the center to where our ellipse should be
                            painter.translate(
                                pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
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
                    painter.drawPoint(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                      self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))

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
                            pos = (self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                   self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
                            a, b, phi = shared_math.ellipsify(each[3], 3.0)
                            a = a * self.mapSpecs.meters_to_print_scale
                            b = b * self.mapSpecs.meters_to_print_scale
                            # Save the previous painter envinronment so we don't mess up the other things
                            painter.save()
                            # get the x and y components of the ellipse position
                            ellipse_x_offset = math.cos(
                                phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                            ellipse_y_offset = math.sin(
                                phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                            # translate the center to where our ellipse should be
                            painter.translate(
                                pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                            # Rotate by phi to turn the ellipse the correct way
                            painter.rotate(math.degrees(phi))
                            # Draw the ellipse at 0.0
                            painter.drawEllipse(0, 0, a, b)
                            # Restore the environment to what it was before
                            painter.restore()

            if self.display_localization:
                pen.setBrush(brush_color['localization_error'])
                if self.localization_centroid != []:
                    pen.setWidth(6)
                    painter.setPen(pen)
                    painter.drawPoint(self.translateX(self.localization_centroid[idx][0] * self.mapSpecs.meters_to_print_scale),
                                      self.translateY(self.localization_centroid[idx][1] * self.mapSpecs.meters_to_print_scale))
                if self.localization_error != [] and self.localization_centroid != []:
                    pen.setWidth(.5)
                    painter.setPen(pen)
                    pos = (self.translateX(self.localization_centroid[idx][0] * self.mapSpecs.meters_to_print_scale),
                           self.translateY(self.localization_centroid[idx][1] * self.mapSpecs.meters_to_print_scale))
                    a, b, phi = shared_math.ellipsify(
                        self.localization_error[idx], 3.0)
                    a = a * self.mapSpecs.meters_to_print_scale
                    b = b * self.mapSpecs.meters_to_print_scale
                    # Save the previous painter envinronment so we don't mess up the other things
                    painter.save()
                    # get the x and y components of the ellipse position
                    ellipse_x_offset = math.cos(
                        phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                    ellipse_y_offset = math.sin(
                        phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                    # translate the center to where our ellipse should be
                    painter.translate(
                        pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
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
                                     self.translateY(
                                         each[2] * self.mapSpecs.meters_to_print_scale),
                                     self.translateX(
                                         (each[1] + (each[4])) * self.mapSpecs.meters_to_print_scale),
                                     self.translateY((each[2] + (each[5])) * self.mapSpecs.meters_to_print_scale))

                    if self.display_covariance:
                        pen.setBrush(
                            brush_color['sensor_fusion_error_ellipse'])
                        pen.setWidth(.5)
                        painter.setPen(pen)
                        # Make sure covariance parameters have been added
                        if len(each) >= 3:
                            pos = (self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                   self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
                            a, b, phi = shared_math.ellipsify(each[3], 3.0)
                            a = a * self.mapSpecs.meters_to_print_scale
                            b = b * self.mapSpecs.meters_to_print_scale

                            if not math.isnan(a) and not math.isnan(b):
                                # Save the previous painter envinronment so we don't mess up the other things
                                painter.save()
                                # get the x and y components of the ellipse position
                                ellipse_x_offset = math.cos(
                                    phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                                ellipse_y_offset = math.sin(
                                    phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                                # translate the center to where our ellipse should be
                                painter.translate(
                                    pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                                # Rotate by phi to tunr the ellipse the correct way
                                painter.rotate(math.degrees(phi))
                                # Draw the ellipse at 0.0
                                painter.drawEllipse(0, 0, a, b)
                                # Restore the environment to what it was before
                                painter.restore()

    def paint_global_fusion(self, pen, painter):
        # Now draw the camera fusion detections
        pen.setBrush(brush_color['global_sensor_fusion_centroid'])
        pen.setWidth(6)
        painter.setPen(pen)
        for fuse in self.global_sensor_fusion_centroid:
            painter.drawPoint(self.translateX(fuse[1] * self.mapSpecs.meters_to_print_scale),
                              self.translateY(fuse[2] * self.mapSpecs.meters_to_print_scale))
        pen.setWidth(.5)
        painter.setPen(pen)
        for fuse in self.global_sensor_fusion_centroid:
            painter.drawLine(self.translateX(fuse[1] * self.mapSpecs.meters_to_print_scale),
                             self.translateY(
                                 fuse[2] * self.mapSpecs.meters_to_print_scale),
                             self.translateX(
                                 (fuse[1] + (fuse[4])) * self.mapSpecs.meters_to_print_scale),
                             self.translateY((fuse[2] + (fuse[5])) * self.mapSpecs.meters_to_print_scale))

        if self.display_covariance:
            pen.setBrush(brush_color['global_sensor_fusion_error_ellipse'])
            pen.setWidth(.5)
            painter.setPen(pen)
            for fuse in self.global_sensor_fusion_centroid:
                # Make sure covariance parameters have been added
                if len(fuse) >= 4:
                    pos = (self.translateX(fuse[1] * self.mapSpecs.meters_to_print_scale),
                           self.translateY(fuse[2] * self.mapSpecs.meters_to_print_scale))
                    a, b, phi = shared_math.ellipsify(fuse[3], 3.0)
                    a = a * self.mapSpecs.meters_to_print_scale
                    b = b * self.mapSpecs.meters_to_print_scale
                    if not math.isnan(a) and not math.isnan(b):
                        # Save the previous painter envinronment so we don't mess up the other things
                        painter.save()
                        # get the x and y components of the ellipse position
                        ellipse_x_offset = math.cos(
                            phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                        ellipse_y_offset = math.sin(
                            phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                        # translate the center to where our ellipse should be
                        painter.translate(
                            pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                        # Rotate by phi to tunr the ellipse the correct way
                        painter.rotate(math.degrees(phi))
                        # Draw the ellipse at 0.0
                        painter.drawEllipse(0, 0, a, b)
                        # Restore the environment to what it was before
                        painter.restore()

    def paint_sensors(self, pen, painter):
        for idx, cis in enumerate(self.sensors):
            global_position_x = cis[0]
            global_position_y = cis[1]
            theta = cis[2]
            velocity = cis[3]
            width = cis[4]
            length = cis[5]

            pen = QPen()
            pen.setWidth(4)
            pen.setBrush(brush_color['camera'])
            painter.setPen(pen)

            # Draw the camera position
            painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale),
                             self.translateY(
                                 global_position_y * self.mapSpecs.meters_to_print_scale),
                             self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale + (
                                 length * .5 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                 theta)),
                             self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale + (
                                 length * .5 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                 theta)))
            painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale + (
                (width/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                theta + math.radians(90))),
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale + (
                                (length/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                theta + math.radians(90))),
                self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale - (
                                (length/2) * self.mapSpecs.meters_to_print_scale) * math.cos(
                                theta + math.radians(90))),
                self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale - (
                                (length/2) * self.mapSpecs.meters_to_print_scale) * math.sin(
                                theta + math.radians(90))))

            if self.camera_debug:
                # Draw the FOV of cam
                pen.setWidth(.5)
                pen.setBrush(brush_color['camera_fov'])
                painter.setPen(pen)

                camera_center = self.sensor_camera_center[idx]
                camera_fov = self.sensor_camera_fov[idx]

                painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale),
                                 self.translateY(
                                     global_position_y * self.mapSpecs.meters_to_print_scale),
                                 self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + camera_center + (camera_fov/2))),
                                 self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + camera_center + (camera_fov/2))))

                painter.drawLine(self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale),
                                 self.translateY(
                                     global_position_y * self.mapSpecs.meters_to_print_scale),
                                 self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     theta + camera_center + -(camera_fov/2))),
                                 self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale - (
                                     1.0 * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     theta + camera_center + -(camera_fov/2))))

                # Now draw the vehicle camera detections
                pen.setBrush(brush_color['camera_detection_centroid'])
                pen.setWidth(4)
                painter.setPen(pen)
                for each in self.sensor_camera_detection_centroid[idx]:
                    painter.drawPoint(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                      self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))

                # Time for displaying the covaraince
                if self.display_covariance:
                    pen.setBrush(brush_color['camera_detection_error'])
                    pen.setWidth(.5)
                    painter.setPen(pen)
                    for each in self.sensor_camera_detection_centroid[idx]:
                        # Make sure covariance parameters have been added
                        if len(each) >= 3:
                            pos = (self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                   self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
                            a, b, phi = shared_math.ellipsify(each[3], 3.0)
                            a = a * self.mapSpecs.meters_to_print_scale
                            b = b * self.mapSpecs.meters_to_print_scale
                            # Save the previous painter envinronment so we don't mess up the other things
                            painter.save()
                            # get the x and y components of the ellipse position
                            ellipse_x_offset = math.cos(
                                phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                            ellipse_y_offset = math.sin(
                                phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                            # translate the center to where our ellipse should be
                            painter.translate(
                                pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
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
                if self.sensor_localization_error != []:
                    pos = (self.translateX(global_position_x * self.mapSpecs.meters_to_print_scale),
                           self.translateY(global_position_y * self.mapSpecs.meters_to_print_scale))
                    a, b, phi = shared_math.ellipsify(
                        self.sensor_localization_error[idx], 3.0)
                    a = a * self.mapSpecs.meters_to_print_scale
                    b = b * self.mapSpecs.meters_to_print_scale
                    # Save the previous painter envinronment so we don't mess up the other things
                    painter.save()
                    # get the x and y components of the ellipse position
                    ellipse_x_offset = math.cos(
                        phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                    ellipse_y_offset = math.sin(
                        phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                    # translate the center to where our ellipse should be
                    painter.translate(
                        pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
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
                for each in self.sensor_sensor_fusion_centroid[idx]:
                    painter.drawPoint(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                      self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
                pen.setWidth(.5)
                painter.setPen(pen)
                for each in self.sensor_sensor_fusion_centroid[idx]:
                    painter.drawLine(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                     self.translateY(
                                         each[2] * self.mapSpecs.meters_to_print_scale),
                                     self.translateX(
                                         (each[1] + (each[4])) * self.mapSpecs.meters_to_print_scale),
                                     self.translateY((each[2] + (each[5])) * self.mapSpecs.meters_to_print_scale))

                    if self.display_covariance:
                        pen.setBrush(
                            brush_color['sensor_fusion_error_ellipse'])
                        pen.setWidth(.5)
                        painter.setPen(pen)
                        # Make sure covariance parameters have been added
                        if len(each) >= 3:
                            pos = (self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                   self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
                            a, b, phi = shared_math.ellipsify(each[3], 3.0)
                            a = a * self.mapSpecs.meters_to_print_scale
                            b = b * self.mapSpecs.meters_to_print_scale

                            if not math.isnan(a) and not math.isnan(b):
                                # Save the previous painter envinronment so we don't mess up the other things
                                painter.save()
                                # get the x and y components of the ellipse position
                                ellipse_x_offset = math.cos(
                                    phi)*(a/2.0) + -math.sin(phi)*(b/2.0)
                                ellipse_y_offset = math.sin(
                                    phi)*(a/2.0) + math.cos(phi)*(b/2.0)
                                # translate the center to where our ellipse should be
                                painter.translate(
                                    pos[0]-ellipse_x_offset, pos[1]-ellipse_y_offset)
                                # Rotate by phi to tunr the ellipse the correct way
                                painter.rotate(math.degrees(phi))
                                # Draw the ellipse at 0.0
                                painter.drawEllipse(0, 0, a, b)
                                # Restore the environment to what it was before
                                painter.restore()

    # def paint_cooperative_monitoring(self, pen, painter):
    #     # Print out the legend for all the colors used
    #     self.results_list = []
    #     if self.results_list == []:
    #         self.results_list.append(QLabel(self))
    #         self.results_list[0].setText("Ground Truth:")
    #         self.results_list[0].move(500, 800)
    #         for idx, brush in enumerate(brush_color):
    #             self.labelList.append(QLabel(self))
    #             self.labelList[idx].setAutoFillBackground(True) # This is important!!
    #             color = QColor(brush_color[brush])
    #             alpha  = 140
    #             values = "{r}, {g}, {b}, {a}".format(r = color.red(),
    #                                                 g = color.green(),
    #                                                 b = color.blue(),
    #                                                 a = alpha
    #                                                 )
    #             self.labelList[idx].setStyleSheet("QLabel { background-color: rgba("+values+"); }")
    #             self.labelList[idx].setText(brush)
    #             self.labelList[idx].resize(200, 20)
    #             self.labelList[idx].move(5, 25 + (idx * 20))
