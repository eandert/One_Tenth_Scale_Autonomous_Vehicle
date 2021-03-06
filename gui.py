import sys
import time
import math
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from threading import Lock, Thread
import mapGenerator


def angleDifference( angle1, angle2 ):
    diff = ( angle2 - angle1 + math.pi ) % (2*math.pi) - math.pi;
    if diff < -math.pi:
        return diff + (2*math.pi)
    else:
        return diff

class MainWindow(QMainWindow):
    def __init__(self, mapSpecs, vehiclesLock, vehicles, trafficLightArray):
        print ( " GUI Started ")

        self.vehicles = vehicles
        self.trafficLightArray = trafficLightArray
        self.pause_simulation = True
        self.vehiclesLock = vehiclesLock

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

        self.testGroup = QButtonGroup(self)  # Radio button group

        self.radioTrafficLight = QRadioButton("Traffic Light", self)
        self.radioTrafficLight.resize(200, 32)
        self.radioTrafficLight.move(1000, 400)
        # self.radioTrafficLight.clicked.connect(self.showCustomOptions)
        self.radioTrafficLight.toggle()  # start in traffic test
        self.testGroup.addButton(self.radioTrafficLight)

        self.radioAutonomousIntersection = QRadioButton("Autonomous Intersection", self)
        self.radioAutonomousIntersection.resize(200, 32)
        self.radioAutonomousIntersection.move(1000, 460)
        # self.radioAutonomousIntersection.clicked.connect(self.showCustomOptions)
        self.testGroup.addButton(self.radioAutonomousIntersection)

        self.startButton = QPushButton('Start Test', self)
        self.startButton.resize(140, 32)
        self.startButton.move(1000, 600)

        self.startButton.clicked.connect(self.on_start_clicked)

        self.pauseButton = QPushButton('Pause Test', self)
        self.pauseButton.setEnabled(False)
        self.pauseButton.resize(140, 32)
        self.pauseButton.move(1000, 650)

        self.pauseButton.clicked.connect(self.on_pause_clicked)

        self.endButton = QPushButton('End Test', self)
        self.endButton.setEnabled(False)
        self.endButton.resize(140, 32)
        self.endButton.move(1000, 700)

        self.endButton.clicked.connect(self.on_end_clicked)

        self.drawIntersection = True

        # Set this to true after we have set the coordinates set
        # as this will enable the GUI to draw.
        self.drawCoordinates = True

        # Start the tcp server
        # Commenting out for movement over to Flask instead of reactor
        #reactor.listenTCP(12345, SimpleProtocolFactory(), interface='192.168.1.162')
        #reactor.run()

        # Running in a thread
        # Commenting out for movement over to Flask instead of reactor
        #Thread(target=reactor.run, args=(False,)).start()

        #time.sleep(1)

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

        self.setVelocityButton = QPushButton('Set Velocity', self)
        self.setVelocityButton.clicked.connect(self.on_set_velocity_clicked)
        self.setVelocityButton.resize(140, 32)
        self.setVelocityButton.move(1000, 300)

        self.phaseTarget = 1

        self.setVelocity = True

        self.drawVehicle = True

        # Initialize target points
        # for vehicle in vehicles:
        #    vehicle.pursuit_index = vehicle.search_target_index()

        # self.drawVehicle = True

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

    def on_end_clicked(self):
        sys.exit()

    def stepTime(self):
        self.time = round(time.time() * 1000)

        # 100HZ
        if (self.time - self.lastPositionUpdate) >= 10:
            self.lastPositionUpdate = self.time
            self.vehiclesLock.acquire()
            for key, vehicle in self.vehicles.items():
                # Update vehicle position based on physics
                vehicle.updatePosition(.01)
                self.drawTrafficLight = True
            self.vehiclesLock.release()

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
            for idx, vehicle in self.vehicles.items():
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
                        # print("veh" + str(idx) + " pause")
                        vehicle.update_localization()
                        vehicle.distance_pid_control_overide = True
                        vehicle.targetVelocity = 0.0
                        vehicle.update_pid()
                    else:
                        # print("veh" + str(idx) + " play")
                        # Update ourself
                        vehicle.update_localization()
                        vehicle.recieve_coordinate_group_commands(self.trafficLightArray)
                        vehicle.pure_pursuit_control()

                        # Get the last known location of all other vehicles
                        vehicleList = []
                        for otherIdx, otherVehicle in self.vehicles.items():
                            if idx != otherIdx:
                                vehicleList.append(otherVehicle.get_location())

                        # Now update our current PID with respect to other vehicles
                        vehicle.check_positions_of_other_vehicles_adjust_velocity(vehicleList, idx)
                        # We can't update the PID controls until after all positions are known
                        vehicle.update_pid()
            self.vehiclesLock.release()

        QApplication.processEvents()
        self.update()

    def on_set_velocity_clicked(self):
        self.setVelocity = True

    def translateX(self, x):
        return self.mapSpecs.centerX + x

    def translateY(self, y):
        return self.mapSpecs.centerY - y

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
            painter.setPen(pen)

            for x, y in zip(self.mapSpecs.xCoordinates, self.mapSpecs.yCoordinates):
                # painter.translate(self.mapSpecs.centerX-15, self.mapSpecs.centerY-30);
                # painter.rotate(90);
                painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * x),
                                  self.translateY(self.mapSpecs.meters_to_print_scale * y))

            # self.drawVehicle = False

        if self.drawVehicle:
            for idx, vehicle in self.vehicles.items():
                pen = QPen()
                pen.setWidth(4)
                pen.setBrush(Qt.green)
                painter.setPen(pen)
                # Draw the vehicle position
                painter.drawLine(self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale),
                                 self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                                             vehicle.wheelbaseLength * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta)),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             vehicle.wheelbaseLength * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta)))
                painter.drawLine(self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale + (
                            vehicle.wheelbaseWidth * self.mapSpecs.meters_to_print_scale) * math.cos(
                    vehicle.theta + math.radians(90))),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale + (
                                             vehicle.wheelbaseWidth * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90))),
                                 self.translateX(vehicle.localizationPositionX * self.mapSpecs.meters_to_print_scale - (
                                             vehicle.wheelbaseWidth * self.mapSpecs.meters_to_print_scale) * math.cos(
                                     vehicle.theta + math.radians(90))),
                                 self.translateY(vehicle.localizationPositionY * self.mapSpecs.meters_to_print_scale - (
                                             vehicle.wheelbaseWidth * self.mapSpecs.meters_to_print_scale) * math.sin(
                                     vehicle.theta + math.radians(90))))
                # Draw the target point
                pen.setBrush(Qt.red)
                painter.setPen(pen)
                painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * vehicle.targetIndexX),
                                  self.translateY(self.mapSpecs.meters_to_print_scale * vehicle.targetIndexY))
                # painter.drawPoint(self.translateX(self.mapSpecs.meters_to_print_scale * vehicle.centerPointX),
                #                  self.translateY(self.mapSpecs.meters_to_print_scale * vehicle.centerPointY))
                pen.setBrush(Qt.gray)
                pen.setWidth(1)
                painter.setPen(pen)
                self.drawTargetArc(vehicle.centerPointX, vehicle.centerPointY, vehicle.localizationPositionX,
                                   vehicle.localizationPositionY, vehicle.targetIndexX, vehicle.targetIndexY, painter)

                self.labelVehicleSpeedActual[idx].setText('VA=' + str(round(vehicle.velocity, 2)))
                self.labelVehicleSpeedTarget[idx].setText('VT=' + str(round(vehicle.targetVelocity, 2)))
                self.labelVehicleAcceleration[idx].setText('AA=' + str(round(vehicle.motorAcceleration, 2)))

                pen.setBrush(Qt.blue)
                pen.setWidth(4)
                painter.setPen(pen)
                buffer = .1
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

                # Now draw the vehicle detections
                pen.setBrush(Qt.cyan)
                pen.setWidth(4)
                painter.setPen(pen)
                for each in vehicle.lidarDetections:
                    #print ( each )
                    painter.drawPoint(self.translateX(each[1] * self.mapSpecs.meters_to_print_scale),
                                 self.translateY(each[2] * self.mapSpecs.meters_to_print_scale))
            # self.drawVehicle = False

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