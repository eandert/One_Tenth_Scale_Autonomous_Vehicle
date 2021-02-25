from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys
import time
from timeloop import Timeloop
from datetime import timedelta
import math
import numpy as np
from simple_pid import PID
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from threading import Thread
from twisted.internet.protocol import Factory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
import re

import mapGenerator

tl = Timeloop()

global mainWin

class SharedDataStore():
    def __init__(self):
        self.pose = {}
        self.commands = {}
        self.vehicles = []
        self.trafficLightArray = [0, 2, 0]

pose = {}
commands = {}
vehicles = []
trafficLightArray = [0, 2, 0]
stored_data = b''

pause_simulation = True


@tl.job(interval=timedelta(seconds=1))
def read_simulation_every_1s():
    global mainWin
    mainWin.checkStatus()

def angleDifference( angle1, angle2 ):
    diff = ( angle2 - angle1 + math.pi ) % (2*math.pi) - math.pi;
    if diff < -math.pi:
        return diff + (2*math.pi)
    else:
        return diff


class SimpleProtocol(LineReceiver):

    def parse_incoming(self, string):
        string_array = str(string).strip().strip('\x00').split(':')
        return string_array

    def calc_velocity(self, x1, y1, x2, y2, theta):
        velocity = math.hypot(x2 - x1, y2 - y1) * (1/8)
        expected_theta = math.atan2(y2 - y1, x2 - x1)
        if not (theta < (expected_theta + math.radians(45)) and theta > (expected_theta - math.radians(45))):
            # We are traveling backwards so adjust the velocity accordingly
            velocity = -velocity
        return velocity

    def connectionMade(self):
        print ( 'connectionMade' )

    def updateVehicleLocationOnly(self, id):
        # Update ourself
        vehicles[id].update_localization()

    def updateVehicle(self, id):
        # Update ourself
        vehicles[id].update_localization()
        vehicles[id].recieve_coordinate_group_commands(trafficLightArray)
        vehicles[id].pure_pursuit_control()

        # Get the last known location of all other vehicles
        vehicleList = []
        for idx, vehicle in enumerate(vehicles):
            vehicleList.append(vehicle.get_location())

        # Now update our current PID with respect to other vehicles
        vehicles[id].check_positions_of_other_vehicles_adjust_velocity(vehicleList, id)
        # We can't update the PID controls until after all positions are known
        vehicles[id].update_pid()

    def getVehicleRoute(self, id):
        # Get our route
        return vehicles[id].get_route()

    # NOTE: lineReceived(...) doesn't seem to get called

    def dataReceived(self, data):
        global stored_data
        #data = self.parse_incoming(data.decode('utf-8'))

        print ( data )
        print ( len(data) )
        print (data[0],data[1],data[2],data[7],data[12])

        if (len(data) >= 13) and chr(data[0]) == '[' and chr(data[1]) == 'R' and chr(data[2]) == ',' and chr(data[7]) == ',' and chr(data[12]) == ']':
            print ( "Case R")
            vehicle_id = int.from_bytes(data[3:6], byteorder='little')
            message_length = int.from_bytes(data[8:11], byteorder='little')
            return_message = bytes("R", 'utf-8') + (0).to_bytes(4, byteorder='big')# + self.getVehicleRoute(data[1])
            print ( " Sending back: " + str(return_message))
            self.sendLine(return_message)
            #for index in range(13):
            #    del stored_data[0]
        elif data[0] == 'P':
            if len(data) != 5:
                print("Error: invalid data received")
                return

            # Check if the vehicle ID exists yet
            # 0: ID, 1: x, 2: y, 3: theta
            if data[0] in pose:
                #print("case1")
                velocity = self.calc_velocity(float(pose[data[0]][1]), float(pose[data[0]][2]), float(data[1]),
                                              float(data[2]), float(data[3]))
                pose[data[0]] = [float(data[1]), float(data[2]), float(data[3]), velocity, int(round(time.time() * 1000))]
                # We should have updated controls from the simulator by now
            else:
                #print("case2")
                # Not enough data to calculate velocity, assume it is 0 for now
                velocity = 0.0
                pose[data[0]] = [float(data[1]), float(data[2]), float(data[3]), velocity, int(round(time.time() * 1000))]
                # init the control array to 0
                # Since this is the first time we have data, start the controls at 0 until we get the velocity
                commands[data[0]] = [0, 0]

            if pause_simulation:
                self.updateVehicleLocationOnly(int(data[0]))
                commands[data[0]] = [0, 0]
            else:
                self.updateVehicle(int(data[0]))

            message = str(commands[data[0]][0]) + ":" + str(commands[data[0]][1])
            self.sendLine(message.encode('utf-8'))

        #print(pose[data[0]])
        #print(commands[data[0]])

class SimpleProtocolFactory(Factory):

    def buildProtocol(self, addr):
        return SimpleProtocol()


class Vehicle:
    def __init__(self):
        # Static Vehicle Params
        self.width = .3
        self.length = .57
        self.wheelbaseLengthFromRear = .1
        self.wheelbaseLength = .4
        self.wheelbaseWidth = .1
        self.steeringAngleMax = 30.0
        self.velocityMax = 1.0
        
        self.maxTurningRadius = self.wheelbaseLength / math.tan(self.steeringAngleMax)
        
        self.k = 0.3  # look forward gain
        self.Lfc = 0.5  # look-ahead distance
        
        # Updatable Params
        self.velocity = 0
        self.theta = 0
        self.seeringAngle = 0
        self.positionX_sim = 0
        self.positionY_sim = 0
        self.localizationPositionX = 0
        self.localizationPositionY = 0
        self.lastPointIndex = 0
        self.targetIndexX = 0
        self.targetIndexY = 0
        self.lookAheadIndex = 0
        self.targetVelocity = 0
        self.steeringAcceleration = 0
        self.motorAcceleration = 0
        self.pursuit_index = 0
        self.centerPointX = 0
        self.centerPointY = 0
        self.lastTargetWithinTL = 0
        self.distance_pid_control_en = False
        self.distance_pid_control_overide = False
        self.followDistance = self.length + self.Lfc
        self.targetFollowDistance = 1

        self.id = None
        self.simVehicle = True
        
    def initialVehicleAtPosition(self, x_offset, y_offset, theta_offset, xCoordinates, yCoordinates, vCoordinates, id, simVehicle):
        self.targetVelocityGeneral = 0
        self.id = id
        self.simVehicle = simVehicle
        self.seeringAngle = 0
        # This holds the actual position of the vehicle
        self.positionX_offset = x_offset
        self.positionY_offset = y_offset
        self.theta_offset = math.radians(theta_offset)
        # This is the known localization position
        if simVehicle:
            self.localizationPositionX = self.positionX_offset
            self.localizationPositionY = self.positionY_offset
            self.positionX_sim = self.localizationPositionX
            self.positionY_sim = self.localizationPositionY
            self.velocity = 0
            self.theta = self.theta_offset
        else:
            # Since this is a real world test we will start the vehicle somewhere random until it connects
            self.localizationPositionX = 5 + self.positionX_offset
            self.localizationPositionY = 5 + self.positionY_offset
            self.theta = self.theta_offset
            self.velocity = 0
        self.lastPointIndex = None
        self.xCoordinates = xCoordinates
        self.yCoordinates = yCoordinates
        self.vCoordinates = vCoordinates
        
        # Initialize the controllers\
        if self.simVehicle:
            self.v_pid = PID(3, 0.00, 0.0, setpoint=self.targetVelocity)
            self.d_pid = PID(2, 0.00, 0.0, setpoint=self.Lfc)
        else:
            self.v_pid = PID(1.5, 0.00, 0.0, setpoint=self.targetVelocity)
            self.d_pid = PID(2, 0.00, 0.0, setpoint=self.Lfc)

    def updatePosition(self, timestep):
        self.positionX_sim += self.velocity * math.cos(self.theta) * timestep
        self.positionY_sim += self.velocity * math.sin(self.theta) * timestep
        self.theta += ( self.velocity / self.wheelbaseLength ) * math.tan(self.steeringAcceleration) * timestep
        #print ( self.motorAcceleration )
        self.velocity += self.motorAcceleration * timestep

    def update_localization(self):
        if self.simVehicle:
            # Update the localization, we could inject error here if we want
            self.localizationPositionX = self.positionX_sim
            self.localizationPositionY = self.positionY_sim
        else:
            # Update the localization from real data
            self.localizationPositionX = pose[self.id][0] + self.positionX_offset
            self.localizationPositionY = pose[self.id][1] + self.positionY_offset
            self.theta = pose[self.id][2] + self.theta_offset
            self.velocity = pose[self.id][3]

    def pure_pursuit_control(self):

        ind = self.search_target_index()

        tx = self.xCoordinates[ind]
        ty = self.yCoordinates[ind]

        alpha = math.atan2(ty - self.localizationPositionY, tx - self.localizationPositionX) - self.theta

        Lf = self.k * self.velocity + self.Lfc

        delta = math.atan2(2.0 * self.wheelbaseLength * math.sin(alpha) / Lf, 1.0)

        if delta > math.radians(self.steeringAngleMax):
            delta = math.radians(self.steeringAngleMax)
        elif delta < -math.radians(self.steeringAngleMax):
            delta = -math.radians(self.steeringAngleMax)

        # Account for the fact that in reverse we should be turning the other way
        #if self.velocity < 0:
        #    delta = -delta
        self.steeringAcceleration = delta

        self.targetIndexX = tx
        self.targetIndexY = ty
        self.distance_pid_control_overide = False
        if self.vCoordinates[ind] == 0:
            self.targetVelocity = self.targetVelocityGeneral
            self.lastTargetWithinTL = 0
        else:
            if self.coordinateGroupVelocities[self.vCoordinates[ind]] == 1:
                if self.lastTargetWithinTL == 1:
                    if self.targetVelocity == 0:
                        # We are already stopping so keep stopping
                        self.targetVelocity = 0
                        self.distance_pid_control_overide = True
                    else:
                        # We have already entered the light so keep going
                        self.targetVelocity = self.targetVelocityGeneral
                else:
                    # This is the first point we have seen of this TFL, should have enought time to stop
                    self.targetVelocity = 0
                    self.distance_pid_control_overide = True
                self.lastTargetWithinTL = 1
            elif self.coordinateGroupVelocities[self.vCoordinates[ind]] == 2:
                self.targetVelocity = self.targetVelocityGeneral
                self.lastTargetWithinTL = 1
            else:
                self.targetVelocity = 0
                self.lastTargetWithinTL = 1
                self.distance_pid_control_overide = True

        if alpha != 0:
            turningRadius = self.wheelbaseLength / math.tan(alpha)
            if turningRadius > 10:
                turningRadius = 10
            if turningRadius < -10:
                turningRadius = -10
        else:
            turningRadius = 10
        self.centerPointX = self.localizationPositionX + turningRadius * math.cos(self.theta + math.radians(90))
        self.centerPointY = self.localizationPositionY + turningRadius * math.sin(self.theta + math.radians(90))

    def update_pid(self):
        if self.distance_pid_control_en and not self.distance_pid_control_overide:
            #print(str(self.id) + "TD", self.targetFollowDistance, "FD", self.followDistance)
            self.d_pid.setpoint = self.targetFollowDistance
            self.motorAcceleration = self.d_pid(self.followDistance)
        else:
            # Default to velocity PID cotnrol
            #print ( self.targetVelocity )
            self.v_pid.setpoint = self.targetVelocity
            self.motorAcceleration = self.v_pid(self.velocity)
            #print("veh" + str(self.id) + " TV:", str(self.targetVelocity) + " MA:" + str(self.motorAcceleration))
        if self.simVehicle == False:
            commands[self.id] = [-self.steeringAcceleration, self.motorAcceleration]

    def calc_distance(self, point_x, point_y):
        dx = self.localizationPositionX - point_x
        dy = self.localizationPositionY - point_y
        return math.hypot(dx, dy)

    def recieve_coordinate_group_commands(self, commands):
        self.coordinateGroupVelocities = commands

    def get_location(self):
        return [self.localizationPositionX, self.localizationPositionY, self.theta, self.targetVelocity, .2]

    def get_route(self):
        messageString = ""
        for xzip, yzip in zip(self.xCoordinates, self.yCoordinates):
            if len(messageString) == 0:
                messageString = str(xzip) + ',' + str(yzip)
            else:
                messageString = messageString + ':' + str(xzip) + ',' + str(yzip)
        return messageString

    def check_if_point_in_rectangle(self, x1, y1, x2, y2, x, y):
        if x1 > x2:
            xtemp = x2
            x2 = x1
            x1 = xtemp
        if y1 > y2:
            ytemp = y2
            y2 = y1
            y1 = ytemp
        if (x > x1) and (x < x2):
            if (y > y1) and (y < y2):
                return True
        return False

    def check_positions_of_other_vehicles_adjust_velocity(self, positions, myIndex):
        self.followDistance = 99
        self.distance_pid_control_en = False
        for idx, each in enumerate(positions):
            # Create a bounding box for each vehicle that is length + 2*buffer long and width + 2*buffer wide
            x1 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]+math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.cos(each[2]-math.radians(180))))
            y1 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]+math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.sin(each[2]-math.radians(180))))
            x2 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]-math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.cos(each[2]-math.radians(180))))
            y2 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]-math.radians(90)) + ((self.wheelbaseLengthFromRear + each[4])*math.sin(each[2]-math.radians(180))))
            x3 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]-math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.cos(each[2])))
            y3 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]-math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.sin(each[2])))
            x4 = each[0] + ((self.width/2 + each[4])*math.cos(each[2]+math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.cos(each[2])))
            y4 = each[1] + ((self.width/2 + each[4])*math.sin(each[2]+math.radians(90)) + ((self.length - self.wheelbaseLengthFromRear + each[4])*math.sin(each[2])))
            point = Point(self.targetIndexX, self.targetIndexY)
            polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
            #print(polygon.contains(point))
            #if self.check_if_point_in_rectangle(x1, y1, x2, y2, self.targetIndexX, self.targetIndexY):
            if polygon.contains(point):
                #print ( "True" )
                # Within the safety buffer, adjust speed to be that of the front vehicle
                #if self.targetVelocity > each[3]:
                #    self.targetVelocity = each[3]
                # For distance control
                targetFollowDistance = self.length - self.wheelbaseLengthFromRear + self.Lfc
                followDistance = math.hypot(each[0]+((2*self.length - 2*self.wheelbaseLengthFromRear + each[4])*math.cos(each[2]))-self.localizationPositionX,each[1]+((2*self.length - 2*self.wheelbaseLengthFromRear + each[4])*math.sin(each[2]))-self.localizationPositionY)
                if self.followDistance > followDistance:
                    self.followDistance = followDistance
                    self.targetFollowDistance = targetFollowDistance
                    self.distance_pid_control_en = True

    def check_steering_angle_possible(self, x, y):
        # This equation is a quick check for if it is possible to get to the current point based on geometry
        # Essentually 2 circles that we cant go to
        dx = self.localizationPositionX + self.maxTurningRadius*math.cos(angleDifference(self.theta + math.radians(90),x))
        dy = self.localizationPositionY + self.maxTurningRadius*math.cos(angleDifference(self.theta + math.radians(90),y))
        d = math.hypot(dx, dy)

        dx2 = self.localizationPositionX + self.maxTurningRadius*math.cos(angleDifference(self.theta - math.radians(90), x))
        dy2 = self.localizationPositionY + self.maxTurningRadius*math.cos(angleDifference(self.theta - math.radians(90), y))
        d2 = math.hypot(dx2, dy2)

        a = angleDifference(math.atan2(self.localizationPositionY - y, self.localizationPositionX - x), self.theta)
        
        # Check if the target point is within either no go circle
        if d < self.maxTurningRadius or d2 < self.maxTurningRadius and a > math.radians(-90) and a < math.radians(90):
            return False
        else:
            return True

    def search_target_index(self):
        if self.lastPointIndex is None:
            # Search for the initial point, not reverse
            dx = [self.localizationPositionX - icx for icx in self.xCoordinates]
            dy = [self.localizationPositionY - icy for icy in self.yCoordinates]
            d = np.hypot(dx, dy)
            for index in range(len(dx)):
                if not self.check_steering_angle_possible(dx[index],dy[index]):
                    d[index] = 1000.0
            ind = np.argmin(d)
            self.lastPointIndex = ind
        else:
            ind = self.lastPointIndex
            distance_this_index = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])
            while True:
                checkind = ind + 1
                if checkind >= len(self.xCoordinates):
                    checkind = 0
                distance_next_index = self.calc_distance(self.xCoordinates[checkind], self.yCoordinates[checkind])
                if distance_this_index < distance_next_index:
                    break
                distance_this_index = distance_next_index
                ind = checkind
            self.lastPointIndex = ind

        L = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])

        Lf = self.k * self.velocity + self.Lfc

        # search look ahead target point index
        while Lf > L:
            L = self.calc_distance(self.xCoordinates[ind], self.yCoordinates[ind])
            ind += 1
            if ind >= len(self.xCoordinates):
                ind = 0

        return ind
        
    #def pidControl(self, timestep):


class MainWindow(QMainWindow):
    def __init__(self):
        # Parameters of test
        # Intersection Params
        self.intersectionStraightLength = 1
        self.intersectionWidth = .5
        self.centerX = 500
        self.centerY = 500
        self.meters_to_print_scale = 100.0
        
        # Parameters for the trajectory point generation
        self.distanceInterval = .1
        
        # Simulation/Real Params
        self.isSimulation = 1
        self.lightTime = 0
        self.lightTimePeriod = 8 * 5 # 5 seconds * 8 hz
        
        # Time params
        self.time = 0
        self.lastPositionUpdate = 0
        self.lastLocalizationUpdate = 0

        # Draw params
        self.drawTrafficLight = False
        
        QMainWindow.__init__(self)

        self.setMinimumSize(QSize(1280, 1000))    
        self.setWindowTitle("Demo GUI")

        self.labelName = QLabel(self)
        self.labelName.setText('Settings:')
        self.labelName.move(1000, 20)

        self.testGroup = QButtonGroup(self) # Radio button group
        
        self.radioTrafficLight = QRadioButton("Traffic Light", self)
        self.radioTrafficLight.resize(200, 32)
        self.radioTrafficLight.move(1000, 400)
        #self.radioTrafficLight.clicked.connect(self.showCustomOptions)
        self.radioTrafficLight.toggle() # start in traffic test
        self.testGroup.addButton(self.radioTrafficLight)
        
        self.radioAutonomousIntersection = QRadioButton("Autonomous Intersection", self)
        self.radioAutonomousIntersection.resize(200, 32)
        self.radioAutonomousIntersection.move(1000, 460)
        #self.radioAutonomousIntersection.clicked.connect(self.showCustomOptions)
        self.testGroup.addButton(self.radioAutonomousIntersection)

        self.startButton = QPushButton('Start Test', self)
        self.startButton.resize(140,32)
        self.startButton.move(1000, 600)

        self.startButton.clicked.connect(self.on_start_clicked)
        
        self.pauseButton = QPushButton('Pause Test', self)
        self.pauseButton.setEnabled(False)
        self.pauseButton.resize(140,32)
        self.pauseButton.move(1000, 650)

        self.pauseButton.clicked.connect(self.on_pause_clicked)

        self.endButton = QPushButton('End Test', self)
        self.endButton.setEnabled(False)
        self.endButton.resize(140,32)
        self.endButton.move(1000, 700)

        self.endButton.clicked.connect(self.on_end_clicked)
        
        self.drawIntersection = True

        self.xCoordinates, self.yCoordinates, self.vCoordinates = mapGenerator.generateFigureEight(self.intersectionStraightLength, self.intersectionWidth, self.distanceInterval)

        # Set this to true after we have set the coordinates set
        # as this will enable the GUI to draw.
        self.drawCoordinates = True

        # Start the tcp server
        # Commenting out for movement over to Flask instead of reactor
        reactor.listenTCP(12345, SimpleProtocolFactory(), interface='192.168.1.162')
        reactor.run()

        # Running in a thread
        # Commenting out for movement over to Flask instead of reactor
        Thread(target=reactor.run, args=(False,)).start()
        
        time.sleep(1)

        # Lets create some vehicles, this would be done automatically as vehicles are added if this is not a simulation
        newvehicle1 = Vehicle()
        newvehicle1.initialVehicleAtPosition(
            (- (self.intersectionWidth * self.meters_to_print_scale / 2) - 50) / self.meters_to_print_scale, 0, 0,
            self.xCoordinates, self.yCoordinates, self.vCoordinates, len(vehicles), False)
        vehicles.append(newvehicle1)

        newvehicle2 = Vehicle()
        newvehicle2.initialVehicleAtPosition(0, (
                    (self.intersectionWidth * self.meters_to_print_scale / 2) + 50) / self.meters_to_print_scale, 270,
                                             self.xCoordinates, self.yCoordinates, self.vCoordinates, len(vehicles), True)
        vehicles.append(newvehicle2)

        self.labelVehicleSpeed = []
        self.lineVehicleSpeed = []
        self.labelVehicleSpeedActual = []
        self.labelVehicleSpeedTarget = []
        self.labelVehicleAcceleration = []

        for idx, vehicle in enumerate(vehicles):

            self.labelVehicleSpeed.append(QLabel(self))
            self.labelVehicleSpeed[idx].setText('Speed Vehicle ' + str(idx) + ':')
            self.labelVehicleSpeed[idx].move(1000, 80 + 80*idx)

            self.lineVehicleSpeed.append(QLineEdit(self))
            self.lineVehicleSpeed[idx].move(1000, 120 + 80*idx)
            self.lineVehicleSpeed[idx].resize(100, 32)
            self.lineVehicleSpeed[idx].setText("0")

            self.labelVehicleSpeedActual.append(QLabel(self))
            self.labelVehicleSpeedActual[idx].setText('VA=0')
            self.labelVehicleSpeedActual[idx].move(1100, 120 + 80*idx)

            self.labelVehicleSpeedTarget.append(QLabel(self))
            self.labelVehicleSpeedTarget[idx].setText('VT=0')
            self.labelVehicleSpeedTarget[idx].move(1150, 120 + 80*idx)

            self.labelVehicleAcceleration.append(QLabel(self))
            self.labelVehicleAcceleration[idx].setText('AA=0')
            self.labelVehicleAcceleration[idx].move(1200, 120 + 80 * idx)

        self.setVelocityButton = QPushButton('Set Velocity', self)
        self.setVelocityButton.clicked.connect(self.on_set_velocity_clicked)
        self.setVelocityButton.resize(140, 32)
        self.setVelocityButton.move(1000, 300)

        self.phaseTarget = 1

        self.setVelocity = True
        
        self.drawVehicle = True

        # Initialize target points
        #for vehicle in vehicles:
        #    vehicle.pursuit_index = vehicle.search_target_index()
        
        # self.drawVehicle = True
        if self.isSimulation:
            # Do this forevvvvveerrrrr!!!
            timer = QtCore.QTimer(self, timeout=self.stepTime, interval=25)
            timer.start()

    def on_start_clicked(self):
        global pause_simulation
        self.endButton.setEnabled(True)
        self.pauseButton.setEnabled(True)
        self.startButton.setEnabled(False)
        pause_simulation = False

    def on_pause_clicked(self):
        global pause_simulation
        self.pauseButton.setEnabled(False)
        self.startButton.setEnabled(True)
        pause_simulation = True

    def on_end_clicked(self):
        sys.exit()

    def stepTime(self):
        global trafficLightArray
        self.time = round(time.time() * 1000)
        
        # 100HZ
        if (self.time - self.lastPositionUpdate) >= 10:
            self.lastPositionUpdate = self.time
            for vehicle in vehicles:
                # Update vehicle position based on physics
                vehicle.updatePosition(.01)
                self.drawTrafficLight = True
    
        # 8HZ
        if (self.time - self.lastLocalizationUpdate) >= 125:
            #print ( self.time )
            self.lastLocalizationUpdate = self.time
            # Traffic light update sequence
            if self.lightTime > self.lightTimePeriod:
                self.lightTime = 0
                if trafficLightArray[1] == 2:
                    trafficLightArray[1] = 1
                    trafficLightArray[2] = 0
                    lightTimePeriod = 0 * 8
                elif trafficLightArray[2] == 2:
                    trafficLightArray[1] = 0
                    trafficLightArray[2] = 1
                    lightTimePeriod = 0 * 8
                elif trafficLightArray[1] == 1:
                    trafficLightArray[1] = 0
                    trafficLightArray[2] = 2
                    lightTimePeriod = 5 * 8
                elif trafficLightArray[2] == 1:
                    trafficLightArray[1] = 2
                    trafficLightArray[2] = 0
                    lightTimePeriod = 5 * 8
            else:
                self.lightTime += 1
            for idx, vehicle in enumerate(vehicles):
                vehicle.targetVelocityGeneral = float(self.lineVehicleSpeed[idx].text())
                if vehicle.simVehicle:
                    if pause_simulation:
                        #print("veh" + str(idx) + " pause")
                        vehicle.update_localization()
                        vehicle.distance_pid_control_overide = True
                        vehicle.targetVelocity = 0.0
                        vehicle.update_pid()
                    else:
                        #print("veh" + str(idx) + " play")
                        # Update ourself
                        vehicle.update_localization()
                        vehicle.recieve_coordinate_group_commands(trafficLightArray)
                        vehicle.pure_pursuit_control()

                        # Get the last known location of all other vehicles
                        vehicleList = []
                        for otherIdx, otherVehicle in enumerate(vehicles):
                            if idx != otherIdx:
                                vehicleList.append(otherVehicle.get_location())

                        # Now update our current PID with respect to other vehicles
                        vehicle.check_positions_of_other_vehicles_adjust_velocity(vehicleList, idx)
                        # We can't update the PID controls until after all positions are known
                        vehicle.update_pid()

        QApplication.processEvents()
        self.update()

    def on_set_velocity_clicked(self):
        self.setVelocity = True

    def translateX(self, x):
        return self.centerX + x
        
    def translateY(self, y):
        return self.centerY - y

    def drawTargetArc(self, x0, y0, x1, y1, x2, y2, painter):
        r = math.hypot(x1 - x0, y1 - y0)
        x = x0 - r
        y = y0 + r
        width = 2 * r
        height = 2 * r
        startAngle = math.atan2(y1 - y0, x1 - x0)
        endAngle = math.atan2(y2 - y0, x2 - x0)
        angleLen = math.degrees(angleDifference(startAngle,endAngle))
        startAngle = math.degrees(startAngle)
        painter.drawArc(self.translateX(x*self.meters_to_print_scale), self.translateY(y*self.meters_to_print_scale), width*self.meters_to_print_scale, height*self.meters_to_print_scale, startAngle * 16, angleLen * 16)
        
    def paintEvent(self, event):
    
        painter = QPainter(self)
        if self.drawIntersection:
            painter.setPen(Qt.black)

            # Draw top section
            painter.drawLine(self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionStraightLength*self.meters_to_print_scale) - (self.intersectionWidth*self.meters_to_print_scale/2))
            painter.drawLine(self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionStraightLength*self.meters_to_print_scale) - (self.intersectionWidth*self.meters_to_print_scale/2))
            
            # Draw bottom section
            painter.drawLine(self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionStraightLength*self.meters_to_print_scale) + (self.intersectionWidth*self.meters_to_print_scale/2))
            painter.drawLine(self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionStraightLength*self.meters_to_print_scale) + (self.intersectionWidth*self.meters_to_print_scale/2))
            
            # Draw left section
            painter.drawLine(self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2) - (self.intersectionStraightLength*self.meters_to_print_scale), self.centerY - (self.intersectionWidth*self.meters_to_print_scale/2))
            painter.drawLine(self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2) - (self.intersectionStraightLength*self.meters_to_print_scale), self.centerY + (self.intersectionWidth*self.meters_to_print_scale/2))
            
            # Draw right section
            painter.drawLine(self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2) + (self.intersectionStraightLength*self.meters_to_print_scale), self.centerY - (self.intersectionWidth*self.meters_to_print_scale/2))
            painter.drawLine(self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2) + (self.intersectionStraightLength*self.meters_to_print_scale), self.centerY + (self.intersectionWidth*self.meters_to_print_scale/2))
            
            # Draw top right 3/4 circle
            painter.drawArc(self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionWidth*self.meters_to_print_scale/2) - (self.intersectionStraightLength*self.meters_to_print_scale*2), self.intersectionStraightLength*self.meters_to_print_scale*2, self.intersectionStraightLength*self.meters_to_print_scale*2, -90 * 16, 270 * 16)
            painter.drawArc(self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY - (self.intersectionWidth*self.meters_to_print_scale*1.5) - (self.intersectionStraightLength*self.meters_to_print_scale*2), self.intersectionStraightLength*self.meters_to_print_scale*2 + (self.intersectionWidth*self.meters_to_print_scale*2), self.intersectionStraightLength*self.meters_to_print_scale*2 + (self.intersectionWidth*self.meters_to_print_scale*2), -90 * 16, 270 * 16)
            
            # Draw top right 3/4 circle
            painter.drawArc(self.centerX - (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionWidth*self.meters_to_print_scale/2) + (self.intersectionStraightLength*self.meters_to_print_scale*2), -self.intersectionStraightLength*self.meters_to_print_scale*2, -self.intersectionStraightLength*self.meters_to_print_scale*2, 90 * 16, 270 * 16)
            painter.drawArc(self.centerX + (self.intersectionWidth*self.meters_to_print_scale/2), self.centerY + (self.intersectionWidth*self.meters_to_print_scale*1.5) + (self.intersectionStraightLength*self.meters_to_print_scale*2), -self.intersectionStraightLength*self.meters_to_print_scale*2 - (self.intersectionWidth*self.meters_to_print_scale*2), -self.intersectionStraightLength*self.meters_to_print_scale*2 - (self.intersectionWidth*self.meters_to_print_scale*2), 90 * 16, 270 * 16)
            
            #self.drawIntersection = False

        if self.drawCoordinates:
            pen = QPen()
            pen.setWidth(2)
            painter.setPen(pen)
            
            for x,y in zip(self.xCoordinates,self.yCoordinates):
                #painter.translate(self.centerX-15, self.centerY-30);
                #painter.rotate(90);
                painter.drawPoint(self.translateX(self.meters_to_print_scale*x),self.translateY(self.meters_to_print_scale*y))

            #self.drawVehicle = False

        if self.drawVehicle:
            for idx, vehicle in enumerate(vehicles):
                pen = QPen()
                pen.setWidth(4)
                pen.setBrush(Qt.green)
                painter.setPen(pen)
                # Draw the vehicle position
                painter.drawLine(self.translateX(vehicle.localizationPositionX*self.meters_to_print_scale),
                                self.translateY(vehicle.localizationPositionY*self.meters_to_print_scale),
                                self.translateX(vehicle.localizationPositionX*self.meters_to_print_scale + (vehicle.wheelbaseLength*self.meters_to_print_scale)*math.cos(vehicle.theta)),
                                self.translateY(vehicle.localizationPositionY*self.meters_to_print_scale + (vehicle.wheelbaseLength*self.meters_to_print_scale)*math.sin(vehicle.theta)))
                painter.drawLine(self.translateX(vehicle.localizationPositionX*self.meters_to_print_scale + (vehicle.wheelbaseWidth*self.meters_to_print_scale)*math.cos(vehicle.theta+math.radians(90))),
                                self.translateY(vehicle.localizationPositionY*self.meters_to_print_scale + (vehicle.wheelbaseWidth*self.meters_to_print_scale)*math.sin(vehicle.theta+math.radians(90))),
                                self.translateX(vehicle.localizationPositionX*self.meters_to_print_scale - (vehicle.wheelbaseWidth*self.meters_to_print_scale)*math.cos(vehicle.theta+math.radians(90))),
                                self.translateY(vehicle.localizationPositionY*self.meters_to_print_scale - (vehicle.wheelbaseWidth*self.meters_to_print_scale)*math.sin(vehicle.theta+math.radians(90))))
                # Draw the target point
                pen.setBrush(Qt.red)
                painter.setPen(pen)
                painter.drawPoint(self.translateX(self.meters_to_print_scale*vehicle.targetIndexX),self.translateY(self.meters_to_print_scale*vehicle.targetIndexY))
                #painter.drawPoint(self.translateX(self.meters_to_print_scale * vehicle.centerPointX),
                #                  self.translateY(self.meters_to_print_scale * vehicle.centerPointY))
                pen.setBrush(Qt.gray)
                pen.setWidth(1)
                painter.setPen(pen)
                self.drawTargetArc(vehicle.centerPointX,vehicle.centerPointY,vehicle.localizationPositionX,vehicle.localizationPositionY,vehicle.targetIndexX,vehicle.targetIndexY,painter)

                self.labelVehicleSpeedActual[idx].setText('VA=' + str(round(vehicle.velocity,2)))
                self.labelVehicleSpeedTarget[idx].setText('VT=' + str(round(vehicle.targetVelocity,2)))
                self.labelVehicleAcceleration[idx].setText('AA=' + str(round(vehicle.motorAcceleration, 2)))

                pen.setBrush(Qt.blue)
                pen.setWidth(4)
                painter.setPen(pen)
                buffer = .1
                x1 = vehicle.localizationPositionX + ((vehicle.width/2 + buffer)*math.cos(vehicle.theta+math.radians(90)) + ((vehicle.wheelbaseLengthFromRear + buffer)*math.cos(vehicle.theta-math.radians(180))))
                y1 = vehicle.localizationPositionY + ((vehicle.width/2 + buffer)*math.sin(vehicle.theta+math.radians(90)) + ((vehicle.wheelbaseLengthFromRear + buffer)*math.sin(vehicle.theta-math.radians(180))))
                x2 = vehicle.localizationPositionX + ((vehicle.width/2 + buffer)*math.cos(vehicle.theta-math.radians(90)) + ((vehicle.wheelbaseLengthFromRear + buffer)*math.cos(vehicle.theta-math.radians(180))))
                y2 = vehicle.localizationPositionY + ((vehicle.width/2 + buffer)*math.sin(vehicle.theta-math.radians(90)) + ((vehicle.wheelbaseLengthFromRear + buffer)*math.sin(vehicle.theta-math.radians(180))))
                x3 = vehicle.localizationPositionX + ((vehicle.width/2 + buffer)*math.cos(vehicle.theta-math.radians(90)) + ((vehicle.length - vehicle.wheelbaseLengthFromRear + buffer)*math.cos(vehicle.theta)))
                y3 = vehicle.localizationPositionY + ((vehicle.width/2 + buffer)*math.sin(vehicle.theta-math.radians(90)) + ((vehicle.length - vehicle.wheelbaseLengthFromRear + buffer)*math.sin(vehicle.theta)))
                x4 = vehicle.localizationPositionX + ((vehicle.width/2 + buffer)*math.cos(vehicle.theta+math.radians(90)) + ((vehicle.length - vehicle.wheelbaseLengthFromRear + buffer) * math.cos(vehicle.theta)))
                y4 = vehicle.localizationPositionY + ((vehicle.width/2 + buffer)*math.sin(vehicle.theta+math.radians(90)) + ((vehicle.length - vehicle.wheelbaseLengthFromRear + buffer) * math.sin(vehicle.theta)))

                #print (vehicle.localizationPositionX, vehicle.localizationPositionY, x1, y1, x2, y2)
                painter.drawPoint(self.translateX(x1 * self.meters_to_print_scale),
                                  self.translateY(y1 * self.meters_to_print_scale))
                painter.drawPoint(self.translateX(x2 * self.meters_to_print_scale),
                                  self.translateY(y2 * self.meters_to_print_scale))
                painter.drawPoint(self.translateX(x3 * self.meters_to_print_scale),
                                  self.translateY(y3 * self.meters_to_print_scale))
                painter.drawPoint(self.translateX(x4 * self.meters_to_print_scale),
                                  self.translateY(y4 * self.meters_to_print_scale))
            #self.drawVehicle = False

        if self.drawTrafficLight:
            pen = QPen()

            # N/S direction
            pen.setWidth(4)
            if trafficLightArray[2] == 2:
                pen.setBrush(Qt.green)
            elif trafficLightArray[2] == 1:
                pen.setBrush(Qt.yellow)
            else:
                pen.setBrush(Qt.red)
            painter.setPen(pen)
            painter.drawLine(self.centerX - (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY + (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerX + (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY + (self.intersectionWidth * self.meters_to_print_scale / 2))
            painter.drawLine(self.centerX - (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY - (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerX + (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY - (self.intersectionWidth * self.meters_to_print_scale / 2))

            # E/W direction
            pen.setWidth(4)
            if trafficLightArray[1] == 2:
                pen.setBrush(Qt.green)
            elif trafficLightArray[1] == 1:
                pen.setBrush(Qt.yellow)
            else:
                pen.setBrush(Qt.red)
            painter.setPen(pen)
            painter.drawLine(self.centerX + (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY + (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerX + (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY - (self.intersectionWidth * self.meters_to_print_scale / 2))
            painter.drawLine(self.centerX - (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY + (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerX - (self.intersectionWidth * self.meters_to_print_scale / 2),
                             self.centerY - (self.intersectionWidth * self.meters_to_print_scale / 2))


# global mainWin
#
# # Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
# QTapp = QtWidgets.QApplication(sys.argv)
# mainWin = MainWindow()
# mainWin.show()
#
# sys.exit(QTapp.exec_())