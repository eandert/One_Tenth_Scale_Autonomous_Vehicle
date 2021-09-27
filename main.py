import time
import math
import sys
from threading import Lock, Thread
from queue import Queue
from road_side_unit.src import mapGenerator, communication, gui
from connected_autonomous_vehicle.src import planning_control as vehicle_planning
from connected_infrastructure_sensor.src import planning_control as cam_planning

global mainWin


class RSU():
    def __init__(self, mapSpecs, vehiclesLock, vehicles, sensors, trafficLightArray, isSimulation):
        self.mapSpecs = mapSpecs
        self.vehicles = vehicles
        self.sensors = sensors
        self.vehiclesLock = vehiclesLock
        self.trafficLightArray = trafficLightArray

        self.vehiclesLock.acquire()

        # Lets create some simulation vehicles, this would be done automatically as vehicles are added if this is not a simulation
        if isSimulation:
            newvehicle1 = vehicle_planning.Planner()
            newvehicle1.initialVehicleAtPosition(
                (- (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale, 0,
                0,
                mapSpecs.xCoordinates, mapSpecs.yCoordinates, mapSpecs.vCoordinates, 0, True)

            newvehicle2 = vehicle_planning.Planner()
            newvehicle2.initialVehicleAtPosition(
                2.0 * (- (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale, 0,
                0,
                mapSpecs.xCoordinates, mapSpecs.yCoordinates, mapSpecs.vCoordinates, 1, True)

            # newvehicle3 = vehicle_planning.Planner()
            newvehicle2.initialVehicleAtPosition(0, (
                    (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale,
                                                 270,
                                                 mapSpecs.xCoordinates, mapSpecs.yCoordinates, mapSpecs.vCoordinates, len(self.vehicles),
                                                 True)

            newSensor = cam_planning.Planner()
            newSensor.initialVehicleAtPosition(
                (- (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale, (
                    (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale,
                -45, 2, True)

            # newSensor2 = cam_planning.Planner()
            # newSensor2.initialVehicleAtPosition(
            #     (+ (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale, -(
            #         (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale,
            #     -45 + 180, 2, True)

            self.vehicles[0] = newvehicle1
            self.vehicles[1] = newvehicle2
            #self.vehicles[2] = newvehicle3
            self.sensors[0] = newSensor
            #self.sensors[1] = newSensor2

            # print("Pos veh 0: ", (- (
            #             mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale,
            #       0, 0)
            # print("Pos veh 1: ", 2*(- (
            #             mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale,
            #       0, 0)
            # print("Pos sens 0: ", (- (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale, (
            #         (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale,
            #     -45)

        self.vehiclesLock.release()

    def register(self, key, id, type, timestamp, x, y, z, roll, pitch, yaw):
        if type == 0:
            # Check if this vehicle ID is taken or not
            if id in self.vehicles:
                print ( " Warning: Vehicle ID already in use!")

            # TODO: replace this with somethign better, this is the init funciton that arbitrarily locates the vehicles at positions
            # and if the vehicle is not at the correct location this will not work
            if id == 0:
                init_x = ( - (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale
                init_y = 0.0
                init_yaw = 0
            elif id == 1:
                init_x = 2.0 * (- (
                            mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale
                init_y = 0.0
                init_yaw = 0

            # TODO: Improve this to add the cars dynamically

            # Set the key so we have some security
            self.vehicles[id].key = key

            # Now init the vehicle at a location
            self.vehicles[id].update_localization(x, y, yaw, 0.0)
            self.vehicles[id].recieve_coordinate_group_commands(self.trafficLightArray)

            # We update this just for the visualizer
            self.vehicles[id].pure_pursuit_control()

            # Get the last known location of all other vehicles
            vehicleList = []
            for idx, vehicle in self.vehicles.items():
                vehicleList.append(vehicle.get_location())

            # Now update our current PID with respect to other vehicles
            self.vehicles[id].check_positions_of_other_vehicles_adjust_velocity(vehicleList, id)

            # We can't update the PID controls until after all positions are known
            # We still do this here just for debugging as it should match the PID controls
            # on the actual car and then it will be displayed on the UI
            self.vehicles[id].update_pid()

            # Finally we can create the return messages
            registerResponse = dict(
                v_t=self.vehicles[id].targetVelocityGeneral,
                t_x=init_x,
                t_y=init_y,
                t_z="0.0",
                t_roll="0.0",
                t_pitch="0.0",
                t_yaw=math.radians(init_yaw),
                route_x=self.mapSpecs.xCoordinates,
                route_y=self.mapSpecs.yCoordinates,
                route_TFL=self.mapSpecs.vCoordinates,
                tfl_state=self.trafficLightArray,
                veh_locations=vehicleList,
                timestep=time.time()
            )

            return registerResponse

        elif type == 1:
            # Check if this vehicle ID is taken or not
            if id in self.sensors:
                print(" Warning: Sensor ID already in use!")

            # TODO: replace this with somethign better, this is the init funciton that arbitrarily locates the vehicles at positions
            # and if the vehicle is not at the correct location this will not work
            if id == 0:
                init_x = (-(mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale
                init_y = ((mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale
                init_yaw = -45

            # TODO: Improve this to add the cars dynamically

            # Set the key so we have some security
            self.sensors[id].key = key

            # Now init the vehicle at a location
            self.sensors[id].update_localization(x, y, yaw, 0.0)

            # Finally we can create the return messages
            registerResponse = dict(
                t_x=init_x,
                t_y=init_y,
                t_z="0.0",
                t_roll="0.0",
                t_pitch="0.0",
                t_yaw=math.radians(init_yaw),
                route_x=self.mapSpecs.xCoordinates,
                route_y=self.mapSpecs.yCoordinates,
                route_TFL=self.mapSpecs.vCoordinates,
                tfl_state=self.trafficLightArray,
                timestep=time.time()
            )

            return registerResponse


    def checkinFastResponse(self, key, id, type, timestamp, x, y, z, roll, pitch, yaw):
        if type == 0:
            # Double check our security, this is pretty naive at this point
            if self.vehicles[id].key == key:
                # TODO: possibly do these calculation after responding to increase response time

                # Calculate our velocity using our last position
                velocity = self.calc_velocity(self.vehicles[id].localizationPositionX, self.vehicles[id].localizationPositionY, x, y, yaw)

                # Update ourself
                self.vehicles[id].update_localization(x, y, yaw, velocity)
                #self.vehicles[vehicle_id].recieve_coordinate_group_commands(trafficLightArray)

                # Get the last known location of all other vehicles
                vehicleList = []
                for idx, vehicle in self.vehicles.items():
                    vehicleList.append(vehicle.get_location())

                # Finally we can create the return messages
                registerResponse = dict(
                    v_t=self.vehicles[id].targetVelocity,
                    tfl_state=self.trafficLightArray,
                    veh_locations=vehicleList,
                    timestep=time.time()
                )

                # Finally we can create the return message
                return registerResponse
        elif type == 1:
            # Double check our security, this is pretty naive at this point
            if self.sensors[id].key == key:
                # Finally we can create the return messages
                registerResponse = dict(
                    tfl_state=self.trafficLightArray,
                    timestep=time.time()
                )

                # Finally we can create the return message
                return registerResponse

    def calc_velocity(self, x1, y1, x2, y2, theta):
        velocity = math.hypot(x2 - x1, y2 - y1) * (1/8)
        expected_theta = math.atan2(y2 - y1, x2 - x1)
        if not (theta < (expected_theta + math.radians(45)) and theta > (expected_theta - math.radians(45))):
            # We are traveling backwards so adjust the velocity accordingly
            velocity = -velocity
        return velocity


def BackendProcessor(q, vehicles, sensors, trafficLightArray):
    while True:
        message = q.get()
        #print ( message )
        key, id, type, timestamp, x, y, yaw, detections = message

        # See if we are dealing with a sensor or a vehicle
        if type == 1:
            # Double check our security, this is pretty naive at this point
            if sensors[id].key == key:
                # Lets add the detections to the sensor class
                sensors[id].cameraDetections = detections["cam_obj"]
                with open("output.csv", "a") as file:
                    file.write("cam," + str(id) + "," + str(x)
                               + "," + str(y)
                               + "," + str(yaw)
                               + "," + str(detections)
                               + "\n")
        elif type == 0:
            # Double check our security, this is pretty naive at this point
            if vehicles[id].key == key:
                # We do these calculation after responding to increase response time
                vehicles[id].recieve_coordinate_group_commands(trafficLightArray)

                # We update this just for the visualizer
                vehicles[id].pure_pursuit_control()

                #print ( "detections: ", detections["lidar_obj"] )

                # Lets add the detections to the vehicle class
                vehicles[id].cameraDetections = detections["cam_obj"]
                vehicles[id].lidarDetections = detections["lidar_obj"]

                # Get the last known location of all other vehicles
                vehicleList = []
                for idx, vehicle in vehicles.items():
                    vehicleList.append(vehicle.get_location())

                # Now update our current PID with respect to other vehicles
                vehicles[id].check_positions_of_other_vehicles_adjust_velocity(vehicleList, id)

                # We can't update the PID controls until after all positions are known
                # We still do this here just for debugging as it should match the PID controls
                # on the actual car and then it will be displayed on the UI
                vehicles[id].update_pid()

                with open("output.csv", "a") as file:
                    file.write("cav," + str(id) + "," + str(x)
                               + "," + str(y)
                               + "," + str(yaw)
                               + "," + str(detections)
                               +"\n")


def main(mapSpecs, vehiclesLock, vehicles, sensors, trafficLightArray, unit_test):
    # Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
    QTapp = gui.QtWidgets.QApplication(sys.argv)
    mainWin = gui.MainWindow(mapSpecs, vehiclesLock, vehicles, sensors, trafficLightArray, unit_test)
    mainWin.show()

    sys.exit(QTapp.exec_())
    #QTapp.exec_()

#for each in range(9):
# Unit test stuff
simulation = True
unit_test = [True, 1, 1]

# Setup the thread lock
vehiclesLock = Lock()

# Setup the shared variables
vehicles = {}
sensors = {}
pose = {}
trafficLightArray = [0, 2, 0]

# Setup the mapspecs
mapSpecs = mapGenerator.MapSpecs(0)
sim = RSU(mapSpecs, vehiclesLock, vehicles, sensors, trafficLightArray, simulation)

# Start up the GUI as it's own thread
t = Thread(target=main, args=(mapSpecs, vehiclesLock, vehicles, sensors, trafficLightArray, unit_test))
t.daemon = True
t.start()

# Queue to talk with threads
q = Queue()
# Start up the Flask back end processor as it's own thread
t2 = Thread(target=BackendProcessor, args=(q, vehicles, sensors, trafficLightArray))
t2.daemon = True
t2.start()

# Startup the web service
communication.flask_app.config['RSUClass'] = sim
communication.flask_app.config['RSUQueue'] = q
communication.flask_app.run(host="localhost") #host='192.168.0.103')