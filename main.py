import sys
import time
import math
from threading import Lock, Thread
import vehicle
import gui
import mapGenerator
import communication


global mainWin


class RSU():
    def __init__(self, mapSpecs, vehiclesLock, vehicles, trafficLightArray, isSimulation):
        self.mapSpecs = mapSpecs
        self.vehicles = vehicles
        self.vehiclesLock = vehiclesLock
        self.trafficLightArray = trafficLightArray

        self.vehiclesLock.acquire()

        # Lets create some simulation vehicles, this would be done automatically as vehicles are added if this is not a simulation
        if isSimulation:
            newvehicle1 = vehicle.Vehicle()
            newvehicle1.initialVehicleAtPosition(
                (- (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale, 0,
                0,
                mapSpecs.xCoordinates, mapSpecs.yCoordinates, mapSpecs.vCoordinates, len(self.vehicles), True)
            self.vehicles[0] = newvehicle1

            newvehicle2 = vehicle.Vehicle()
            newvehicle2.initialVehicleAtPosition(0, (
                    (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale,
                                                 270,
                                                 mapSpecs.xCoordinates, mapSpecs.yCoordinates, mapSpecs.vCoordinates, len(self.vehicles),
                                                 False)
            self.vehicles[1] = newvehicle2

        self.vehiclesLock.release()

    def register(self, key, vehicle_id, timestamp, x, y, z, roll, pitch, yaw):
        # Check if this vehicle ID is taken or not
        if vehicle_id in self.vehicles:
            print ( " Error: Vehicle ID already in use!")

        # TODO: replace this with somethign better, this is the init funciton that arbitrarily locates the vehicles at positions
        # and if the vehicle is not at the correct location this will not work
        if vehicle_id == 0:
            init_x = ( - (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale
            init_y = 0.0
            init_yaw = 0
        elif vehicle_id == 1:
            init_x = 0.0
            init_y = ((mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale
            init_yaw = 270

        # TODO: Improve this to add the cars dynamically
        # # Lets make a new vehicle
        # newvehicle = vehicle.Vehicle()
        # newvehicle.initialVehicleAtPosition(
        #     init_x,
        #     init_y,
        #     0,
        #     mapSpecs.xCoordinates,
        #     mapSpecs.yCoordinates,
        #     mapSpecs.vCoordinates,
        #     vehicle_id,
        #     False)

        # Set the key so we have some security
        newvehicle.key = key

        # Now init the vehicle at a location
        self.vehicles[vehicle_id].update_localization(x, y, yaw, 0.0)
        self.vehicles[vehicle_id].recieve_coordinate_group_commands(trafficLightArray)

        # We update this just for the visualizer
        self.vehicles[vehicle_id].pure_pursuit_control()

        # Get the last known location of all other vehicles
        vehicleList = []
        for idx, vehicle in enumerate(vehicles):
            vehicleList.append(vehicle.get_location())

        # Now update our current PID with respect to other vehicles
        self.vehicles[vehicle_id].check_positions_of_other_vehicles_adjust_velocity(vehicleList, vehicle_id)

        # We can't update the PID controls until after all positions are known
        # We still do this here just for debugging as it should match the PID controls
        # on the actual car and then it will be displayed on the UI
        self.vehicles[vehicle_id].update_pid()

        # Finally we can create the return messages
        return [self.vehicles[vehicle_id].targetVelocity, init_x, init_y, 0.0, init_yaw, 0.0, 0.0, self.mapSpecs.xCoordinates, self.mapSpecs.yCoordinates, self.mapSpecs.vCoordinates, trafficLightArray, vehicleList, time.time()]


    def checkin(self, key, vehicle_id, timestamp, x, y, z, roll, pitch, yaw):
        # Double check our security, this is pretty naive at this point
        if self.vehicles[vehicle_id].key == key:
            # Calculate our velocity using our last position
            velocity = self.calc_velocity(self.vehicles[vehicle_id].localizationPositionX, self.vehicles[vehicle_id].localizationPositionY, x, y, yaw)

            # Update ourself
            self.vehicles[vehicle_id].update_localization(x, y, yaw, velocity)
            self.vehicles[vehicle_id].recieve_coordinate_group_commands(trafficLightArray)

            # We update this just for the visualizer
            self.vehicles[vehicle_id].pure_pursuit_control()

            # Get the last known location of all other vehicles
            vehicleList = []
            for idx, vehicle in enumerate(vehicles):
                vehicleList.append(vehicle.get_location())

            # Now update our current PID with respect to other vehicles
            self.vehicles[vehicle_id].check_positions_of_other_vehicles_adjust_velocity(vehicleList, vehicle_id)

            # We can't update the PID controls until after all positions are known
            # We still do this here just for debugging as it should match the PID controls
            # on the actual car and then it will be displayed on the UI
            self.vehicles[vehicle_id].update_pid()

            # Finally we can create the return message
            return [self.vehicles[vehicle_id].targetVelocity, trafficLightArray, vehicleList, time.time()]

    def calc_velocity(self, x1, y1, x2, y2, theta):
        velocity = math.hypot(x2 - x1, y2 - y1) * (1/8)
        expected_theta = math.atan2(y2 - y1, x2 - x1)
        if not (theta < (expected_theta + math.radians(45)) and theta > (expected_theta - math.radians(45))):
            # We are traveling backwards so adjust the velocity accordingly
            velocity = -velocity
        return velocity


def main(mapSpecs, vehiclesLock, vehicles, trafficLightArray):
    # Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
    QTapp = gui.QtWidgets.QApplication(sys.argv)
    mainWin = gui.MainWindow(mapSpecs, vehiclesLock, vehicles, trafficLightArray)
    mainWin.show()

    sys.exit(QTapp.exec_())

# Setup the thread lock
vehiclesLock = Lock()

# Setup the shared variables
vehicles = {}
pose = {}
trafficLightArray = [0, 2, 0]

# Setup the mapspecs
mapSpecs = mapGenerator.MapSpecs()
sim = RSU(mapSpecs, vehiclesLock, vehicles, trafficLightArray, True)

print ( "hi" )

# Start up the GUI as it's own thread
t = Thread(target=main, args=(mapSpecs, vehiclesLock, vehicles, trafficLightArray))
t.daemon = True
t.start()

print ( "end" )

# Startup the web service
communication.flask_app.config['RSUClass'] = sim
communication.flask_app.run(host='192.168.1.162')

#while(True):
#    time.sleep(1)