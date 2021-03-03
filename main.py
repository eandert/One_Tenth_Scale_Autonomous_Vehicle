import sys
import time
from threading import Lock, Thread
import vehicle
import gui
import mapGenerator
import communication


global mainWin


class RSU():
    def __init__(self, mapSpecs, isSimulation):
        global vehicles
        vehiclesLock.acquire()

        # Lets create some simulation vehicles, this would be done automatically as vehicles are added if this is not a simulation
        if isSimulation:
            newvehicle1 = vehicle.Vehicle()
            newvehicle1.initialVehicleAtPosition(
                (- (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) - 50) / mapSpecs.meters_to_print_scale, 0,
                0,
                mapSpecs.xCoordinates, mapSpecs.yCoordinates, mapSpecs.vCoordinates, len(vehicles), True)
            vehicles.append(newvehicle1)

            newvehicle2 = vehicle.Vehicle()
            newvehicle2.initialVehicleAtPosition(0, (
                    (mapSpecs.intersectionWidth * mapSpecs.meters_to_print_scale / 2) + 50) / mapSpecs.meters_to_print_scale,
                                                 270,
                                                 mapSpecs.xCoordinates, mapSpecs.yCoordinates, mapSpecs.vCoordinates, len(vehicles),
                                                 True)
            vehicles.append(newvehicle2)

        vehiclesLock.release()


def main(mapSpecs, vehiclesLock, vehicles, trafficLightArray):
    # Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
    QTapp = gui.QtWidgets.QApplication(sys.argv)
    mainWin = gui.MainWindow(mapSpecs, vehiclesLock, vehicles, trafficLightArray)
    mainWin.show()

    sys.exit(QTapp.exec_())

# Setup the thread lock
vehiclesLock = Lock()

# Setup the shared variables
vehicles = []
pose = {}
trafficLightArray = [0, 2, 0]

# Setup the mapspecs
mapSpecs = mapGenerator.MapSpecs()
sim = RSU(mapSpecs, True)

print ( "hi" )

# Start up the GUI as it's own thread
t = Thread(target=main, args=(mapSpecs, vehiclesLock, vehicles, trafficLightArray))
t.daemon = True
t.start()

print ( "end" )

# Startup the web service
app.config['foo'] = foo
communication.flask_app.run(host='localhost')

#while(True):
#    time.sleep(1)