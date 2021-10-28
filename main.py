import time
import math
import sys
from threading import Thread

from road_side_unit.src import rsu, communication

# Import our config file
import config

#global mainWin

# def main(mapSpecs, vehiclesLock, vehicles, sensors, trafficLightArray, unit_test):
#     # Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
#     QTapp = gui.QtWidgets.QApplication(sys.argv)
#     mainWin = gui.MainWindow(mapSpecs, vehiclesLock, vehicles, sensors, trafficLightArray, unit_test)
#     mainWin.show()

#     sys.exit(QTapp.exec_())
#     #QTapp.exec_()

def initFlask(q, rsu_instance, rsu_ip):
    # Start up the Flask front end processor as it's own thread
    if __name__ == "__main__":
        frontend = Thread(target=FlaskProccess, args=(q, rsu_instance, rsu_ip))
        frontend.daemon = True
        frontend.start()

def FlaskProccess(q, rsu_instance, rsu_ip):
    # Startup the web service
    communication.flask_app.config['RSUClass'] = rsu_instance
    communication.flask_app.config['RSUQueue'] = q
    communication.flask_app.run(host=rsu_ip, debug = True, use_reloader=False)

# Setup the RSU
print( "start ")
rsu_instance = rsu.RSU(config)

# This must be called after the class has been initialized fully
initFlask(rsu_instance.q, rsu_instance, config.rsu_ip)
time.sleep(1)

# If we are in a simulation, this will start the threads
rsu_instance.initSimulation(config)

while(True):
    continue
    #time.sleep(.1)
    #print ( "sleep" )

sys.exit()