import time
import math
import sys
from threading import Thread

from road_side_unit.src import rsu, communication
from gui.src import gui

# Import our config file
import config

global mainWin

def GuiProcess(config):
    # Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
    QTapp = gui.QtWidgets.QApplication(sys.argv)
    mainWin = gui.MainWindow(config)
    mainWin.show()
    QTapp.exec_()

def initGui(config):
    # Start up the Flask front end processor as it's own thread
    gui_handler = Thread(target=GuiProcess, args=(config, ))
    gui_handler.daemon = True
    gui_handler.start()

def run_single_test(conf):
    # Setup the RSU
    rsu_instance = rsu.RSU(conf)

    time.sleep(1)

    # Start the GUI
    initGui(conf)

    time.sleep(5)

    rsu_instance.stepSim()

    while(True):
        rsu_instance.check_state()

        if rsu_instance.end:
            sys.exit()

        time.sleep(.001)

def run_unit_tests(conf):
    # Setup the RSU
    rsu_instance = rsu.RSU(conf)

    time.sleep(1)

    # Start the GUI
    initGui(conf)

    time.sleep(5)

    rsu_instance.stepSim()

    while(True):
        rsu_instance.check_state()

        if rsu_instance.end:
            sys.exit()

        time.sleep(.001)

if __name__ == "__main__":
    # Configure the settings
    conf = config.Setting("four_cav_simulation")

    if conf.unit_test:
        run_unit_tests(conf)
    else:
        run_single_test(conf)

sys.exit()