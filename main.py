import time
import math
import sys
from threading import Thread

from road_side_unit.src import rsu, communication
from gui.src import gui

# Import our config file
import config

global mainWin

class UnitTest():
    def __init__(self):
        self.full_simulation = True
        self.simulate_error = True
        self.parameterized_covariance = False
        self.pause_simulation = False
        self.real_lidar = False
        self.unit_test_idx = 0

    def run(self, config):
        self.unit_test_idx = 0

        # Determing mode
        if len(self.unitTest) <= self.unit_test_idx:
            # Calculate the prior results
            self.calcResultsOfUnitTest()
            self.resetUnitTestStats()
            self.printUnitTestStats()
            
            # Test over
            sys.exit()
        else:
            # Setup the RSU
            rsu_instance = rsu.RSU(conf, self.unit_test_idx)

            time.sleep(1)

            # Start the GUI
            initGui(conf)

            time.sleep(5)

            rsu_instance.stepSim()

            while(True):
                test_end, stats = rsu_instance.check_state()

                if test_end:
                    rsu_instance.end_threads() = True
                    break

                if rsu_instance.end:
                    sys.exit()

                time.sleep(.001)

            # Add the test state
            self.addUnitTestStats(stats)

            # Reset the stats
            self.resetUnitTestStats()

            # Incrememt the unit test state
            self.unit_test_idx += 1

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
    return gui_handler

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

if __name__ == "__main__":
    # Configure the settings
    conf = config.Setting("four_cav_simulation")

    if conf.unit_test:
        UnitTest.UnitTest()
        UnitTest.run(conf)
    else:
        run_single_test(conf)

sys.exit()