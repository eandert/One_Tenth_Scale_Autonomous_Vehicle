import time
import math
import sys
import random
from threading import Thread

from road_side_unit.src import rsu, communication
from gui.src import gui
import multiprocessing as mp

# Import our config file
import config

global mainWin

parameterized_covariance_unit_test_set = ["two_cav_simulation_unit_test","two_cav_simulation_unit_test_2","two_cav_simulation_unit_test_3", "two_cav_simulation_unit_test_4","four_cav_simulation_unit_test","four_cav_simulation_unit_test_2","four_cav_simulation_unit_test_3","four_cav_simulation_unit_test_4"]
error_malicious_injection_unit_test_set = ["four_cav_simulation_error_injection_1"] * 10 + ["four_cav_simulation_error_injection_2"] * 10+ ["four_cav_simulation_error_injection_3"] * 10+ ["four_cav_simulation_error_injection_4"] * 10+ ["four_cav_simulation_error_injection_5"] * 10+ ["four_cav_simulation_error_injection_6"] * 10
error_malicious_injection_unit_test_set_retester = ["four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_e rror_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_6"]
error_malicious_injection_unit_test_set_quick = ["four_cav_simulation_error_injection_7", "four_cav_simulation_error_injection_8"]#"four_cav_simulation_error_injection_4","four_cav_simulation_error_injection_5","four_cav_simulation_error_injection_6","four_cav_simulation_error_injection_7","four_cav_simulation_error_injection_8"]
class UnitTest():
    def __init__(self):
        self.full_simulation = True
        self.simulate_error = True
        self.parameterized_covariance = False
        self.pause_simulation = False
        self.real_lidar = False
        self.unit_test_idx = 0
        self.unit_test_localization_rmse_results = []
        self.unit_test_localization_variance_results = []
        self.unit_test_localization_velocity_results = []
        # Onboard
        self.unit_test_local_rmse_results = []
        self.unit_test_local_variance_results = []
        self.unit_test_local_under_detection_miss_results = []
        self.unit_test_local_over_detection_miss_results = []
        # Global
        self.unit_test_global_rmse_results = []
        self.unit_test_global_variance_results = []
        self.unit_test_global_under_detection_miss_results = []
        self.unit_test_global_over_detection_miss_results = []
        self.error_monitor = []
        self.config = None

    def run(self, config_list):
        for conf_name in config_list:
            print(" Setting test config to: ", conf_name)
            conf = config.Setting(conf_name)
            self.unit_test_idx = 0
            self.config = conf

            # Determing mode
            while len(conf.unit_test_config) > self.unit_test_idx:
                injection_offset = random.randint(-conf.error_injection_time_range, conf.error_injection_time_range)
                conf.error_injection_time_tmp = conf.error_injection_time + injection_offset
                conf.unit_test_time_tmp = conf.unit_test_time + injection_offset

                # Setup the RSU
                rsu_instance = rsu.RSU(conf, self.unit_test_idx)

                time.sleep(5) 

                # Start the GUI
                #initGui(conf)
                #time.sleep(5)

                rsu_instance.stepSim()

                stats = [0,0,0,0,0,0,0,0,0,0,0]
                while(True):
                    test_end, stats, error_monitor = rsu_instance.check_state()

                    if test_end:
                        rsu_instance.end_threads()
                        print("Test ended")
                        break

                    if rsu_instance.end:
                        sys.exit()

                    time.sleep(.001)

                # Add the test state
                self.add_unit_test_stats(stats, error_monitor)
                self.print_unit_test_stats()

                # Incrememt the unit test state
                self.unit_test_idx += 1

                # End the file every time in case we don't hit the mark
                with open('twenty_percent_break_point.txt', 'a') as f:
                    f.write("\n")

            with open('output.txt', 'a') as f:
                f.write("\n")
            
            # Calculate the prior results
            self.print_unit_test_stats(end=True)

            self.clear_unit_test_stats()
        
        # Test over
        sys.exit()

    def add_unit_test_stats(self, stats, error_monitor):
        # Calculate the prior results
        # Localization
        self.unit_test_localization_rmse_results.append(stats[0])
        self.unit_test_localization_variance_results.append(stats[1])
        self.unit_test_localization_velocity_results.append(stats[2])

        # Onboard
        self.unit_test_local_rmse_results.append(stats[3])
        self.unit_test_local_variance_results.append(stats[4])
        self.unit_test_local_under_detection_miss_results.append(stats[5])
        self.unit_test_local_over_detection_miss_results.append(stats[6])

        # Global
        self.unit_test_global_rmse_results.append(stats[7])
        self.unit_test_global_variance_results.append(stats[8])
        self.unit_test_global_under_detection_miss_results.append(stats[9])
        self.unit_test_global_over_detection_miss_results.append(stats[10])

        self.error_monitor.append(error_monitor)

    def clear_unit_test_stats(self):
        self.unit_test_idx = 0
        self.unit_test_localization_rmse_results = []
        self.unit_test_localization_variance_results = []
        self.unit_test_localization_velocity_results = []
        # Onboard
        self.unit_test_local_rmse_results = []
        self.unit_test_local_variance_results = []
        self.unit_test_local_under_detection_miss_results = []
        self.unit_test_local_over_detection_miss_results = []
        # Global
        self.unit_test_global_rmse_results = []
        self.unit_test_global_variance_results = []
        self.unit_test_global_under_detection_miss_results = []
        self.unit_test_global_over_detection_miss_results = []
        self.error_monitor = []

    def print_unit_test_stats(self, end = False):
        idx = 0
        fails = 0
        for l_rmse, l_var, l_vel, o_rmse, o_var, o_o_miss, o_u_miss, g_rmse, g_var, g_o_miss, g_u_miss in zip(self.unit_test_localization_rmse_results,
            self.unit_test_localization_velocity_results, self.unit_test_localization_variance_results, 
            self.unit_test_local_rmse_results, self.unit_test_local_variance_results,
            self.unit_test_local_under_detection_miss_results, self.unit_test_local_over_detection_miss_results,
            self.unit_test_global_rmse_results, self.unit_test_global_variance_results,
            self.unit_test_global_under_detection_miss_results, self.unit_test_global_over_detection_miss_results):
            print( "Test: ", idx, " l_mode:", self.config.unit_test_config[idx][0], " g_mode:", self.config.unit_test_config[idx][1], " est_cov:", self.config.unit_test_config[idx][2] )
            print( "  localization_rmse_val: ", l_rmse, " variance: ", l_var, " velocity ", l_vel)
            print( "  onboard_rmse_val: ", o_rmse, " variance: ", o_var, " over misses: ", o_o_miss, " under misses: ", o_u_miss)
            print( "  global_rmse_val: ", g_rmse, " variance: ", g_var, " over misses: ", g_o_miss, " under misses: ", g_u_miss)
            print(self.error_monitor[idx])

            # if end:
            #     with open('output.txt', 'a') as f:
            #         f.write(str(idx) + "," +  str(self.config.unit_test_config[idx][1]) + "," + str(l_rmse) + "," + str(g_rmse) + "\n")

            idx += 1

def GuiProcess(config):
    # Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
    QTapp = gui.QtWidgets.QApplication(sys.argv)
    mainWin = gui.MainWindow(config)
    mainWin.show()
    QTapp.exec_()

def initGui(config):
    # Start up the Flask front end processor as it's own thread
    # gui_handler = Thread(target=GuiProcess, args=(config, ))
    # gui_handler.daemon = True
    # gui_handler.start()

    gui_handler = mp.Process(target=GuiProcess, args=(config, ))
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
    multiple_mode = True
    if multiple_mode:
        conf_list = error_malicious_injection_unit_test_set
        unit_test = UnitTest()
        unit_test.run(conf_list)
    else:
        conf = config.Setting("two_cav_simulation")
        run_single_test(conf)

#sys.exit() 