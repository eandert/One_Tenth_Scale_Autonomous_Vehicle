class UnitTest():
    def __init__(self):
        self.full_simulation = True
        self.simulate_error = True
        self.parameterized_covariance = False
        self.pause_simulation = False
        self.real_lidar = False
        self.unit_test_idx = 0

    def run(self, config):
        # Determing mode
        if self.unit_test_state == 0:
            self.unit_test_idx = 0

            # Set the fusion modes
            self.local_fusion_mode = self.unitTest[self.unit_test_idx][0]
            self.global_fusion_mode = self.unitTest[self.unit_test_idx][1]
            self.parameterized_covariance = self.unitTest[self.unit_test_idx][2]
            self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
            for idx, veh in self.vehicles.items():
                if veh.simVehicle:
                    self.localFusionCAV[idx].fusion_mode = self.local_fusion_mode
            for idx, sens in self.cis.items():
                if sens.simCIS:
                    self.localFusionCIS[idx].fusion_mode = self.local_fusion_mode

            # Reset the stats
            self.resetUnitTestStats()

            # Increment the unit test counter for those long tests
            self.unit_test_state = 1
            self.unit_test_idx += 1
        elif len(self.unitTest) <= self.unit_test_idx:
            # Calculate the prior results
            self.calcResultsOfUnitTest()
            self.resetUnitTestStats()
            self.printUnitTestStats()
            
            # Test over
            sys.exit()
        else:
            # Calculate the prior results
            self.calcResultsOfUnitTest()
            self.resetUnitTestStats()

            # Set the fusion modes
            self.local_fusion_mode = self.unitTest[self.unit_test_idx][0]
            self.global_fusion_mode = self.unitTest[self.unit_test_idx][1]
            self.parameterized_covariance = self.unitTest[self.unit_test_idx][2]
            self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
            for idx, veh in self.vehicles.items():
                if veh.simVehicle:
                    self.localFusionCAV[idx].fusion_mode = self.local_fusion_mode
            for idx, sens in self.cis.items():
                if sens.simCIS:
                    self.localFusionCIS[idx].fusion_mode = self.local_fusion_mode

            # Incrememt the unit test state
            self.unit_test_idx += 1

    def calcResultsOfUnitTest(self):
        # Calculate the prior results
        # Localization
        differences_squared_l = np.array(self.localization_differences) ** 2
        mean_of_differences_squared_l = differences_squared_l.mean()
        rmse_val_l = np.sqrt(mean_of_differences_squared_l)
        variance_l = np.var(self.localization_differences,ddof=1)

        self.unit_test_localization_rmse_results.append(rmse_val_l)
        self.unit_test_localization_variance_results.append(variance_l)

        # Onboard
        differences_squared = np.array(self.local_differences) ** 2
        mean_of_differences_squared = differences_squared.mean()
        rmse_val = np.sqrt(mean_of_differences_squared)
        variance = np.var(self.local_differences,ddof=1)

        self.unit_test_local_rmse_results.append(rmse_val)
        self.unit_test_local_variance_results.append(variance)
        self.unit_test_local_under_detection_miss_results.append(self.local_under_detection_miss)
        self.unit_test_local_over_detection_miss_results.append(self.local_over_detection_miss)

        # Global
        differences_squared_g = np.array(self.global_differences) ** 2
        mean_of_differences_squared_g = differences_squared_g.mean()
        rmse_val_g = np.sqrt(mean_of_differences_squared_g)
        variance_g = np.var(self.global_differences,ddof=1)

        self.unit_test_global_rmse_results.append(rmse_val_g)
        self.unit_test_global_variance_results.append(variance_g)
        self.unit_test_global_under_detection_miss_results.append(self.global_under_detection_miss)
        self.unit_test_global_over_detection_miss_results.append(self.global_over_detection_miss)

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

    def resetUnitTestStats(self):
        # Reset the stats
        self.localization_differences = []
        self.local_over_detection_miss = 0
        self.local_under_detection_miss = 0
        self.local_differences = []
        self.global_over_detection_miss = 0
        self.global_under_detection_miss = 0
        self.global_differences = []