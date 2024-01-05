import pickle
from shared_library import global_fusion, shared_math, general_functions
import numpy as np

test_reruns = 1
num_tests = 3
error_malicious_injection_unit_test_set = ["four_cav_simulation_error_injection_1"]# + ["four_cav_simulation_error_injection_2"] + ["four_cav_simulation_error_injection_3"] * 0 + ["four_cav_simulation_error_injection_4"] * 0 + ["four_cav_simulation_error_injection_5"] * 0 + ["four_cav_simulation_error_injection_6"] * test_length + ["four_cav_simulation_error_injection_7"] * test_length + ["four_cav_simulation_error_injection_8"] * test_length  + ["four_cav_simulation_error_injection_9"] * test_length

for test in error_malicious_injection_unit_test_set:
    for test_rerun in range(test_reruns):
        for num_test in range(num_tests):
            with open("input_fast_global_mode/" + test + "_" + str(test_rerun) + "_" + str(num_test) + ".pickle", 'rb') as handle:
                pickle_dict = pickle.load(handle)

                global_fusion_mode = 0
                error_target_vehicle = 1

                # # Normal
                # global_differences = []
                # global_over_detection_miss = 0
                # global_under_detection_miss = 0
                # globalFusionList = []
                # globalFusion = None
                # globalFusion = global_fusion.GlobalFUSION(global_fusion_mode)
                # error_injection_time = sorted(pickle_dict.keys())[-1] - 60.0
                # twenty_percent_error_hit = False
                # error_at_100 = 1.0
                # for current_time in sorted(pickle_dict.keys()):
                #     ground_truth = pickle_dict[current_time][0]
                #     cav_cis_fusions = pickle_dict[current_time][1]

                #     for cis_cav in cav_cis_fusions:
                #         globalFusion.processDetectionFrame(current_time, cis_cav, .25, True)

                #     globalFusionList, _, _ = globalFusion.fuseDetectionFrame(True, True)
                
                #     # Uses true positions of the CAVs to ground truth the sensing.
                #     # This mode is only available when in simulation mode and unit testing.
                #     # Ground truth the local fusion result
                #     # Ground truth the global fusion result
                #     if current_time >= error_injection_time:
                #         over_detection_miss_g, under_detection_miss_g, differences_g = general_functions.ground_truth_dataset(globalFusionList, ground_truth)
                #         global_differences += differences_g
                #         global_over_detection_miss += over_detection_miss_g
                #         global_under_detection_miss += under_detection_miss_g

                #         # Calculate the results
                #         rmse_val_g = shared_math.RMSE(global_differences)
                #         variance_g = np.var(global_differences, ddof=1)
                #         rmse = rmse_val_g
                #         variance = variance_g

                #     print(" Global fusion completed step: ", current_time)

                # with open('test_output/fast/rmse.txt', 'a') as f:
                #     f.write(str(rmse) + " ,")
                # with open('test_output/fast/variance.txt', 'a') as f:
                #     f.write(str(variance) + " ,")

                # print(" Global fusion run complete -----")

                # Conclave
                global_differences = []
                global_over_detection_miss = 0
                global_under_detection_miss = 0
                globalFusionList = []
                conclave_dict = {}
                error_at_100 = 1.0
                globalFusion = None
                globalFusion = global_fusion.GlobalFUSION(global_fusion_mode)
                error_injection_time = sorted(pickle_dict.keys())[-1] - 60.0
                conclave_vehicle_weight = 1.0
                twenty_percent_error_hit = False
                error_at_100 = 1.0
                for current_time in sorted(pickle_dict.keys()):
                    ground_truth = pickle_dict[current_time][0]
                    cav_cis_fusions = pickle_dict[current_time][1]
                    cav_pos = pickle_dict[current_time][2]
                    cis_pos = pickle_dict[current_time][3]

                    for idx, cis_cav in enumerate(cav_cis_fusions):
                        if current_time >= error_injection_time and idx == error_target_vehicle:
                            globalFusion.processDetectionFrame(current_time, cis_cav, .25, True, conclave_vehicle_weight)
                        else:
                            globalFusion.processDetectionFrame(current_time, cis_cav, .25, True)

                    globalFusionList, conclave_data, _ = globalFusion.fuseDetectionFrame(True, True)

                    if current_time % 1 == 0:
                        conclave_vehicle_weight, error_at_100, twenty_percent_error_hit = general_functions.cooperative_monitoring_process_conclave(conclave_data, globalFusionList, cav_pos, cis_pos, current_time, conclave_dict, error_injection_time, error_at_100, twenty_percent_error_hit)
                
                    # Uses true positions of the CAVs to ground truth the sensing.
                    # This mode is only available when in simulation mode and unit testing.
                    # Ground truth the local fusion result
                    # Ground truth the global fusion result
                    if current_time >= error_injection_time:
                        over_detection_miss_g, under_detection_miss_g, differences_g = general_functions.ground_truth_dataset(globalFusionList, ground_truth)
                        global_differences += differences_g
                        global_over_detection_miss += over_detection_miss_g
                        global_under_detection_miss += under_detection_miss_g

                        # Calculate the results
                        rmse_val_g = shared_math.RMSE(global_differences)
                        variance_g = np.var(global_differences, ddof=1)
                        rmse = rmse_val_g
                        variance = variance_g

                    print(" Conclave fusion completed step: ", current_time)

                with open('test_output/fast/rmse_conclave.txt', 'a') as f:
                    f.write(str(rmse) + " ,")
                with open('test_output/fast/variance_conclave.txt', 'a') as f:
                    f.write(str(variance) + " ,")

                print(" Conclave fusion run complete -----")

                # Trupercept
                global_differences = []
                global_over_detection_miss = 0
                global_under_detection_miss = 0
                globalFusionList = []
                trupercept_dict = {}
                globalFusion = None
                globalFusion = global_fusion.GlobalFUSION(global_fusion_mode)
                error_injection_time = sorted(pickle_dict.keys())[-1] - 60.0
                trupercept_vehicle_weight = 1.0
                twenty_percent_error_hit = False
                error_at_100 = 1.0
                for current_time in sorted(pickle_dict.keys()):
                    ground_truth = pickle_dict[current_time][0]
                    cav_cis_fusions = pickle_dict[current_time][1]
                    cav_pos = pickle_dict[current_time][2]
                    cis_pos = pickle_dict[current_time][3]

                    for idx, cis_cav in enumerate(cav_cis_fusions):
                        if current_time >= error_injection_time and idx == error_target_vehicle:
                            globalFusion.processDetectionFrame(current_time, cis_cav, .25, True, trupercept_vehicle_weight)
                        else:
                            globalFusion.processDetectionFrame(current_time, cis_cav, .25, True)

                    globalFusionList, _, trupercept_data = globalFusion.fuseDetectionFrame(True, True)

                    if current_time % 1 == 0:
                        trupercept_vehicle_weight, error_at_100, twenty_percent_error_hit  = general_functions.cooperative_monitoring_process_truepercept(trupercept_data, globalFusionList, cav_pos, cis_pos, current_time, trupercept_dict, error_injection_time, error_at_100, twenty_percent_error_hit)
                
                    # Uses true positions of the CAVs to ground truth the sensing.
                    # This mode is only available when in simulation mode and unit testing.
                    # Ground truth the local fusion result
                    # Ground truth the global fusion result
                    if current_time >= error_injection_time:
                        over_detection_miss_g, under_detection_miss_g, differences_g = general_functions.ground_truth_dataset(globalFusionList, ground_truth)
                        global_differences += differences_g
                        global_over_detection_miss += over_detection_miss_g
                        global_under_detection_miss += under_detection_miss_g

                        # Calculate the results
                        rmse_val_g = shared_math.RMSE(global_differences)
                        variance_g = np.var(global_differences, ddof=1)
                        rmse = rmse_val_g
                        variance = variance_g

                    print(" Trupercept fusion completed step: ", current_time)

                with open('test_output/fast/rmse_truepercept.txt', 'a') as f:
                    f.write(str(rmse) + " ,")
                with open('test_output/fast/variance_truepercept.txt', 'a') as f:
                    f.write(str(variance) + " ,")

                print(" Trupercept fusion run complete -----")

                # Normal
                global_differences = []
                global_over_detection_miss = 0
                global_under_detection_miss = 0
                globalFusionList = []
                globalFusion = None
                globalFusion = global_fusion.GlobalFUSION(global_fusion_mode)
                error_injection_time = sorted(pickle_dict.keys())[-1] - 60.0
                twenty_percent_error_hit = False
                error_at_100 = 1.0
                for current_time in sorted(pickle_dict.keys()):
                    ground_truth = pickle_dict[current_time][0]
                    cav_cis_fusions = pickle_dict[current_time][1]

                    for cis_cav in cav_cis_fusions:
                        globalFusion.processDetectionFrame(current_time, cis_cav, .25, True)

                    globalFusionList, _, _ = globalFusion.fuseDetectionFrame(True, True)
                
                    # Uses true positions of the CAVs to ground truth the sensing.
                    # This mode is only available when in simulation mode and unit testing.
                    # Ground truth the local fusion result
                    # Ground truth the global fusion result
                    if current_time >= error_injection_time:
                        over_detection_miss_g, under_detection_miss_g, differences_g = general_functions.ground_truth_dataset(globalFusionList, ground_truth)
                        global_differences += differences_g
                        global_over_detection_miss += over_detection_miss_g
                        global_under_detection_miss += under_detection_miss_g

                        # Calculate the results
                        rmse_val_g = shared_math.RMSE(global_differences)
                        variance_g = np.var(global_differences, ddof=1)
                        rmse = rmse_val_g
                        variance = variance_g

                    print(" Global fusion completed step: ", current_time)

                with open('test_output/fast/rmse.txt', 'a') as f:
                    f.write(str(rmse) + " ,")
                with open('test_output/fast/variance.txt', 'a') as f:
                    f.write(str(variance) + " ,")

                print(" Global fusion run complete -----")
