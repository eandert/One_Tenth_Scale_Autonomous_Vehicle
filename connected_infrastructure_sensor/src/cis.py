import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Manager

from connected_infrastructure_sensor.src import communication, planning_stationary
from shared_library import local_fusion, sensor

# This function is for controlling the time function in case of simulation
def fetch_time(simulation_time, global_time = 1.0):
    if simulation_time:
        return global_time
    else:
        return time.time()

def update_time_from_rsu_sim(sensor_id, debug, rsu_sim_check = None):
    while(True):
        new_time = rsu_sim_check.getSimTime()["time"]
        if new_time != -99 and new_time != None:
            if debug: print( " CIS ", sensor_id, " got sim time from server ", new_time)
            return new_time
        elif new_time == -99:
            return -99

def cis(config, sid, test_idx):
    sensor_id = sid
    debug = config.debug
    simulation_time = False
    rsu = None
    global_time = -99
    bounding_box = [[0.0, 0.0],[0.0, 0.0]]
    last_response = []
    data_collect_mode = config.data_collect_mode
    if config.unit_test:
        local_fusion_mode = config.unit_test_config[test_idx][0]
    else:
        local_fusion_mode = 0

    print( " CIS ", sensor_id, " begin.")

    if not config.simulation:
    	# Do our imports within this function so we dont disturb the simulation
        from shared_library import camera_recognition

        # Init the camera
        # Setup our various settings
        settings = camera_recognition.Settings()
        settings.darknetPath = '../darknet/'
        camSpecs = camera_recognition.CameraSpecifications()
        camSpecs.cameraHeight = .2
        camSpecs.cameraAdjustmentAngle = 0.0
        if data_collect_mode:
            settings.record = True
            settings.outputFilename = "live_test_output.avi"
    else:
        simulation_time = True
        cavs = config.cav
        ciss = config.cis
        cooperative_monitoring = config.cooperative_monitoring
        cooperative_monitoring_update = config.cooperative_monitoring_update
        cooperative_monitoring_step = 0
        global_time = 1.0 # This must start as nonzero else Python will confuse with none

    # Set up the timing
    if config.simulation:
        start_time = fetch_time(simulation_time, global_time)
    else:
        start_time = fetch_time(simulation_time, global_time) + config.init_time
    interval = config.interval
    interval_offset = config.offset_interval
    fallthrough_delay = config.fallthrough_delay

    if not config.simulation:
        # Create the camera class
        cameraRecognition = camera_recognition.Camera(settings, camSpecs, False)

    # Start the connection with the RSU (Road Side Unit) server through sockets
    rsu = communication.connectServer(config.rsu_ip)
    response = rsu.register(sensor_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, fetch_time(simulation_time, global_time))

    # Now wait for the response
    while 'route_x' not in response:
        time.sleep(.01)

    if debug: print( " CIS ", sensor_id, " init pos ", response["t_x"], response["t_y"], response["t_yaw"])

    # Save the specs we got back from the RSU in case we need them someday
    specs = [response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"], response["route_TFL"]]

    # We need to intiialize the "planner" class to store these values
    sensor_planner = planning_stationary.Planner()
    sensor_planner.initialSensorAtPosition(response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"],
                                    response["route_TFL"], sensor_id, config.simulation)
    if debug: print( " CIS ", sensor_id, " planner initialized " , sensor_planner.localizationPositionX, sensor_planner.localizationPositionY, sensor_planner.theta, sensor_planner.positionX_offset, sensor_planner.positionY_offset, sensor_planner.theta_offset)

    # Fails keeps track of how many tries to connect with 
    fails = 0

    # Start the sensor fusion pipeline
    fusion = local_fusion.FUSION(local_fusion_mode, sensor_id)
    if debug: print( " CIS ", sensor_id, " started fusion node" )

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time, global_time) - .01
    if wait_until_start > 0 and not config.simulation:
        time.sleep(wait_until_start)

    next_time = start_time + interval_offset
    if debug: print( " CIS ", sensor_id, " start time is ", next_time)

    while True:
        if config.simulation:
            global_time = update_time_from_rsu_sim(sensor_id, debug, rsu)
            if global_time == -99:
                exit(0)
        if fetch_time(simulation_time, global_time) >= next_time:
            if config.simulation:
                # Receive positions of other vehicles from RSU so we can fake the sensor values
                sim_values = rsu.getSimPositions(sensor_id)
                while(sim_values['step_sim_vehicle'] == False):
                    time.sleep(.01)
                    sim_values = rsu.getSimPositions(sensor_id)
                    if debug: print( " CIS ", sensor_id, " requesting simulation positions" )
                
                vehicle_object_positions = sim_values['veh_locations']

                cam_returned, lidar_returned = sensor.simulate_sensors(sensor_planner, None, fetch_time(simulation_time, global_time), sim_values, vehicle_object_positions)
                camcoordinates = cam_returned[0]
                camtimestamp = cam_returned[1]

            else:
                # Process the camera frame
                localization_error_gaussian, localization_error = sensor_planner.localization.getErrorParamsAtVelocity(abs(sensor_planner.velocity), sensor_planner.theta)
                camcoordinates, camtimestamp = cameraRecognition.takeCameraFrame(settings, camSpecs)

                # There is no LIDAR but this contains our localization
                lidar_returned = [[], [], None]
                lidar_returned[0] = [specs[0], specs[1], specs[2], 0.0, localization_error_gaussian.covariance.tolist()]

            localization = lidar_returned[0]

            # Fusion
            fusion_result = []
            fusion_start = fetch_time(simulation_time, global_time)
            if not data_collect_mode:
                fusion.processDetectionFrame(local_fusion.CAMERA, camtimestamp, camcoordinates, .25, sim_values['parameterized_covariance'])
                fusion_result = fusion.fuseDetectionFrame()
        
            # Message the RSU, for now we must do this before our control loop
            # as the RSU has the traffic light state information
            objectPackage = {
                "localization_t": camtimestamp,
                "localization": localization,
                "lidar_t": camtimestamp,
                "lidar_detection_raw": [],
                "lidar_obj": [],
                "cam_t": camtimestamp,
                "cam_obj": camcoordinates,
                "fused_t": fusion_start,
                "fused_obj": fusion_result
            }
            
            # Dummy value used if we don't do a round
            bosco_results = None

            # Send and recieve messages for Bosco
            if cooperative_monitoring and cooperative_monitoring_step >= cooperative_monitoring_update:
                cooperative_monitoring_step = 1
                monitor = True
            else:
                cooperative_monitoring_step += 1
                monitor = False
            
            if monitor:
                import json, os

                start = time.time()

                bosco_id = sensor_id
                sensor_platform_ids = len(cavs) + len(ciss)
                data = str([sensor_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2], objectPackage, fetch_time(simulation_time, global_time)])

                # Create a "message" using a file for each of our other vehicles
                for platform_id in range(sensor_platform_ids):
                    if platform_id != bosco_id:
                        with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_init.txt", 'w') as f:
                            json.dump(data, f, sort_keys=True)

                # Wait some arbitrary time so everyone can write their files (this is hacky)
                time.sleep(1)

                # Read the messages from the other vehicles and delete after reading
                recieved_data_init = []
                for platform_id in range(sensor_platform_ids):
                    if platform_id == bosco_id:
                        recieved_data_init.append(data)
                    else:
                        if os.path.exists("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt"):
                            with open("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt", 'r') as f:
                                recieved_data_init.append(json.load(f))
                            os.remove("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt")
                        else:
                            print("The file does not exist")
                        

                # Now send these larger data packets to all vehicles again
                for platform_id in range(sensor_platform_ids):
                    if platform_id != bosco_id:
                        with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_final.txt", 'w') as f:
                            json.dump(recieved_data_init, f, sort_keys=True)

                # Wait some arbitrary time so everyone can write their files (this is hacky)
                time.sleep(1)

                # Read the messages from the other vehicles and delete after reading
                recieved_data_final = []
                for platform_id in range(sensor_platform_ids):
                    if platform_id == bosco_id:
                        recieved_data_final.append(recieved_data_init)
                    else:
                        if os.path.exists("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt"):
                            with open("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt", 'r') as f:
                                recieved_data_final.append(json.load(f))
                            os.remove("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt")
                        else:
                            print("The file does not exist")

                # Add bosco here using the values stored in recieved_data_final
                # recieved_data_final
                bosco_results = [True, []]
                for check_value in recieved_data_final:
                    is_good = []
                    for check_against in recieved_data_final:
                        #print( "                 +++" + str(check_value))
                        #print( "                 ---" + str(check_against))
                        if check_value == check_against:
                            is_good.append(True)
                            #print(True)
                        else:
                            is_good.append(False)
                            #print(False)
                    consensus_result = True
                    for good_or_naw in is_good:
                        if not good_or_naw:
                            consensus_result = False
                    bosco_results[1].append(consensus_result)
                    #print(consensus_result)

                # Faking the results
                # [overal_round_finished_boolean, [cav/cis_0_agrees_boolean, cav/cis_1_agrees_boolean, ..., cav/cis_n_agrees_boolean]]
                # bosco_results = [True, []]
                # for each in range(sensor_platform_ids):
                #     bosco_results[1].append(True)

                print(" ++++++++++++++++++++ Consensus time taken= ", time.time() - start - 2.0)

            response_checkin = rsu.checkin(sensor_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2], objectPackage, fetch_time(simulation_time, global_time))

            # Check if our result is valid
            if response_checkin == None:
                # The central controller may be down
                print ( "Error: RSU response not recieved in time cis ", sensor_id )
                if fails < 20:
                    fails += 1
                else:
                    print ( " CIS ", sensor_id, "Attempting to re-register with RSU" )
                    # We have failed a lot lets try to re-register but use our known location
                    response = rsu.register(sensor_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2], fetch_time(simulation_time, global_time))
            else:
                # Nothing to update since we dont move!
                fails = 0

            print ( " Time taken: " , time.time() - camtimestamp, time.time() )

