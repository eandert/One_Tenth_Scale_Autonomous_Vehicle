import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Manager

from connected_infrastructure_sensor.src import communication, planning_stationary
from shared_library import local_fusion, sensor

# This function is for controlling the time function in case of simulation
def fetch_time(simulation_time, global_time = 1.0): #ORIG
    if simulation_time: #ORIG
        return global_time #ORIG
    else: #ORIG
        return time.time() #ORIG

def update_time_from_rsu_sim(sensor_id, debug, rsu_sim_check = None): #ORIG
    while(True): #ORIG
        new_time = rsu_sim_check.getSimTime()["time"] #ORIG
        if new_time != -99 and new_time != None: #ORIG
            if debug: print( " CIS ", sensor_id, " got sim time from server ", new_time) #ORIG
            return new_time #ORIG

def cis(config, sid): #BOTH
    sensor_id = sid #BOTH
    debug = config.debug #BOTH
    simulation_time = False #BOTH
    rsu = None #BOTH
    global_time = -99 #BOTH
    bounding_box = [[0.0, 0.0],[0.0, 0.0]] #BOTH
    last_response = [] #ORIG
    data_collect_mode = config.data_collect_mode #BOTH

    print( " CIS ", sensor_id, " begin.") #BOTH

    if not config.simulation: #BOTH
    	# Do our imports within this function so we dont disturb the simulation
        from shared_library import camera_recognition #BOTH

        # Init the camera
        # Setup our various settings
        settings = camera_recognition.Settings() #BOTH
        settings.darknetPath = '../darknet/' #BOTH
        camSpecs = camera_recognition.CameraSpecifications() #BOTH
        camSpecs.cameraHeight = .2 #BOTH
        camSpecs.cameraAdjustmentAngle = 0.0 #BOTH
        if data_collect_mode: #BOTH
            settings.record = True #BOTH
            settings.outputFilename = "live_test_output.avi" #BOTH
    else: #BOTH
        simulation_time = True #ORIG
        global_time = 1.0 # This must start as nonzero else Python will confuse with none #ORIG

    # Set up the timing
    if config.simulation: #ORIG
        start_time = fetch_time(simulation_time, global_time) #ORIG
    else:
        start_time = fetch_time(simulation_time, global_time) + config.init_time #ORIG
    interval = config.interval #ORIG
    interval_offset = config.offset_interval #ORIG
    fallthrough_delay = config.fallthrough_delay #ORIG

    if not config.simulation: #BOTH
        # Create the camera class
        cameraRecognition = camera_recognition.Camera(settings, camSpecs, False) #BOTH

    # Start the connection with the RSU (Road Side Unit) server through sockets
    rsu = communication.connectServer(config.rsu_ip) #ORIG_NETWORKING
    response = rsu.register(sensor_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)#ORIG_NETWORKING

    if debug: print( " CIS ", sensor_id, " init pos ", response["t_x"], response["t_y"], response["t_yaw"])#ORIG_NETWORKING

    # Save the specs we got back from the RSU in case we need them someday
    specs = [response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"], response["route_TFL"]]#ORIG_NETWORKING

    # We need to intiialize the "planner" class to store these values
    sensor_planner = planning_stationary.Planner() #BOTH
    sensor_planner.initialSensorAtPosition(response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"],
                                    response["route_TFL"], sensor_id, config.simulation) #BOTH

    # Fails keeps track of how many tries to connect with 
    fails = 0 #ORIG

    # Start the sensor fusion pipeline
    fusion = local_fusion.FUSION(0, sensor_id) #BOTH
    if debug: print( " CIS ", sensor_id, " started fusion node" ) #BOTH

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time, global_time) - .01 #ORIG
    if wait_until_start > 0 and not config.simulation: #ORIG
        time.sleep(wait_until_start) #ORIG

    next_time = start_time + interval_offset #ORIG
    if debug: print( " CIS ", sensor_id, " start time is ", next_time) #ORIG

    while True: #ORIG
        if config.simulation: #ORIG
            global_time = update_time_from_rsu_sim(sensor_id, debug, rsu) #ORIG
        if fetch_time(simulation_time, global_time) >= next_time: #ORIG
            if config.simulation: #BOTH
                # Recieve positions of other vehicles from RSU so we can fake the sensor values
                sim_values = rsu.getSimPositions(sensor_id) #BOTH
                while(sim_values['step_sim_vehicle'] == False): #ORIG_NETWORKING
                    time.sleep(.01) #ORIG_NETWORKING
                    sim_values = rsu.getSimPositions(sensor_id) #ORIG_NETWORKING
                    if debug: print( " CIS ", sensor_id, " requesting simulation positions" ) #ORIG_NETWORKING
                
                tempList = sim_values['veh_locations'] #BOTH
                cam_returned = [[], None] #BOTH

                # Faking sensor values according to configuration
                localization_error_gaussian, localization_error = sensor_planner.localization.getErrorParamsAtVelocity(abs(sensor_planner.velocity), sensor_planner.theta) #BOTH
                if sim_values["estimate_covariance"]: #BOTH
                    temp_covariance = localization_error_gaussian #BOTH
                else: #BOTH
                    temp_covariance = sensor.BivariateGaussian(0.01, 0.01, 0) #BOTH
                point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(sensor_planner, tempList, [], 15.0, 15.0, 0.0, 160.0) #BOTH
                
                if sim_values["simulate_error"]: #BOTH
                    cam_returned[0] = camera_error_array #BOTH
                    cam_returned[1] = fetch_time(simulation_time, global_time) #ORIG
                else: #BOTH
                    cam_returned[0] = camera_array #BOTH
                    cam_returned[1] = fetch_time(simulation_time, global_time) #ORIG
                camcoordinates = cam_returned[0] #BOTH
                camtimestamp =cam_returned[1] #BOTH
            else:
                # Process the camera frame
                localization_error_gaussian, localization_error = sensor_planner.localization.getErrorParamsAtVelocity(abs(sensor_planner.velocity), sensor_planner.theta) #BOTH
                camcoordinates, camtimestamp = cameraRecognition.takeCameraFrame(settings, camSpecs) #BOTH

            # There is no LIDAR but this contains our localization
            lidar_returned = [[], [], None] #BOTH
            lidar_returned[0] = [specs[0], specs[1], specs[2], 0.0, temp_covariance.covariance.tolist()] #BOTH

            localization = lidar_returned[0] #BOTH

            # Fusion
            fusion_result = [] #BOTH
            fusion_start = fetch_time(simulation_time, global_time) #BOTH
            if not data_collect_mode: #BOTH
                fusion.processDetectionFrame(local_fusion.CAMERA, camtimestamp, camcoordinates, .25, 1) #BOTH
                fusion_result = fusion.fuseDetectionFrame(1, sensor_planner) #BOTH
        
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
            } #BOTH

            response_checkin = rsu.checkin(sensor_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2], objectPackage) #ORIG_NETWORKING

            # Check if our result is valid
            if response_checkin == None: #ORIG_NETWORKING
                # The central controller may be down
                print ( "Error: RSU response not recieved in time cis ", sensor_id ) #ORIG_NETWORKING
                if fails < 20: #ORIG_NETWORKING
                    fails += 1 #ORIG_NETWORKING
                else: #ORIG_NETWORKING
                    print ( " CIS ", sensor_id, "Attempting to re-register with RSU" ) #ORIG_NETWORKING
                    # We have failed a lot lets try to re-register but use our known location
                    response = rsu.register(sensor_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2]) #ORIG_NETWORKING
            else:
                # Nothing to update since we dont move!
                fails = 0 #ORIG_NETWORKING

            print ( " Time taken: " , time.time() - camtimestamp, time.time() ) #ORIG

