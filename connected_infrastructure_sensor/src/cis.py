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

def cis(config, sid):
    sensor_id = sid
    debug = config.debug
    simulation_time = False
    rsu = None
    global_time = -99
    bounding_box = [[0.0, 0.0],[0.0, 0.0]]
    last_response = []
    data_collect_mode = config.data_collect_mode

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
    response = rsu.register(sensor_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    if debug: print( " CIS ", sensor_id, " init pos ", response["t_x"], response["t_y"], response["t_yaw"])

    # Save the specs we got back from the RSU in case we need them someday
    specs = [response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"], response["route_TFL"]]

    # We need to intiialize the "planner" class to store these values
    sensor_planner = planning_stationary.Planner()
    sensor_planner.initialSensorAtPosition(response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"],
                                    response["route_TFL"], sensor_id, config.simulation)

    # Fails keeps track of how many tries to connect with 
    fails = 0

    # Start the sensor fusion pipeline
    fusion = local_fusion.FUSION(0, sensor_id)
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
        if fetch_time(simulation_time, global_time) >= next_time:
            if config.simulation:
                # Recieve positions of other vehicles from RSU so we can fake the sensor values
                sim_values = rsu.getSimPositions(sensor_id)
                while(sim_values['step_sim_vehicle'] == False):
                    time.sleep(.01)
                    sim_values = rsu.getSimPositions(sensor_id)
                    if debug: print( " CIS ", sensor_id, " requesting simulation positions" )
                
                tempList = sim_values['veh_locations']
                cam_returned = [[], None]

                # Faking sensor values according to configuration
                localization_error_gaussian, localization_error = sensor_planner.localization.getErrorParamsAtVelocity(abs(sensor_planner.velocity), sensor_planner.theta)
                if sim_values["estimate_covariance"]:
                    temp_covariance = localization_error_gaussian
                else:
                    temp_covariance = sensor.BivariateGaussian(0.01, 0.01, 0)
                point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(sensor_planner, tempList, [], 15.0, 15.0, 0.0, 160.0)
                
                if sim_values["simulate_error"]:
                    cam_returned[0] = camera_error_array
                    cam_returned[1] = fetch_time(simulation_time, global_time)
                else:
                    cam_returned[0] = camera_array
                    cam_returned[1] = fetch_time(simulation_time, global_time)
                camcoordinates = cam_returned[0]
                camtimestamp =cam_returned[1]
            else:
                # Process the camera frame
                localization_error_gaussian, localization_error = sensor_planner.localization.getErrorParamsAtVelocity(abs(sensor_planner.velocity), sensor_planner.theta)
                camcoordinates, camtimestamp = cameraRecognition.takeCameraFrame(settings, camSpecs)

            # There is no LIDAR but this contains our localization
            lidar_returned = [[], [], None]
            lidar_returned[0] = [specs[0], specs[1], specs[2], 0.0, temp_covariance.covariance.tolist()]

            localization = lidar_returned[0]

            # Fusion
            fusion_result = []
            fusion_start = fetch_time(simulation_time, global_time)
            if not data_collect_mode:
                fusion.processDetectionFrame(local_fusion.CAMERA, camtimestamp, camcoordinates, .25, 1)
                fusion_result = fusion.fuseDetectionFrame(1, sensor_planner)
        
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

            response_checkin = rsu.checkin(sensor_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2], objectPackage)

            # Check if our result is valid
            if response_checkin == None:
                # The central controller may be down
                print ( "Error: RSU response not recieved in time cis ", sensor_id )
                if fails < 20:
                    fails += 1
                else:
                    print ( " CIS ", sensor_id, "Attempting to re-register with RSU" )
                    # We have failed a lot lets try to re-register but use our known location
                    response = rsu.register(sensor_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2])
            else:
                # Nothing to update since we dont move!
                fails = 0

            print ( " Time taken: " , time.time() - camtimestamp, time.time() )
