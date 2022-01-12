import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Manager

from connected_autonomous_vehicle.src import communication
from shared_library import local_fusion, sensor

# This function is for controlling the time function in case of simulation
def fetch_time(simulation_time, global_time = 0.0):
    if simulation_time:
        return global_time
    else:
        return time.time()

# This function is for controlling the time function in case of simulation
def fetch_new_time(simulation_time, rsu_sim_check = None):
    if simulation_time:
        returned = rsu_sim_check.getSimTime()
        if returned["time"] != -99:
            return returned["time"]
        else:
            return -99


def sourceImagesThread(out_queue, settings, camSpecs, simulation_time, data_collect_mode, start_time, interval):
    # DO our imports within this function so we dont disturb the simulation
    from connected_autonomous_vehicle.src import camera_recognition
    
    # Init the camera class
    cameraRecognition = camera_recognition.Camera(settings, camSpecs)

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time) - .01
    if wait_until_start > 0:
        time.sleep(wait_until_start)

    target = start_time

    # Now wait and ask for input
    while 1:
        if fetch_time(simulation_time) >= target:
            #print( "camera")
            now = fetch_time(simulation_time)

            # Take the camera frame and process
            if data_collect_mode:
                # Only collecting data, skip processing
                camcoordinates = []
                camtimestamp_start, frame = cameraRecognition.takeCameraFrameRaw()
            else:
                camcoordinates, camtimestamp_start, camtimestamp_end = cameraRecognition.takeCameraFrame()
            print ( "CAM send time " + str(fetch_time(simulation_time)))

            # Prep value to be sent to the part of the program
	    # Clear the queue of old data
            while not out_queue.empty():
                out_queue.get()
            out_queue.put([camcoordinates, camtimestamp_start, now])

            # Sleep to reduce busy wait
            #wait_until_next = round(fetch_time(simulation_time, global_time),3) % interval - .001
            #if wait_until_next > 0:
            #    time.sleep(wait_until_next)

            with open("cam_output.txt", 'a') as file1:
                file1.write(str(camtimestamp_start) + ',' + str(frame) + '\n')

            # New target
            target = target + interval
        time.sleep(.001)

def processCommunicationsThread(comm_q, v_id, init, response, rsu_ip):
    vehicle_id = v_id
    # Start the connection with the RSU (Road Side Unit) server through sockets
    rsu = communication.connectServer(rsu_ip)
    init_returned = rsu.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    init["t_x"] = init_returned["t_x"]
    init["t_y"] = init_returned["t_y"]
    init["t_yaw"] = init_returned["t_yaw"]
    init["route_x"] = init_returned["route_x"]
    init["route_y"] = init_returned["route_y"]
    init["route_TFL"] = init_returned["route_TFL"]

    # Fails keeps track of how many tries to connect with the RSU
    fails = 0

    # Now wait for input
    while 1:
        if not comm_q.empty():
            got = comm_q.get()
            x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, objectPackage = got

            response_message = rsu.checkin(vehicle_id, x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, objectPackage)

            # Check if our result is valid
            if response_message == None:
                response["error"] = 1
                print("Error: RSU response not recieved in time, stopping")
                if fails < 20:
                    fails += 1
                else:
                    print("Attempting to re-register with RSU")
                    # We have failed a lot lets try to re-register but use our known location
                    response_message = rsu.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            else:
                response["v_t"] = 0
                response["tfl_state"] = response_message["tfl_state"]
                response["veh_locations"] = []
                response["error"] = 0
                fails = 0

def cis(config, vid):
    # The first thing we should always do is initialize the control module
    # This is important to make sure a rogue signal doesn't drive us away
    # We do not need this if this is simulation
    vehicle_id = vid
    debug = config.debug
    simulation_time = False
    rsu_sim_check = None
    global_time = -99
    bounding_box = [[0.0, 0.0],[0.0, 0.0]]
    last_response = []
    data_collect_mode = config.data_collect_mode


    if not config.simulation:
	# Do our imports within this function so we dont disturb the simulation
        from connected_autonomous_vehicle.src import camera_recognition

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
        global_time = 0.0

    # Set up the timing
    if config.simulation:
        start_time = fetch_time(simulation_time, global_time)
    else:
        start_time = fetch_time(simulation_time, global_time) + config.init_time
    interval = config.interval
    interval_offset = config.offset_interval
    fallthrough_delay = config.fallthrough_delay

    if not config.simulation:
        # Spawn the camera processing thread
        cam_out_queue = Queue()
        cameraThread = Process(target=sourceImagesThread, args=(cam_out_queue, settings, camSpecs, simulation_time, data_collect_mode, start_time, interval))
        cameraThread.start()

        # Spawn the communication thread
        manager = Manager()
        init = manager.dict()
        response = manager.dict()
        response["error"] = 1
        comm_q = Queue()
        commThread = Process(target=processCommunicationsThread, args=(comm_q, vehicle_id, init, response, config.rsu_ip))
        commThread.start()
    else:
        # We are in sumulation and cannot start unlimited threads so this will be done in series
        fails = 0        
        simulation_time = True
        global_time = 0.0
        init = {}
        response = {}

        # Manually do what the thread would normally do
        if debug: print( " Vehicle ", vehicle_id, " connecting to RSU ", config.rsu_ip)
        # Start the connection with the RSU (Road Side Unit) server through sockets
        rsu_sim_check = communication.connectServer(config.rsu_ip)
        if debug: print( " Vehicle ", vehicle_id, " connected to RSU ", config.rsu_ip)
        init_returned = rsu_sim_check.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if debug: print( " Vehicle ", vehicle_id, " registered with RSU ", config.rsu_ip)

        # Store the init values for continuous use
        init["t_x"] = init_returned["t_x"]
        init["t_y"] = init_returned["t_y"]
        init["t_yaw"] = init_returned["t_yaw"]
        init["route_x"] = init_returned["route_x"]
        init["route_y"] = init_returned["route_y"]
        init["route_TFL"] = init_returned["route_TFL"]

    # Now wait for the response
    while 'route_x' not in init:
        time.sleep(.01)

    # If this is a simulation we need a second communication class to get some ideal positions
    if config.simulation:
        global_time = rsu_sim_check.getSimTime()['time']
        if debug: print( " Vehicle ", vehicle_id, " got sim time from server ", global_time )

    # Start the sensor fusion pipeline
    fusion = local_fusion.FUSION(0, vehicle_id)
    if debug: print( " Vehicle ", vehicle_id, " started fusion node" )

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time, global_time) - .01
    if wait_until_start > 0 and not config.simulation:
        time.sleep(wait_until_start)

    next_time = start_time + interval_offset
    if debug: print( " Vehicle ", vehicle_id, " start time is ", next_time, " now is ", fetch_time(simulation_time, global_time))

    last_lidar_time = fetch_time(simulation_time, global_time)

    while True:
        if config.simulation:
            new_time = fetch_new_time(simulation_time, rsu_sim_check)
            if new_time!= -99:
                global_time = new_time
            print ( fetch_time(simulation_time, global_time), next_time )
        if fetch_time(simulation_time, global_time) >= next_time:
            if config.simulation:
                # Special simulation setup where we do not use the source threads
                # Update the localization first because we use it here

                # Send the lolcailzation values to the RSU
                rsu_sim_check.sendSimPosition(vehicle_id, planner.positionX_sim, planner.positionY_sim, 0.0, 0.0, 0.0, planner.theta, planner.velocity)
                if debug: print( " Vehicle ", vehicle_id, " sent simulated position to RSU" )

                # Recieve positions of other vehicles from RSU so we can fake the sensor values
                sim_values = rsu_sim_check.getSimPositions(vehicle_id)
                while(sim_values['step_sim_vehicle'] == False):
                    #time.sleep(.01)
                    sim_values = rsu_sim_check.getSimPositions(vehicle_id)
                
                tempList = sim_values['veh_locations']
                cam_returned = [[], None]
                
                # Faking sensor values according to configuration
                if sim_values["estimate_covariance"]:
                    temp_covariance = localization_error_gaussian
                else:
                    temp_covariance = sensor.BivariateGaussian(0.175, 0.175, 0)
                point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(planner, tempList, [], 15.0, 15.0, 0.0, 160.0, l_error = localization_error, l_error_gauss = temp_covariance)
                lidar_returned[0] = [planner.localizationPositionX + localization_error[0], planner.localizationPositionY + localization_error[1],
                                    planner.theta, planner.velocity, temp_covariance.covariance.tolist()]
                if sim_values["simulate_error"]:
                    cam_returned[0] = camera_error_array
                    cam_returned[1] = fetch_time(simulation_time, global_time)
                    planner.localizationError = localization_error_gaussian
                    if sim_values["real_lidar"]:
                        # TODO: check this seems wrong
                        lidar_returned[1], lidar_returned[2] = lidarRecognition.processLidarFrame(point_cloud_error, fetch_time(simulation_time, global_time),
                            lidar_returned[0][0], lidar_returned[0][1], lidar_returned[0][2], planner.lidarSensor)
                        planner.rawLidarDetections = point_cloud_error
                    else:
                        lidar_returned[1] = lidar_detected_error
                else:
                    cam_returned[0] = camera_array
                    cam_returned[1] = fetch_time(simulation_time, global_time)
                    lidar_returned[1], lidar_returned[2] = lidarRecognition.processLidarFrame(point_cloud, fetch_time(simulation_time, global_time),
                        lidar_returned[0][0], lidar_returned[0][1], lidar_returned[0][2], planner.lidarSensor)
                    planner.rawLidarDetections = point_cloud
                planner.groundTruth = camera_array

                # Raw LIDAR for debug
                planner.lidarPoints = point_cloud

                camera_recieved = True
            else:
                # This is not simulation so we need to get the streams from the LIDAR and camera
                camera_recieved = False

                fallthrough = fetch_time(simulation_time, global_time) + fallthrough_delay
                while(fetch_time(simulation_time, global_time) < fallthrough and not (camera_recieved)):
                    # Get the camera
                    if not cam_out_queue.empty():
                        cam_returned = cam_out_queue.get()
                        #print( " Got camera " )
                        camera_recieved = True
            
            if camera_recieved:
                localization = [init["t_x"], init["t_y"], init["t_yaw"], 0.0, 0.0]
                camcoordinates = cam_returned[0]
                camtimestamp = cam_returned[1]
                # TODO: check the timestamps are close

                # Fusion
                fusion_result = []
                fusion_start = fetch_time(simulation_time, global_time)
                #fusion.processDetectionFrame(local_fusion.LIDAR, camtimestamp, camcoordinates, .25, 1)
                #fusion_result = fusion.fuseDetectionFrame(1, planner)

                # Message the RSU, for now we must do this before our control loop
                # as the RSU has the traffic light state information
                objectPackage = {
                    "localization_t": camtimestamp,
                    "localization": localization,
                    "lidar_t": [],
                    "lidar_obj": [],#lidarcoordinates,
                    "cam_t": camtimestamp,
                    "cam_obj": [],#camcoordinates,
                    "fused_t": fusion_start,
                    "fused_obj": fusion_result
                }

                if config.simulation:
                    response_message = rsu_sim_check.checkin(vehicle_id, localization[0], localization[1], 0.0, 0.0, 0.0, localization[2],
                            0.0, 0.0, 0, 0, objectPackage)

                    # Check if our result is valid
                    if response_message == None:
                        response["error"] = 1
                        print("Error: RSU response not recieved in time, stopping")
                        if fails < 5:
                            fails += 1
                        else:
                            print("Attempting to re-register with RSU")
                            # We have failed a lot lets try to re-register but use our known location
                            response_message = rsu_sim_check.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    else:
                        response["v_t"] = response_message["v_t"]
                        response["tfl_state"] = response_message["tfl_state"]
                        response["veh_locations"] = response_message["veh_locations"]
                        response["error"] = 0
                        fails = 0
                else:
                    comm_q.put(
                        [localization[0], localization[1], 0.0, 0.0, 0.0, localization[2],
                            0.0, 0.0, 0, 0, objectPackage])

                    # This should not take long but we will delay just a bit
                    time.sleep(.01)

                if response["error"] != 0:
                    # Cut the engine to make sure that we don't hit anything since the central controller is down
                    continue
                else:
                    continue
                print(" Time taken: ", fetch_time(simulation_time, global_time) - lidartimestamp, fetch_time(simulation_time, global_time) - camtimestamp, fetch_time(simulation_time, global_time))
            else:
                print(" Error, no camera frame returned ", camera_recieved)

            last_next_time = next_time
            while next_time <= fetch_time(simulation_time, global_time):
                next_time = last_next_time + interval
                last_next_time = next_time

            #time.sleep(last_next_time - .01)
        if config.simulation:
            if debug: print(" Vehicle ", vehicle_id, fetch_time(simulation_time, global_time))

        time.sleep(.001)

