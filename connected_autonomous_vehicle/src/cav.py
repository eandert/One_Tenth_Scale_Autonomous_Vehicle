from sre_constants import JUMP
import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Manager

from connected_autonomous_vehicle.src import planning_control, communication
from shared_library import local_fusion, sensor, lidar_recognition

#import local_fusion

pipeFromC = "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc"  #BOTH
pipeToC = "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc" #BOTH

# This function is for controlling the time function in case of simulation
def fetch_time(simulation_time, global_time = 1.0): #ORIG
    if simulation_time: #ORIG
        return global_time #ORIG
    else: #ORIG
        return time.time() #ORIG

def update_time_from_rsu_sim(vehicle_id, debug, rsu_sim_check = None):  #ORIG
    while(True): #ORIG
        new_time = rsu_sim_check.getSimTime()["time"] #ORIG
        if new_time != -99 and new_time != None: #ORIG
            if debug: print( " Vehicle ", vehicle_id, " got sim time from server ", new_time) #ORIG
            return new_time #ORIG

def sourceImagesThread(out_queue, settings, camSpecs, simulation_time, data_collect_mode, start_time, interval):  #BOTH
    # DO our imports within this function so we dont disturb the simulation
    from connected_autonomous_vehicle.src import camera_recognition #BOTH
    
    # Init the camera class
    cameraRecognition = camera_recognition.Camera(settings, camSpecs) #BOTH

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time) - .01 #ORIG
    if wait_until_start > 0: #ORIG
        time.sleep(wait_until_start) #ORIG

    target = start_time #ORIG

    # Now wait and ask for input
    while 1: #ORIG
        if fetch_time(simulation_time) >= target: #ORIG
            #print( "camera")
            now = fetch_time(simulation_time) #ORIG

            # Take the camera frame and process
            if data_collect_mode: #BOTH
                # Only collecting data, skip processing
                camcoordinates = [] #BOTH
                camtimestamp_start, frame = cameraRecognition.takeCameraFrameRaw() #BOTH
            else: #BOTH
                camcoordinates, camtimestamp_start, camtimestamp_end = cameraRecognition.takeCameraFrame() #BOTH
            #print ( "CAM got " + str(fetch_time(simulation_time, global_time)))

            # Prep value to be sent to the part of the program
	    # Clear the queue of old data
            while not out_queue.empty(): #ORIG_QUEUEING
                out_queue.get() #ORIG_QUEUEING 
            out_queue.put([camcoordinates, camtimestamp_start, now]) #ORIG_QUEUEING

            with open("cam_output.txt", 'a') as file1: #BOTH
                file1.write(str(camtimestamp_start) + ',' + str(frame) + '\n') #BOTH

            # New target
            target = target + interval #ORIG
        time.sleep(.001) #ORIG


def sourceLIDARThread(out_queue, pipeFromC, pipeToC, lidarSensor, simulation_time, data_collect_mode, start_time, interval): #BOTH
    global bounding_box #BOTH
    # Start the connection with the LIDAR through pipes
    lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC) #BOTH
    target = interval #ORIG

    # Init the LIDAR processing class
    lidarRecognition = lidar_recognition.LIDAR(fetch_time(simulation_time)) #BOTH

    # Wait for 1 second before we start everything
    time.sleep(2) #ORIG

    start, end = lidarDevice.getFromC() #BOTH
    lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(lidarDevice.parseFromC(),
                                                                          start, 
                                                                          lidarDevice.localizationX,
                                                                          lidarDevice.localizationY, 
                                                                          lidarDevice.localizationYaw,
                                                                          lidarSensor)  #BOTH

    index = 0 #ORIG_PLANB

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time) - .01 #ORIG
    if wait_until_start > 0: #ORIG
        time.sleep(wait_until_start) #ORIG

    target = start_time #ORIG

    # Now wait for input
    while 1: #ORIG
        if fetch_time(simulation_time) >= target: #ORIG
            # Take the LIDAR frame and process
            #print ( "LIDAR start " + str(fetch_time(simulation_time, global_time)))
            start, end = lidarDevice.getFromC()
            #print ( "LIDAR got " + str(fetch_time(simulation_time, global_time)))
            if data_collect_mode:
                # Only collecting data, skip processing
                raw_lidar = lidarDevice.parseFromC() #BOTH
                lidarcoordinates = [] #BOTH
                lidartimestamp = end #BOTH
            else:
                lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(lidarDevice.parseFromC(),
                                                                                    start,
                                                                                    lidarDevice.localizationX,
                                                                                    lidarDevice.localizationY, 
                                                                                    lidarDevice.localizationYaw,
                                                                                    lidarSensor) #BOTH
            localization = [lidarDevice.localizationX, lidarDevice.localizationY, lidarDevice.localizationYaw, index, start] #BOTH
            #print ( "LIDAR detect " + str(fetch_time(simulation_time, global_time)))

            # Send the localization and coordinate results to the fusion function
	        # Clear the queue of old data
            while not out_queue.empty(): #ORIG_QUEUEING
                out_queue.get() #ORIG_QUEUEING
            out_queue.put([localization, lidarcoordinates, lidartimestamp]) #ORIG_QUEUEING

            # Log this to a file
            index += 1 #ORIG

            with open("lidar_output.txt", 'a') as file1: #BOTH
                file1.write(str(lidartimestamp) + '\n' + str(localization) + '\n' + str(raw_lidar) + '\n') #BOTH

            # New target
            target = target + interval #ORIG
        time.sleep(.001) #ORIG

def processCommunicationsThread(comm_q, v_id, init, response, rsu_ip): #ORIG
    vehicle_id = v_id #ORIG
    # Start the connection with the RSU (Road Side Unit) server through sockets
    rsu = communication.connectServer(rsu_ip) #ORIG_NETWORKING
    init_returned = rsu.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #ORIG_NETWORKING

    init["t_x"] = init_returned["t_x"] #ORIG_NETWORKING
    init["t_y"] = init_returned["t_y"] #ORIG_NETWORKING
    init["t_yaw"] = init_returned["t_yaw"] #ORIG_NETWORKING
    init["route_x"] = init_returned["route_x"] #ORIG_NETWORKING
    init["route_y"] = init_returned["route_y"] #ORIG_NETWORKING
    init["route_TFL"] = init_returned["route_TFL"] #ORIG_NETWORKING

    # Fails keeps track of how many tries to connect with the RSU
    fails = 0

    # Now wait for input
    while 1: #ORIG_QUEUEING
        if not comm_q.empty(): #ORIG_QUEUEING
            got = comm_q.get() #ORIG_QUEUEING
            x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, objectPackage = got #ORIG_QUEUEING

            response_message = rsu.checkin(vehicle_id, x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, objectPackage) #ORIG_NETWORKING

            # Check if our result is valid
            if response_message == None: #ORIG_NETWORKING
                response["error"] = 1 #ORIG_NETWORKING
                print("Error: RSU response not recieved in time, stopping") #ORIG_NETWORKING
                if fails < 20: #ORIG_NETWORKING
                    fails += 1 #ORIG_NETWORKING
                else: #ORIG_NETWORKING
                    print("Attempting to re-register with RSU") #ORIG_NETWORKING
                    # We have failed a lot lets try to re-register but use our known location
                    response_message = rsu.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #ORIG_NETWORKING
            else: #ORIG_NETWORKING
                response["v_t"] = response_message["v_t"] #ORIG_NETWORKING
                response["tfl_state"] = response_message["tfl_state"] #ORIG_NETWORKING
                response["veh_locations"] = response_message["veh_locations"] #ORIG_NETWORKING
                response["error"] = 0 #ORIG_NETWORKING
                fails = 0 #ORIG_NETWORKING

def cav(config, vid): #BOTH
    # The first thing we should always do is initialize the control module
    # This is important to make sure a rogue signal doesn't drive us away
    # We do not need this if this is simulation
    vehicle_id = vid #BOTH
    debug = config.debug #BOTH
    simulation_time = False #BOTH
    rsu_sim_check = None #ORIG
    global_time = -99 #ORIG
    bounding_box = [[0.0, 0.0],[0.0, 0.0]] #BOTH
    last_response = [] #ORIG
    data_collect_mode = config.data_collect_mode #BOTH

    print( " CAV ", vid, " begin.") #ORIG

    if not config.simulation: #BOTH
    	# Do our imports within this function so we dont disturb the simulation
        from connected_autonomous_vehicle.src import motors #BOTH
        from shared_library import camera_recognition #BOTH
        egoVehicle = motors.Motors() #BOTH

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
    else:
        simulation_time = True #ORIG
        global_time = 1.0 # This must start as nonzero else Python will confuse with none #ORIG

    # Set up the timing
    if config.simulation: #ORIG
        start_time = fetch_time(simulation_time, global_time) #ORIG
    else: #ORIG
        start_time = fetch_time(simulation_time, global_time) + config.init_time #ORIG
    interval = config.interval #ORIG
    interval_offset = config.offset_interval #ORIG
    fallthrough_delay = config.fallthrough_delay #ORIG

    # Initialize the planner
    planner = planning_control.Planner() #BOTH

    if not config.simulation: #BOTH
        # Spawn the camera processing thread
        cam_out_queue = Queue() #BOTH
        cameraThread = Process(target=sourceImagesThread, args=(cam_out_queue, settings, camSpecs, simulation_time, data_collect_mode, start_time, interval)) #BOTH
        cameraThread.start() #BOTH

        # Spawn the lidar processign thread
        lidar_out_queue = Queue() #BOTH
        cameraThread = Process(target=sourceLIDARThread, args=(lidar_out_queue, pipeFromC, pipeToC, planner.lidarSensor, simulation_time, data_collect_mode, start_time, interval)) #BOTH
        cameraThread.start() #BOTH

        # Spawn the communication thread
        manager = Manager() #ORIG
        init = manager.dict() #ORIG
        response = manager.dict() #ORIG
        response["error"] = 1 #ORIG
        comm_q = Queue() #ORIG_QUEUEING
        commThread = Process(target=processCommunicationsThread, args=(comm_q, vehicle_id, init, response, config.rsu_ip)) #ORIG_QUEUEING
        commThread.start() #ORIG_QUEUEING
    else:
        # We are in sumulation and cannot start unlimited threads so this will be done in series
        fails = 0 #ORIG
        simulation_time = True #ORIG
        global_time = 1.0 #ORIG
        init = {} #ORIG_QUEUEING
        response = {} #ORIG_QUEUEING
        lidarRecognition = lidar_recognition.LIDAR(0.0) #BOTH

        # Manually do what the thread would normally do
        if debug: print( " Vehicle ", vehicle_id, " connecting to RSU... ", config.rsu_ip) #ORIG
        # Start the connection with the RSU (Road Side Unit) server through sockets
        rsu_sim_check = communication.connectServer(config.rsu_ip) #ORIG_QUEUEING
        if debug: print( " Vehicle ", vehicle_id, " connected ", config.rsu_ip) #ORIG
        init_returned = rsu_sim_check.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #ORIG_QUEUEING
        if debug: print( " Vehicle ", vehicle_id, " registered with RSU ", config.rsu_ip) #ORIG

        # Store the init values for continuous use
        init["t_x"] = init_returned["t_x"] #ORIG_QUEUEING
        init["t_y"] = init_returned["t_y"] #ORIG_QUEUEING
        init["t_yaw"] = init_returned["t_yaw"] #ORIG_QUEUEING
        init["route_x"] = init_returned["route_x"] #ORIG_QUEUEING
        init["route_y"] = init_returned["route_y"] #ORIG_QUEUEING
        init["route_TFL"] = init_returned["route_TFL"] #ORIG_QUEUEING

    # Now wait for the response
    while 'route_x' not in init: #ORIG_NETWORKING
        time.sleep(.01) #ORIG

    if debug: print( " Vehicle ", vehicle_id, " init pos ", init["t_x"], init["t_y"], init["t_yaw"]) #ORIG

    # Now that we have chatted with the RSU server, we should know where we are going
    planner.initialVehicleAtPosition(init["t_x"], init["t_y"], init["t_yaw"], init["route_x"], init["route_y"],
                                    init["route_TFL"], vehicle_id, config.simulation) #BOTH
    if debug: print( " Vehicle ", vehicle_id, " planner initialized " , planner.localizationPositionX, planner.localizationPositionY, planner.theta, planner.positionX_offset, planner.positionY_offset, planner.theta_offset)  #ORIG

    # Do a quick check of the coordinates to make abounding box filter so that we can take out uneccessary points
    min_x = 99.0  #BOTH
    max_x = -99.0  #BOTH
    min_y = 99.0  #BOTH
    max_y = -99.0  #BOTH
    for x in init["route_x"]:  #BOTH
        if x > max_x:  #BOTH
            max_x = x  #BOTH
        if x < min_x:  #BOTH
            min_x = x  #BOTH
    for y in init["route_y"]:  #BOTH
        if y > max_y:  #BOTH
            max_y = y  #BOTH
        if y < min_y:  #BOTH
            min_y = y  #BOTH
    buffer = .25  #BOTH
    bounding_box = [[min_x - buffer, max_x + buffer],[min_y - buffer, max_y + buffer]]  #BOTH
    if debug: print( " Vehicle ", vehicle_id, " bounding box params ", bounding_box )  #BOTH

    # If this is a simulation we need a second communication class to get some ideal positions
    if config.simulation:  #BOTH
        global_time = update_time_from_rsu_sim(vehicle_id, debug, rsu_sim_check)  #BOTH
        print("global_time", global_time)  #BOTH

    # Start the sensor fusion pipeline
    fusion = local_fusion.FUSION(0, vehicle_id)  #BOTH
    if debug: print( " Vehicle ", vehicle_id, " started fusion node" )  #BOTH

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time, global_time) - .01  #ORIG
    if wait_until_start > 0 and not config.simulation: #ORIG
        time.sleep(wait_until_start) #ORIG

    next_time = start_time + interval_offset #ORIG
    if debug: print( " Vehicle ", vehicle_id, " start time is ", next_time) #ORIG

    last_lidar_time = fetch_time(simulation_time, global_time) #ORIG

    while True: #ORIG
        if config.simulation: #ORIG
            global_time = update_time_from_rsu_sim(vehicle_id, debug, rsu_sim_check) #ORIG
        if fetch_time(simulation_time, global_time) >= next_time: #ORIG
            if config.simulation: #BOTH
                # Special simulation setup where we do not use the source threads
                # Update the localization first because we use it here
                planner.updatePosition(interval) #BOTH
                planner.update_localization(use_localization = False) #BOTH
                if debug: print( " Vehicle ", vehicle_id, " posiiton and localization updated" ) #BOTH
                
                # Send the lolcailzation values to the RSU
                rsu_sim_check.sendSimPosition(vehicle_id, planner.localizationPositionX, planner.localizationPositionY, 0.0, 0.0, 0.0, planner.theta, planner.velocity) #BOTH
                if debug: print( " Vehicle ", vehicle_id, " sent simulated position to RSU" ) #BOTH

                # Recieve positions of other vehicles from RSU so we can fake the sensor values
                sim_values = rsu_sim_check.getSimPositions(vehicle_id) #ORIG_NETWORKING
                while(sim_values['step_sim_vehicle'] == False): #ORIG_NETWORKING
                    time.sleep(.01) #ORIG_NETWORKING
                    sim_values = rsu_sim_check.getSimPositions(vehicle_id) #ORIG_NETWORKING
                    if debug: print( " Vehicle ", vehicle_id, " requesting simulation positions" ) #ORIG_NETWORKING
                
                tempList = sim_values['veh_locations'] #BOTH
                lidar_returned = [[], [], None] #BOTH
                cam_returned = [[], None] #BOTH
                
                # Faking sensor values according to configuration
                localization_error_gaussian, localization_error = planner.localization.getErrorParamsAtVelocity(abs(planner.velocity), planner.theta) #BOTH
                if sim_values["estimate_covariance"]: #BOTH
                    temp_covariance = localization_error_gaussian #BOTH
                else: #BOTH
                    temp_covariance = sensor.BivariateGaussian(0.175, 0.175, 0) #BOTH
                point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(planner, tempList, [], 15.0, 15.0, 0.0, 160.0)  #BOTH
                lidar_returned[0] = [planner.localizationPositionX + localization_error[0], planner.localizationPositionY + localization_error[1],
                                    planner.theta, planner.velocity, temp_covariance.covariance.tolist()] #BOTH
                if sim_values["simulate_error"]: #BOTH
                    cam_returned[0] = camera_error_array #BOTH
                    cam_returned[1] = fetch_time(simulation_time, global_time) #BOTH
                    planner.localizationError = localization_error_gaussian #BOTH
                    if sim_values["real_lidar"]: #BOTH
                        # TODO: check this seems wrong
                        lidar_returned[1], lidar_returned[2] = lidarRecognition.processLidarFrame(point_cloud_error, fetch_time(simulation_time, global_time),
                            lidar_returned[0][0], lidar_returned[0][1], lidar_returned[0][2], planner.lidarSensor) #BOTH
                        planner.rawLidarDetections = point_cloud_error #BOTH
                    else: #BOTH
                        lidar_returned[1] = lidar_detected_error #BOTH
                        lidar_returned[2] = fetch_time(simulation_time, global_time) #ORIG
                        planner.rawLidarDetections = point_cloud_error #BOTH
                else: #BOTH
                    cam_returned[0] = camera_array #BOTH
                    cam_returned[1] = fetch_time(simulation_time, global_time) #ORIG
                    lidar_returned[1], lidar_returned[2] = lidarRecognition.processLidarFrame(point_cloud, fetch_time(simulation_time, global_time),
                        lidar_returned[0][0], lidar_returned[0][1], lidar_returned[0][2], planner.lidarSensor) #BOTH
                    planner.rawLidarDetections = point_cloud #BOTH
                planner.groundTruth = camera_array #BOTH

                # Raw LIDAR for debug
                planner.lidarPoints = point_cloud #BOTH

                lidar_recieved = True #ORIG
                camera_recieved = True #ORIG
            else:
                # This is not simulation so we need to get the streams from the LIDAR and camera
                lidar_recieved = False #ORIG
                camera_recieved = False #ORIG

                fallthrough = fetch_time(simulation_time, global_time) + fallthrough_delay  #ORIG
                while(fetch_time(simulation_time, global_time) < fallthrough and not (lidar_recieved and camera_recieved)):  #ORIG
                    # Get the lidar
                    if not lidar_out_queue.empty():  #ORIG_QUEUEING
                        lidar_returned = lidar_out_queue.get() #ORIG_QUEUEING
                        #print( " Got LIDAR " )
                        lidar_recieved = True #ORIG_QUEUEING

                    # Get the camera
                    if not cam_out_queue.empty(): #ORIG_QUEUEING
                        cam_returned = cam_out_queue.get() #ORIG_QUEUEING
                        #print( " Got camera " )
                        camera_recieved = True #ORIG_QUEUEING
            
            if lidar_recieved and camera_recieved: #BOTH
                localization = lidar_returned[0] #BOTH
                lidarcoordinates = lidar_returned[1] #BOTH
                lidartimestamp = lidar_returned[2] #BOTH
                lidarraw = planner.rawLidarDetections #BOTH
                camcoordinates = cam_returned[0] #BOTH
                camtimestamp = cam_returned[1] #BOTH
                # TODO: check the timestamps are close

                # Update the steering here while we wait for sensor fusion results and the reply from the RSU about the plan
                if not config.simulation: #BOTH
                    planner.update_localization(True, [localization[0], localization[1], localization[2]]) #BOTH
                    localization[0] = planner.localizationPositionX #BOTH
                    localization[1] = planner.localizationPositionY #BOTH
                    localization[2] = planner.theta #BOTH
                planner.pure_pursuit_control() #BOTH

                # # Now update our current PID with respect to other vehicles
                planner.check_positions_of_other_vehicles_adjust_velocity(last_response) #BOTH
                # # We can't update the PID controls until after all positions are known
                planner.update_pid() #BOTH

                # Finally, issue the commands to the motors
                steering_ppm, motor_pid = planner.return_command_package() #BOTH
                if not config.simulation: #BOTH
                    egoVehicle.setControlMotors(steering_ppm, motor_pid) #BOTH

                # Fusion
                fusion_result = [] #BOTH
                fusion_start = fetch_time(simulation_time, global_time) #ORIG
                if not data_collect_mode: #BOTH
                    fusion.processDetectionFrame(local_fusion.CAMERA, camtimestamp, camcoordinates, .25, 1) #BOTH
                    fusion.processDetectionFrame(local_fusion.LIDAR, lidartimestamp, lidarcoordinates, .25, 1) #BOTH
                    fusion_result = fusion.fuseDetectionFrame(1, planner) #BOTH

                # Message the RSU, for now we must do this before our control loop
                # as the RSU has the traffic light state information
                objectPackage = {
                    "localization_t": lidartimestamp,
                    "localization": localization,
                    "lidar_t": lidartimestamp,
                    "lidar_detection_raw": lidarraw,
                    "lidar_obj": lidarcoordinates,
                    "cam_t": camtimestamp,
                    "cam_obj": camcoordinates,
                    "fused_t": fusion_start,
                    "fused_obj": fusion_result
                } #ORIG_NETWORKING

                if config.simulation:
                    response_message = rsu_sim_check.checkin(vehicle_id, planner.localizationPositionX, planner.localizationPositionY, 0.0, 0.0, 0.0, planner.theta,
                            planner.steeringAcceleration, planner.motorAcceleration, planner.targetIndexX, planner.targetIndexY, objectPackage)  #ORIG_NETWORKING

                    # Check if our result is valid
                    if response_message == None:  #ORIG_NETWORKING
                        response["error"] = 1 #ORIG_NETWORKING
                        print("Error: RSU response not recieved in time, stopping") #ORIG_NETWORKING
                        if fails < 5: #ORIG_NETWORKING
                            fails += 1 #ORIG_NETWORKING
                        else: #ORIG_NETWORKING
                            print("Attempting to re-register with RSU") #ORIG_NETWORKING
                            # We have failed a lot lets try to re-register but use our known location
                            response_message = rsu_sim_check.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #ORIG_NETWORKING
                    else:
                        response["v_t"] = response_message["v_t"] #ORIG_NETWORKING
                        response["tfl_state"] = response_message["tfl_state"] #ORIG_NETWORKING
                        response["veh_locations"] = response_message["veh_locations"] #ORIG_NETWORKING
                        response["error"] = 0 #ORIG_NETWORKING
                        fails = 0 #ORIG_NETWORKING
                else:
                    comm_q.put(
                        [planner.rearAxlePositionX, planner.rearAxlePositionY, 0.0, 0.0, 0.0, planner.theta,
                            planner.steeringAcceleration, planner.motorAcceleration, planner.targetIndexX, planner.targetIndexY, objectPackage]) #ORIG_NETWORKING

                    # This should not take long but we will delay just a bit
                    time.sleep(.01) #ORIG

                if response["error"] != 0: #ORIG_NETWORKING
                    # Cut the engine to make sure that we don't hit anything since the central controller is down
                    if not config.simulation: #BOTH
                        egoVehicle.emergencyStop() #BOTH
                    # Log this to a file
                    # with open("timing.txt", 'a') as file1:
                    #     file1.write(None, start)
                    #     index += 1
                else:
                    # Update our various pieces
                    planner.targetVelocityGeneral = float(response["v_t"]) #BOTH
                    planner.recieve_coordinate_group_commands(response["tfl_state"]) #BOTH
                    planner.pure_pursuit_control() #BOTH

                    # Now update our current PID with respect to other vehicles
                    planner.check_positions_of_other_vehicles_adjust_velocity(response["veh_locations"]) #BOTH
                    if debug: print( " Vehicle ", vehicle_id, " target velocity ", planner.motorAcceleration, planner.targetVelocityGeneral, planner.targetVelocity, planner.targetFollowDistance, planner.followDistance ) #BOTH
                    last_response = response["veh_locations"] #BOTH
                    # We can't update the PID controls until after all positions are known
                    planner.update_pid() #BOTH
                    if debug: print( " Vehicle ", vehicle_id, " ppm, pwm ", steering_ppm, motor_pid ) #BOTH

                    # Finally, issue the commands to the motors
                    steering_ppm, motor_pid = planner.return_command_package() #BOTH
                    if not config.simulation: #BOTH
                        egoVehicle.setControlMotors(steering_ppm, motor_pid) #BOTH

                    # with open("timing.txt", 'a') as file1:
                    #     file1.write(lidarDevice.localizationIdx, fetch_time(simulation_time, global_time))
                    #     index += 1
                    last_lidar_time = fetch_time(simulation_time, global_time) #ORIG
                if not config.simulation:  #BOTH
                    if debug: print(" Time taken: ", fetch_time(simulation_time, global_time) - lidartimestamp, fetch_time(simulation_time, global_time) - camtimestamp, fetch_time(simulation_time, global_time)) #BOTH
            else: #ORIG_PLANB
                if debug: print(" Error, no camera/lidar frame returned ", lidar_recieved, camera_recieved) #ORIG_PLANB
                # Cut the engine to make sure that we don't hit anything since we are blind
                if not config.simulation and (fetch_time(simulation_time, global_time) - last_lidar_time) >= .250 : #ORIG_PLANB
                    egoVehicle.emergencyStop() #ORIG_PLANB

            last_next_time = next_time #ORIG
            while next_time <= fetch_time(simulation_time, global_time): #ORIG
                next_time = last_next_time + interval #ORIG
                last_next_time = next_time #ORIG

            #time.sleep(last_next_time - .01)
        if config.simulation: #ORIG
            if debug: print(" Vehicle ", vehicle_id, fetch_time(simulation_time, global_time)) #ORIG

        time.sleep(.001) #ORIG

    if not config.simulation: #ORIG_PLANB
        egoVehicle.emergencyStop() #ORIG_PLANB
