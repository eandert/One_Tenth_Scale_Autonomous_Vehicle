import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Manager

from connected_autonomous_vehicle.src import lidar_recognition, planning_control, communication
from shared_library import local_fusion, sensor

#import local_fusion

pipeFromC = "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc"
pipeToC = "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc"

vehicle_id = 0
dump_to_file = False
simulation_time = False
rsu_sim_check = None
global_time = -99
debug = False

# This function is for controlling the time function in case of simulation
def fetch_time():
    if simulation_time:
        global_time = rsu_sim_check.getSimTime()
        # Sleep so we dont overwhelm the server
        #time.sleep(.01)
        return global_time
    else:
        return time.time()


def sourceImagesThread(out_queue, settings, camSpecs, start_time, interval):
    # DO our imports within this function so we dont disturb the simulation
    from connected_autonomous_vehicle.src import camera_recognition

    # Init the camera class
    cameraRecognition = camera_recognition.Camera(settings, camSpecs)

    # Sleep until test start time
    wait_until_start = start_time - fetch_time() - .01
    if wait_until_start > 0:
        time.sleep(wait_until_start)

    # Now wait and ask for input
    while 1:
        if (round(fetch_time(),3) % interval) == 0.000:
            #print( "camera")
            now = fetch_time()

            # Take the camera frame and process
            camcoordinates, camtimestamp_start, camtimestamp_end = cameraRecognition.takeCameraFrame()
            print ( "CAM got " + str(fetch_time()))

            # Prep value to be sent to the part of the program
	    # Clear the queue of old data
            while not out_queue.empty():
                out_queue.get()
            out_queue.put([camcoordinates, camtimestamp_start, now])

            # Sleep to reduce busy wait
            #wait_until_next = round(fetch_time(),3) % interval - .001
            #if wait_until_next > 0:
            #    time.sleep(wait_until_next)


def sourceLIDARThread(out_queue, pipeFromC, pipeToC, lidarSensor, start_time, interval):
    # Start the connection with the LIDAR through pipes
    lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC)
    target = interval

    # Init the LIDAR processing class
    lidarRecognition = lidar_recognition.LIDAR(fetch_time())

    # Wait for 1 second before we start everything
    time.sleep(2)

    start, end = lidarDevice.getFromC()
    lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(lidarDevice.parseFromC(),
                                                                          start, 
                                                                          lidarDevice.localizationX,
                                                                          lidarDevice.localizationY, 
                                                                          lidarDevice.localizationYaw,
                                                                          lidarSensor)

    index = 0

    # Sleep until test start time
    wait_until_start = start_time - fetch_time() - .01
    if wait_until_start > 0:
        time.sleep(wait_until_start)

    # Now wait for input
    while 1:
        if (round(fetch_time(),3) % target) == 0.000:
            # Take the LIDAR frame and process
            #print ( "LIDAR start " + str(fetch_time()))
            start, end = lidarDevice.getFromC()
            print ( "LIDAR got " + str(fetch_time()))

            lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(lidarDevice.parseFromC(),
                                                                                start,
                                                                                lidarDevice.localizationX,
                                                                                lidarDevice.localizationY, 
                                                                                lidarDevice.localizationYaw,
                                                                                lidarSensor)
            localization = [lidarDevice.localizationX, lidarDevice.localizationY, lidarDevice.localizationYaw, index, start]
            #print ( "LIDAR detect " + str(fetch_time()))

            # Send the localization and coordinate results to the fusion function
	        # Clear the queue of old data
            while not out_queue.empty():
                out_queue.get()
            out_queue.put([localization, lidarcoordinates, lidartimestamp])

            # Log this to a file
            index += 1

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
            x, y, z, roll, pitch, yaw, objectPackage = got

            response_message = rsu.checkin(vehicle_id, x, y, z, roll, pitch, yaw, objectPackage)

            # Check if our result is valid
            if response_message == None:
                response["error"] = 1
                print("Error: RSU response not recieved in time, stopping")
                if fails < 20:
                    fails += 1
                else:
                    print("Attempting to re-register with RSU")
                    # We have failed a lot lets try to re-register but use our known location
                    response_message = rsu.register(vehicle_id, x, y, z, roll, pitch, yaw)
            else:
                response["v_t"] = response_message["v_t"]
                response["tfl_state"] = response_message["tfl_state"]
                response["veh_locations"] = response_message["veh_locations"]
                response["error"] = 0
                fails = 0

def cav(config, vid):
    # The first thing we should always do is initialize the control module
    # This is important to make sure a rogue signal doesn't drive us away
    # We do not need this if this is simulation
    global vehicle_id, debug, global_time
    vehicle_id = vid
    debug = config.debug
    last_response = []

    if not config.simulation:
        from connected_autonomous_vehicle.src import motors
        egoVehicle = motors.Motors()

        # Init the camera
        # Setup our various settings
        settings = camera_recognition.Settings()
        settings.darknetPath = '../darknet/'
        camSpecs = camera_recognition.CameraSpecifications()
        camSpecs.cameraHeight = .2
        camSpecs.cameraAdjustmentAngle = 0.0

    # Set up the timing
    start_time = fetch_time() + config.init_time
    interval = config.interval
    interval_offset = config.offset_interval
    fallthrough_delay = config.fallthrough_delay

    # Initialize the planner
    planner = planning_control.Planner()

    if not config.simulation:
        # Spawn the camera processing thread
        cam_out_queue = Queue()
        cameraThread = Process(target=sourceImagesThread, args=(cam_out_queue, settings, camSpecs, start_time, interval))
        cameraThread.start()

        # Spawn the lidar processign thread
        lidar_out_queue = Queue()
        cameraThread = Process(target=sourceLIDARThread, args=(lidar_out_queue, pipeFromC, pipeToC, planner.lidarSensor, start_time, interval))
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
        lidarRecognition = lidar_recognition.LIDAR(0.0)

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

    # Now that we have chatted with the RSU server, we should know where we are going
    planner.initialVehicleAtPosition(init["t_x"], init["t_y"], init["t_yaw"], init["route_x"], init["route_y"],
                                    init["route_TFL"], vehicle_id, config.simulation)
    if debug: print( " Vehicle ", vehicle_id, " planner initialized" )

    # If this is a simulation we need a second communication class to get some ideal positions
    if config.simulation:
        global_time = rsu_sim_check.getSimTime()['time']
        if debug: print( " Vehicle ", vehicle_id, " got sim time from server ", global_time )

    # Start the sensor fusion pipeline
    fusion = local_fusion.FUSION(0, vehicle_id)
    if debug: print( " Vehicle ", vehicle_id, " started fusion node" )

    # Sleep until test start time
    wait_until_start = start_time - fetch_time() - .01
    if wait_until_start > 0:
        time.sleep(wait_until_start)

    next_time = start_time + interval_offset
    if debug: print( " Vehicle ", vehicle_id, " start time is ", next_time)

    while True:
        if fetch_time() > next_time:
            if config.simulation:
                # Special simulation setup where we do not use the source threads
                # Update the localization first because we use it here
                planner.updatePosition(interval)
                planner.update_localization(use_localization = False)
                if debug: print( " Vehicle ", vehicle_id, " posiiton and localization updated" )
                
                # Send the lolcailzation values to the RSU
                rsu_sim_check.sendSimPosition(vehicle_id, planner.positionX_sim + planner.positionX_offset, planner.positionY_sim + planner.positionY_offset, 0.0, 0.0, 0.0, planner.theta + planner.theta_offset, planner.velocity)
                if debug: print( " Vehicle ", vehicle_id, " sent simulated position to RSU" )

                # Recieve positions of other vehicles from RSU so we can fake the sensor values
                sim_values = rsu_sim_check.getSimPositions(vehicle_id)
                while(sim_values['step_sim_vehicle'] == False):
                    #time.sleep(.01)
                    sim_values = rsu_sim_check.getSimPositions(vehicle_id)
                
                tempList = sim_values['veh_locations']
                lidar_returned = [[], [], None]
                cam_returned = [[], None]
                
                # Faking sensor values according to configuration
                localization_error_gaussian, localization_error = planner.localization.getErrorParamsAtVelocity(abs(planner.velocity), planner.theta)
                if sim_values["estimate_covariance"]:
                    temp_covariance = localization_error_gaussian
                else:
                    temp_covariance = sensor.BivariateGaussian(0.175, 0.175, 0)
                point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(planner, tempList, [], 15.0, 15.0, 0.0, 160.0, l_error = localization_error, l_error_gauss = temp_covariance)
                lidar_returned[0] = [planner.localizationPositionX + localization_error[0], planner.localizationPositionY + localization_error[1],
                                    planner.theta, planner.velocity, temp_covariance.covariance.tolist()]
                if sim_values["simulate_error"]:
                    cam_returned[0] = camera_error_array
                    cam_returned[1] = fetch_time()
                    planner.localizationError = localization_error_gaussian
                    if sim_values["real_lidar"]:
                        # TODO: check this seems wrong
                        lidar_returned[1], lidar_returned[2] = lidarRecognition.processLidarFrame(point_cloud_error, fetch_time(),
                            lidar_returned[0][0], lidar_returned[0][1], lidar_returned[0][2], planner.lidarSensor)
                        planner.rawLidarDetections = point_cloud_error
                    else:
                        lidar_returned[1] = lidar_detected_error
                else:
                    cam_returned[0] = camera_array
                    cam_returned[1] = fetch_time()
                    lidar_returned[1], lidar_returned[2] = lidarRecognition.processLidarFrame(point_cloud, fetch_time(),
                        lidar_returned[0][0], lidar_returned[0][1], lidar_returned[0][2], planner.lidarSensor)
                    planner.rawLidarDetections = point_cloud
                planner.groundTruth = camera_array

                # Raw LIDAR for debug
                planner.lidarPoints = point_cloud

                lidar_recieved = True
                camera_recieved = True
            else:
                # This is not simulation so we need to get the streams from the LIDAR and camera
                lidar_recieved = False
                camera_recieved = False

                fallthrough = fetch_time() + fallthrough_delay
                while(fetch_time() < fallthrough and not (lidar_recieved and camera_recieved)):
                    # Get the lidar
                    if not lidar_out_queue.empty():
                        lidar_returned = lidar_out_queue.get()
                        #print( " Got LIDAR " )
                        lidar_recieved = True

                    # Get the camera
                    if not cam_out_queue.empty():
                        cam_returned = cam_out_queue.get()
                        #print( " Got camera " )
                        camera_recieved = True
            
            if lidar_recieved and camera_recieved:
                localization = lidar_returned[0]
                lidarcoordinates = lidar_returned[1]
                lidartimestamp = lidar_returned[2]
                camcoordinates = cam_returned[0]
                camtimestamp = cam_returned[1]
                # TODO: check the timestamps are close

                # Update the steering here while we wait for sensor fusion results and the reply from the RSU about the plan
                if not config.simulation:
                    planner.update_localization(True, [localization[0], localization[1], localization[2]])
                # planner.pure_pursuit_control()

                # # Now update our current PID with respect to other vehicles
                # planner.check_positions_of_other_vehicles_adjust_velocity(last_response)
                # # We can't update the PID controls until after all positions are known
                # planner.update_pid()

                # Finally, issue the commands to the motors
                steering_ppm, motor_pid = planner.return_command_package()
                if not config.simulation:
                    egoVehicle.setControlMotors(steering_ppm, motor_pid)

                # Fusion
                results = []
                fusion_start = fetch_time()
                fusion.processDetectionFrame(local_fusion.CAMERA, lidartimestamp, lidarcoordinates, .25, 1)
                fusion.processDetectionFrame(local_fusion.LIDAR, camtimestamp, camcoordinates, .25, 1)
                fusion_result = fusion.fuseDetectionFrame(1, planner)

                # Message the RSU, for now we must do this before our control loop
                # as the RSU has the traffic light state information
                objectPackage = {
                    "localization_t": lidartimestamp,
                    "localization": localization,
                    "lidar_t": lidartimestamp,
                    "lidar_obj": [],#lidarcoordinates,
                    "cam_t": camtimestamp,
                    "cam_obj": [],#camcoordinates,
                    "fused_t": fusion_start,
                    "fused_obj": fusion_result
                }

                if config.simulation:
                    response_message = rsu_sim_check.checkin(vehicle_id, localization[0], localization[1], 0.0, 0.0, 0.0, localization[2],
                            planner.steeringAcceleration, planner.motorAcceleration, planner.targetIndexX + planner.positionX_offset, planner.targetIndexY + planner.positionY_offset, objectPackage)

                    # Check if our result is valid
                    if response_message == None:
                        response["error"] = 1
                        print("Error: RSU response not recieved in time, stopping")
                        if fails < 5:
                            fails += 1
                        else:
                            print("Attempting to re-register with RSU")
                            # We have failed a lot lets try to re-register but use our known location
                            response_message = rsu_sim_check.register(vehicle_id, localization[0], localization[1], 0.0, 0.0, 0.0, localization[2])
                    else:
                        response["v_t"] = response_message["v_t"]
                        response["tfl_state"] = response_message["tfl_state"]
                        response["veh_locations"] = response_message["veh_locations"]
                        response["error"] = 0
                        fails = 0
                else:
                    comm_q.put(
                        [localization[0], localization[1], 0.0, 0.0, 0.0, localization[2],
                            planner.steeringAcceleration, planner.motorAcceleration, planner.targetIndexX, planner.targetIndexY, objectPackage])

                    # This should not take long but we will delay just a bit
                    time.sleep(.01)

                if response["error"] != 0:
                    # Cut the engine to make sure that we don't hit anything since the central controller is down
                    if not config.simulation:
                        egoVehicle.emergencyStop()
                    # Log this to a file
                    # with open("timing.txt", 'a') as file1:
                    #     file1.write(None, start)
                    #     index += 1
                else:
                    # Update our various pieces
                    planner.targetVelocityGeneral = float(response["v_t"])
                    planner.recieve_coordinate_group_commands(response["tfl_state"])
                    planner.pure_pursuit_control()

                    # Now update our current PID with respect to other vehicles
                    planner.check_positions_of_other_vehicles_adjust_velocity(response["veh_locations"])
                    if debug: print( " Vehicle ", vehicle_id, " target velocity ", planner.motorAcceleration, planner.targetVelocityGeneral, planner.targetVelocity, planner.targetFollowDistance, planner.followDistance )
                    last_response = response["veh_locations"]
                    # We can't update the PID controls until after all positions are known
                    planner.update_pid()
                    if debug: print( " Vehicle ", vehicle_id, " ppm, pwm ", steering_ppm, motor_pid )

                    # Finally, issue the commands to the motors
                    steering_ppm, motor_pid = planner.return_command_package()
                    if not config.simulation:
                        egoVehicle.setControlMotors(steering_ppm, motor_pid)

                    # with open("timing.txt", 'a') as file1:
                    #     file1.write(lidarDevice.localizationIdx, fetch_time())
                    #     index += 1
                print(" Time taken: ", fetch_time() - lidartimestamp, fetch_time() - camtimestamp, fetch_time())
            else:
                print(" Error, no camera/lidar frame returned ", lidar_recieved, camera_recieved)
                # Cut the engine to make sure that we don't hit anything since we are blind
                if not config.simulation:
                    egoVehicle.emergencyStop()

            last_next_time = next_time
            while next_time <= fetch_time():
                next_time = last_next_time + interval
                last_next_time = next_time

            #time.sleep(last_next_time - .01)

    if not config.simulation:
        egoVehicle.emergencyStop()
