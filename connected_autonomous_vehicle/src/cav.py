from sre_constants import JUMP
import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Manager
import sys
import math

from connected_autonomous_vehicle.src import planning_control, communication
from shared_library import local_fusion, sensor, lidar_recognition

#import local_fusion

pipeFromC = "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc"
pipeToC = "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc"

# This function is for controlling the time function in case of simulation
def fetch_time(simulation_time, global_time = 1.0):
    if simulation_time:
        return global_time
    else:
        return time.time()

def update_time_from_rsu_sim(vehicle_id, debug, rsu_sim_check = None):
    while(True):
        new_time = rsu_sim_check.getSimTime()["time"]
        if new_time != -99 and new_time != None:
            if debug: print( " Vehicle ", vehicle_id, " got sim time from server ", new_time)
            return new_time
        elif new_time == -99:
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
                #camcoordinates, camtimestamp_start, camtimestamp_end = cameraRecognition.takeCameraFrame()
                camtimestamp_start = time.simte()
            #print ( "CAM got " + str(fetch_time(simulation_time, global_time)))

            # Prep value to be sent to the part of the program
	    # Clear the queue of old data
            while not out_queue.empty():
                out_queue.get()
            out_queue.put([camcoordinates, camtimestamp_start, now])

            # Sleep to reduce busy wait
            #wait_until_next = round(fetch_time(simulation_time, global_time),3) % interval - .001
            #if wait_until_next > 0:
            #    time.sleep(wait_until_next)

            #with open("cam_output.txt", 'a') as file1:
            #    file1.write(str(camtimestamp_start) + ',' + str(frame) + '\n')

            # New target
            target = target + interval
        time.sleep(.001)


def sourceLIDARThread(out_queue, pipeFromC, pipeToC, lidarSensor, simulation_time, data_collect_mode, start_time, interval):
    global bounding_box
    # Start the connection with the LIDAR through pipes
    lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC)
    target = interval

    # Init the LIDAR processing class
    lidarRecognition = lidar_recognition.LIDAR(fetch_time(simulation_time))

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
    wait_until_start = start_time - fetch_time(simulation_time) - .01
    if wait_until_start > 0:
        time.sleep(wait_until_start)

    target = start_time

    # Now wait for input
    while 1:
        if fetch_time(simulation_time) >= target:
            # Take the LIDAR frame and process
            #print ( "LIDAR start " + str(fetch_time(simulation_time, global_time)))
            start, end = lidarDevice.getFromC()
            #print ( "LIDAR got " + str(fetch_time(simulation_time, global_time)))
            if data_collect_mode:
                # Only collecting data, skip processing
                raw_lidar = lidarDevice.parseFromC()
                lidarcoordinates = []
                lidartimestamp = end
            else:
                lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(lidarDevice.parseFromC(),
                                                                                    start,
                                                                                    lidarDevice.localizationX,
                                                                                    lidarDevice.localizationY, 
                                                                                    lidarDevice.localizationYaw,
                                                                                    lidarSensor)
            localization = [lidarDevice.localizationX, lidarDevice.localizationY, lidarDevice.localizationYaw, index, start]
            #print ( "LIDAR detect " + str(fetch_time(simulation_time, global_time)))

            # Send the localization and coordinate results to the fusion function
	        # Clear the queue of old data
            while not out_queue.empty():
                out_queue.get()
            out_queue.put([localization, lidarcoordinates, lidartimestamp])

            # Log this to a file
            index += 1

            #with open("lidar_output.txt", 'a') as file1:
            #    file1.write(str(lidartimestamp) + '\n' + str(localization) + '\n' + str(raw_lidar) + '\n')

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
            x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, intersection_id, objectPackage = got

            response_message = rsu.checkin(vehicle_id, x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, intersection_id, objectPackage)

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
                response["v_t"] = response_message["v_t"]
                response["tfl_state"] = response_message["tfl_state"]
                response["veh_locations"] = response_message["veh_locations"]
                response["error"] = 0
                fails = 0

def cav(config, vid, test_idx):
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
    if config.unit_test:
        local_fusion_mode = config.unit_test_config[test_idx][0]
    else:
        local_fusion_mode = 0

    print( " CAV ", vid, " begin.")

    if not config.simulation:
    	# Do our imports within this function so we dont disturb the simulation
        from connected_autonomous_vehicle.src import motors
        from shared_library import camera_recognition
        egoVehicle = motors.Motors()

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

    # Initialize the planner
    planner = planning_control.Planner()

    if not config.simulation:
        # Spawn the camera processing thread
        cam_out_queue = Queue()
        cameraThread = Process(target=sourceImagesThread, args=(cam_out_queue, settings, camSpecs, simulation_time, data_collect_mode, start_time, interval))
        cameraThread.start()

        # Spawn the lidar processign thread
        lidar_out_queue = Queue()
        cameraThread = Process(target=sourceLIDARThread, args=(lidar_out_queue, pipeFromC, pipeToC, planner.lidarSensor, simulation_time, data_collect_mode, start_time, interval))
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
        global_time = 1.0
        init = {}
        response = {}
        lidarRecognition = lidar_recognition.LIDAR(0.0)

        # Manually do what the thread would normally do
        if debug: print( " Vehicle ", vehicle_id, " connecting to RSU... ", config.rsu_ip)
        # Start the connection with the RSU (Road Side Unit) server through sockets
        rsu_sim_check = communication.connectServer(config.rsu_ip)
        if debug: print( " Vehicle ", vehicle_id, " connected ", config.rsu_ip)
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

    if debug: print( " Vehicle ", vehicle_id, " init pos ", init["t_x"], init["t_y"], init["t_yaw"])

    # Now that we have chatted with the RSU server, we should know where we are going
    planner.initialVehicleAtPosition(init["t_x"], init["t_y"], init["t_yaw"], init["route_x"], init["route_y"],
                                    init["route_TFL"], vehicle_id, config.simulation)
    if debug: print( " Vehicle ", vehicle_id, " planner initialized " , planner.localizationPositionX, planner.localizationPositionY, planner.theta, planner.positionX_offset, planner.positionY_offset, planner.theta_offset)

    # Do a quick check of the coordinates to make a bounding box filter so that we can take out uneccessary points
    min_x = 99.0
    max_x = -99.0
    min_y = 99.0
    max_y = -99.0
    for x in init["route_x"]:
        if x > max_x:
            max_x = x
        if x < min_x:
            min_x = x
    for y in init["route_y"]:
        if y > max_y:
            max_y = y
        if y < min_y:
            min_y = y
    buffer = .25
    bounding_box = [[min_x - buffer, max_x + buffer],[min_y - buffer, max_y + buffer]]
    if debug: print( " Vehicle ", vehicle_id, " bounding box params ", bounding_box )

    # If this is a simulation we need a second communication class to get some ideal positions
    if config.simulation:
        global_time = update_time_from_rsu_sim(vehicle_id, debug, rsu_sim_check)
        print("global_time", global_time)

    # Start the sensor fusion pipeline
    fusion = local_fusion.FUSION(local_fusion_mode, vehicle_id)
    if debug: print( " Vehicle ", vehicle_id, " started fusion node" )

    # Sleep until test start time
    wait_until_start = start_time - fetch_time(simulation_time, global_time) - .01
    if wait_until_start > 0 and not config.simulation:
        time.sleep(wait_until_start)

    next_time = start_time + interval_offset
    if debug: print( " Vehicle ", vehicle_id, " start time is ", next_time)

    last_lidar_time = fetch_time(simulation_time, global_time)

    while True:
        if config.simulation:
            global_time = update_time_from_rsu_sim(vehicle_id, debug, rsu_sim_check)
            if global_time == -99:
                exit(0)
        if fetch_time(simulation_time, global_time) >= next_time:
            if config.simulation:
                # Special simulation setup where we do not use the source threads
                # Update the localization first because we use it here
                planner.updatePosition(interval)
                planner.update_localization(use_localization = False)
                if debug: print( " Vehicle ", vehicle_id, " posiiton and localization updated" )
                
                # Send the lolcailzation values to the RSU
                rsu_sim_check.sendSimPosition(vehicle_id, planner.localizationPositionX, planner.localizationPositionY, 0.0, 0.0, 0.0, planner.theta, planner.velocity)
                if debug: print( " Vehicle ", vehicle_id, " sent simulated position to RSU" )

                # Recieve positions of other vehicles from RSU so we can fake the sensor values
                sim_values = rsu_sim_check.getSimPositions(vehicle_id)
                while(sim_values['step_sim_vehicle'] == False):
                    time.sleep(.01)
                    sim_values = rsu_sim_check.getSimPositions(vehicle_id)
                    if debug: print( " Vehicle ", vehicle_id, " requesting simulation positions" )
                
                vehicle_object_positions = sim_values['veh_locations']
                vehicle_object_positions2 = sim_values['veh_locations']
                if vehicle_id == 0 and global_time > 100.0:
                    for each in vehicle_object_positions:
                        if each[0] != planner.localizationPositionX and each[1] != planner.localizationPositionX:
                            #print(" rotating: ", each)
                            ox = planner.localizationPositionX
                            oy = planner.localizationPositionX
                            angle = math.radians(test_idx)
                            px = each[0]
                            py = each[1]
                            qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
                            qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
                            each[0] = qx
                            each[1] = qy
                            #print(each)

                cam_returned2, lidar_returned = sensor.simulate_sensors(planner, lidarRecognition, fetch_time(simulation_time, global_time), sim_values, vehicle_object_positions)
                cam_returned, lidar_returned2 = sensor.simulate_sensors(planner, lidarRecognition, fetch_time(simulation_time, global_time), sim_values, vehicle_object_positions2)

                lidar_recieved = True
                camera_recieved = True
            else:
                # This is not simulation so we need to get the streams from the LIDAR and camera
                lidar_recieved = False
                camera_recieved = False

                fallthrough = fetch_time(simulation_time, global_time) + fallthrough_delay
                while(fetch_time(simulation_time, global_time) < fallthrough and not (lidar_recieved and camera_recieved)):
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
                lidarraw = planner.rawLidarDetections
                camcoordinates = cam_returned[0]
                camtimestamp = cam_returned[1]
                # TODO: check the timestamps are close

                # Update the steering here while we wait for sensor fusion results and the reply from the RSU about the plan
                if not config.simulation:
                    planner.update_localization(True, [localization[0], localization[1], localization[2]])
                    localization[0] = planner.localizationPositionX
                    localization[1] = planner.localizationPositionY
                    localization[2] = planner.theta
                planner.pure_pursuit_control()

                # # Now update our current PID with respect to other vehicles
                planner.check_positions_of_other_vehicles_adjust_velocity(last_response)
                # # We can't update the PID controls until after all positions are known
                planner.update_pid()

                # Finally, issue the commands to the motors
                steering_ppm, motor_pid = planner.return_command_package()
                if not config.simulation:
                    egoVehicle.setControlMotors(steering_ppm, motor_pid)

                # Fusion
                fusion_result = []
                fusion_start = fetch_time(simulation_time, global_time)
                if not data_collect_mode:
                    fusion.processDetectionFrame(local_fusion.CAMERA, camtimestamp, camcoordinates, .25, 1)
                    fusion.processDetectionFrame(local_fusion.LIDAR, lidartimestamp, lidarcoordinates, .25, 1)
                    fusion_result = fusion.fuseDetectionFrame(1, planner)
                else:
                    fusion_result = []

                # Message the RSU, for now we must do this before our control loop
                # as the RSU has the traffic light state information
                if not data_collect_mode:
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
                    }
                else:
                    objectPackage = {
                        "localization_t": lidartimestamp,
                        "localization": localization,
                        "lidar_t": lidartimestamp,
                        "lidar_detection_raw": [],
                        "lidar_obj": [],
                        "cam_t": camtimestamp,
                        "cam_obj": [],
                        "fused_t": lidartimestamp,
                        "fused_obj": []
                    }

                if config.simulation:
                    response_message = rsu_sim_check.checkin(vehicle_id, planner.localizationPositionX, planner.localizationPositionY, 0.0, 0.0, 0.0, planner.theta,
                            planner.steeringAcceleration, planner.motorAcceleration, planner.targetIndexX, planner.targetIndexY, planner.vCoordinates[planner.tind], objectPackage)

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
                        response["intersection_mode"] = response_message["intersection_mode"]
                        response["av_intersection_permission"] = response_message["av_intersection_permission"]
                        response["error"] = 0
                        fails = 0
                else:
                    comm_q.put(
                        [planner.rearAxlePositionX, planner.rearAxlePositionY, 0.0, 0.0, 0.0, planner.theta,
                            planner.steeringAcceleration, planner.motorAcceleration, planner.targetIndexX, planner.targetIndexY, planner.vCoordinates[planner.ind], objectPackage])

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
                    planner.tfl_mode = int(response["intersection_mode"])
                    planner.av_intersection_permission = int(response["av_intersection_permission"])
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
                    #     file1.write(lidarDevice.localizationIdx, fetch_time(simulation_time, global_time))
                    #     index += 1
                    last_lidar_time = fetch_time(simulation_time, global_time)
                if not config.simulation:
                    if debug: print(" Time taken: ", fetch_time(simulation_time, global_time) - lidartimestamp, fetch_time(simulation_time, global_time) - camtimestamp, fetch_time(simulation_time, global_time))
            else:
                if debug: print(" Error, no camera/lidar frame returned ", lidar_recieved, camera_recieved)
                # Cut the engine to make sure that we don't hit anything since we are blind
                if not config.simulation and (fetch_time(simulation_time, global_time) - last_lidar_time) >= .250 :
                    egoVehicle.emergencyStop()

            last_next_time = next_time
            while next_time <= fetch_time(simulation_time, global_time):
                next_time = last_next_time + interval
                last_next_time = next_time

            #time.sleep(last_next_time - .01)
        if config.simulation:
            if debug: print(" Vehicle ", vehicle_id, fetch_time(simulation_time, global_time))

        time.sleep(.001)

    if not config.simulation:
        egoVehicle.emergencyStop()
