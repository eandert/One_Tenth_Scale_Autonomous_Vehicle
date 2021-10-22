import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Manager

import camera_recognition
import lidar_recognition
import planning_control
import communication
import motors
import local_fusion

pipeFromC = "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc"
pipeToC = "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc"

vehicle_id = 1
dump_to_file = False


def sourceImagesThread(out_queue, settings, camSpecs, start_time, interval):
    # Init the camera class
    cameraRecognition = camera_recognition.Camera(settings, camSpecs)

    # Sleep until test start time
    wait_until_start = start_time - time.time() - .01
    if wait_until_start > 0:
        time.sleep(start_time)

    # Now wait and ask for input
    while 1:
        if (round(time.time(),3) % interval) == 0.000:
            now = time.time()

            # Take the camera frame and process
            camcoordinates, camtimestamp_start, camtimestamp_end = cameraRecognition.takeCameraFrame()

            # Prep value to be sent to the part of the program
            out_queue.put([camcoordinates, camtimestamp_start, now])

            # Sleep to reduce busy wait
            wait_until_next = round(time.time(),3) % interval - .001
            if wait_until_next > 0:
                time.sleep(wait_until_next)


def sourceLIDARThread(out_queue, pipeFromC, pipeToC, lidarSensor, start_time, interval):
    # Start the connection with the LIDAR through pipes
    lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC)
    target = interval

    # Init the LIDAR processing class
    lidarRecognition = lidar_recognition.LIDAR(time.time())

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
    wait_until_start = start_time - time.time() - .01
    if wait_until_start > 0:
        time.sleep(start_time)

    # Now wait for input
    while 1:
        if (round(time.time(),3) % target) == 0.000:
            # Take the LIDAR frame and process
            #print ( "LIDAR start " + str(time.time()))
            start, end = lidarDevice.getFromC()
            #print ( "LIDAR got " + str(time.time()))

            lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(lidarDevice.parseFromC(),
                                                                                  start,
                                                                                  lidarDevice.localizationX,
                                                                                  lidarDevice.localizationY, 
                                                                                  lidarDevice.localizationYaw,
                                                                                  lidarSensor)
            localization = [lidarDevice.localizationX, lidarDevice.localizationY, lidarDevice.localizationYaw, index, start]
            #print ( "LIDAR detect " + str(time.time()))

            # Send the localization and coordinate results to the fusion function
            out_queue.put([localization, lidarcoordinates, lidartimestamp])

            # Log this to a file
            index += 1

            # Sleep to reduce busy wait
            wait_until_next = round(time.time(), 3) % interval - .001
            if wait_until_next > 0:
                time.sleep(wait_until_next)


def processCommunicationsThread(comm_q, v_id, init, response):
    vehicle_id = v_id
    # Start the connection with the RSU (Road Side Unit) server through sockets
    rsu = communication.connectServer()
    init_returned = rsu.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    init["t_x"] = init_returned["t_x"]
    init["t_y"] = init_returned["t_y"]
    init["t_yaw"] = init_returned["t_yaw"]
    init["route_x"] = init_returned["route_x"]
    init["route_y"] = init_returned["route_y"]
    init["route_TFL"] = init_returned["route_TFL"]
    init["start_time"] = init_returned["start_time"]
    init["interval"] = init_returned["interval"]

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


debug = False

index = 0

# The first thing we should always do is initialize the control module
# This is important to make sure a rogue signal doesn't drive us away
egoVehicle = motors.Motors()

# Start the connection with the LIDAR through pipes
lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC)

# Init the camera
# Setup our various settings
settings = camera_recognition.Settings()
settings.darknetPath = '../darknet/'
camSpecs = camera_recognition.CameraSpecifications()
camSpecs.cameraHeight = .2
camSpecs.cameraAdjustmentAngle = 0.0

# Set up the timing
start_time = time.time() + 2
interval = 0.125
fallthrough_delay = 0.100

# Initialize the planner
planner = planning_control.Planner()

# Spawn the camera processing thread
cam_out_queue = Queue()
cameraThread = Process(target=sourceImagesThread, args=(cam_out_queue, settings, camSpecs, start_time, interval))
cameraThread.start()

# Wait to make sure that we have started YOLO
lidar_out_queue = Queue()
cameraThread = Process(target=sourceImagesThread, args=(lidar_out_queue, pipeFromC, pipeToC, planner.lidarSensor, start_time, interval))
cameraThread.start()

# Spawn the communication thread
manager = Manager()
init = manager.dict()
response = manager.dict()
response["error"] = 1
comm_q = Queue()
commThread = Process(target=processCommunicationsThread, args=(comm_q, vehicle_id, init, response))
commThread.start()

# Now wait for the response
while 'route_x' not in init:
    time.sleep(.01)

# Now that we have chatted with the RSU server, we should know where we are going
planner.initialVehicleAtPosition(init["t_x"], init["t_y"], init["t_yaw"], init["route_x"], init["route_y"],
                                 init["route_TFL"], vehicle_id, False)

# Start the sensor fusion pipeline
fusion = local_fusion.FUSION(0, vehicle_id)

while True:
    # Now get the result from the LIDAR
    lidar_recieved = False
    camera_recieved = False

    # Get the camera
    try:
        lidar_returned = lidar_out_queue.get(timeout = fallthrough_delay - .01)
        if len(lidar_returned) == 3:
            lidar_recieved = True
    except TimeoutError:
        print("LIDAR not recieved")

    try:
        cam_returned = cam_out_queue.get(timeout = .01)
        if len(cam_returned) == 3:
            camera_recieved = True
    except TimeoutError:
        print("Cam not recieved")
    
    if lidar_recieved and camera_recieved:
        localization = lidar_returned[0]
        lidarcoordinates = lidar_returned[1]
        lidartimestamp = lidar_returned[2]
        camcoordinates = cam_returned[0]
        camtimestamp = cam_returned[1]
        # TODO: check the timestamps are close

        # Fusion
        fusion.processDetectionFrame(local_fusion.CAMERA, lidartimestamp, lidarcoordinates, .25, 1)
        fusion.processDetectionFrame(local_fusion.LIDAR, camtimestamp, camcoordinates, .25, 1)
        results = fusion.fuseDetectionFrame(1, planner)

        # Message the RSU, for now we must do this before our control loop
        # as the RSU has the traffic light state information
        objectPackage = {
            "lidar_t": lidartimestamp,
            "lidar_obj": lidarcoordinates,
            "cam_t": camtimestamp,
            "cam_obj": camcoordinates,
            "fused_t": time.time(),
            "fused_obj": results
        }

        comm_q.put(
            [localization[0], localization[1], 0.0, 0.0, 0.0, localization[2],
                objectPackage])

        if response["error"] != 0:
            # Cut the engine to make sure that we don't hit anything since the central controller is down
            egoVehicle.emergencyStop()
            # Log this to a file
            # with open("timing.txt", 'a') as file1:
            #     file1.write(None, start)
            #     index += 1
        else:
            # Update our various pieces
            planner.targetVelocityGeneral = float(response["v_t"])
            planner.update_localization([localization[0], localization[1], localization[2]])
            planner.recieve_coordinate_group_commands(response["tfl_state"])
            planner.pure_pursuit_control()

            # Now update our current PID with respect to other vehicles
            planner.check_positions_of_other_vehicles_adjust_velocity(response["veh_locations"])
            # We can't update the PID controls until after all positions are known
            planner.update_pid()

            # Finally, issue the commands to the motors
            steering_ppm, motor_pid = planner.return_command_package()
            egoVehicle.setControlMotors(steering_ppm, motor_pid)

            # with open("timing.txt", 'a') as file1:
            #     file1.write(lidarDevice.localizationIdx, time.time())
            #     index += 1
            # if debug:
            #     plt.cla()
            #     # Create plot for lidar and camera points
            #     for i, data in enumerate(lidarcoordinates[:]):
            #         plt.plot(data[1], data[2], 'go')
            #         plt.annotate(data[0], (data[1], data[2]))
            #     for i, data in enumerate(camcoordinates):
            #         plt.plot(data[1], data[2], 'ro')
            #         plt.annotate(data[0], (data[1], data[2]))
            #     plt.title("Sensors")
            #     plt.pause(0.001)
        print(" Time taken: ", time.time() - lidartimestamp, time.time())
    else:
        print(" Error, no camera/lidar frame returned ")
        # Cut the engine to make sure that we don't hit anything since we are blind
        egoVehicle.emergencyStop()

egoVehicle.emergencyStop()

client.close()
server.close()

plt.show()
