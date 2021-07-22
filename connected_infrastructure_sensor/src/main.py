import time
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue

import camera_recognition
import communication

pipeFromC= "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc"
pipeToC= "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc"

vehicle_id = 0

debug = False

# Init the camera
# Setup our various settings
settings = camera_recognition.Settings()
settings.darknetPath = '../darknet/'
camSpecs = camera_recognition.CameraSpecifications()
camSpecs.cameraHeight = .2
camSpecs.cameraAdjustmentAngle = 0.0
frameID = 1

# Create the camerqa class
cameraRecognition = camera_recognition.Camera(settings, camSpecs, False)

# Wait to make sure that we have started YOLO
time.sleep(10)

# Start the connection with the RSU (Road Side Unit) server through sockets
rsu = communication.connectServer()
response = rsu.register(vehicle_id, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

# Save the specs we got back from the RSU in case we need them someday
specs = [response["t_x"], response["t_y"], response["t_yaw"], response["route_x"], response["route_y"], response["route_TFL"]]

# Fails keeps track of how many tries to connect with 
fails = 0

while True:
    # We do this as fast as possible
    # Process the camera frame
    camcoordinates, camtimestamp = cameraRecognition.takeCameraFrame(settings,camSpecs)

    #print ( "detections: ", camcoordinates )
    
    # Message the RSU, for now we must do this before our control loop
    # as the RSU has the traffic light state information
    objectPackage = {
        "lidar_t":"null",
        "lidar_obj":"null",
        "cam_t":camtimestamp,
        "cam_obj":camcoordinates,
        "fused_t":"null",
        "fused_obj":"null",
    }
    response_checkin = rsu.checkin(vehicle_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2], objectPackage)
    #tflState = rsu.messageLocation(vehicle_id, lidartimestamp, lidarDevice.localization, vehicleObservations)

    # Check if our result is valid
    if response_checkin == None:
        # The central controller may be down
        print ( "Error: RSU response not recieved in time, stopping" )
        if fails < 20:
            fails += 1
        else:
            print ( "Attempting to re-register with RSU" )
            # We have failed a lot lets try to re-register but use our known location
            response = rsu.register(vehicle_id, specs[0], specs[1], 0.0, 0.0, 0.0, specs[2])
    else:
        # Nothing to update since we dont move!
        fails = 0

        if debug:
            plt.cla()
            # Create plot for camera points
            for i, data in enumerate(camcoordinates):
                plt.plot(data[1], data[2], 'ro')
                plt.annotate(data[0], (data[1], data[2]))
            plt.title("Sensors")
            plt.pause(0.001)

    print ( " Time taken: " , time.time() - camtimestamp, time.time() )

client.close()
server.close()

plt.show()
