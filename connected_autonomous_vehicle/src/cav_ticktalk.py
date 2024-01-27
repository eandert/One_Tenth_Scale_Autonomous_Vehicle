import Tag
import WaitingMatching
import Token
import secrets
import socket
import json
import motors
import communication
import planning_control
import lidar_recognition
import camera_recognition
import ProcessManager
import Network
import sys
import os
import time
import math

import pickle
import multiprocessing
import queue
from intervaltree import Interval

# from competition import Token, WaitingMatching, Tag
# from competition import Network
# from competition import ProcessManager

sys.path.insert(0, os.path.abspath('../ticktalkpython/competition/'))


def whoAmI():
    # Finds our own IP address automagically, doesn't work on mac!
    return [l for l in (
        [ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [
            [(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in
             [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]


def sourceImagesThread(settings, camSpecs, network_send_queue, start_time, interval):
    # Init the camera class
    cameraRecognition = camera_recognition.Camera(settings, camSpecs)
    target = interval

    # Now wait for input
    while 1:
        if (round(time.time(), 3) % target) == 0.000:
            now = time.time()

            # Take the camera frame and process
            camcoordinates, camtimestamp_start, camtimestamp_end = cameraRecognition.takeCameraFrame()

            # Prep value to be sent to the part of the program
            output_tag = Tag.Tag('local_fusion', 'camera',
                                 camtimestamp_start, camtimestamp_end)
            output_token = Token.Token(output_tag, camcoordinates)
            recipient_name = 'CAV1'
            # send to named device, e.g. CAV1, RSU
            network_send_queue.put((output_token, recipient_name))


def sourceLIDARThread(pipeFromC, pipeToC, network_send_queue, start_time, interval):
    # Start the connection with the LIDAR through pipes
    lidarDevice = communication.connectLIDAR(pipeFromC, pipeToC)
    target = interval

    # Init the LIDAR processing class
    lidarRecognition = lidar_recognition.LIDARClusteringtime.time())

    # Wait for 1 second before we start everything
    time.sleep(2)

    start, end = lidarDevice.getFromC()
    lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(
        lidarDevice.parseFromC(), start)

    index = 0

    # Now wait for input
    while 1:
        if (round(time.time(), 3) % target) == 0.000:
            # Take the LIDAR frame and process
            # print ( "LIDAR start " + str(time.time()))
            start, end = lidarDevice.getFromC()
            # print ( "LIDAR got " + str(time.time()))

            lidarcoordinates, lidartimestamp = lidarRecognition.processLidarFrame(
                lidarDevice.parseFromC(), start)
            localization = [lidarDevice.localizationX, lidarDevice.localizationY,
                            lidarDevice.localizationYaw, index, start]
            # print ( "LIDAR detect " + str(time.time()))

            # Prep value to be sent to the part of the program
            output_tag = Tag.Tag('local_fusion', 'localization', start, end)
            output_token = Token.Token(output_tag, localization)
            recipient_name = 'CAV1'
            # send to named device, e.g. CAV1, RSU
            network_send_queue.put((output_token, recipient_name))

            # Prep value to be sent to the part of the program
            output_tag = Tag.Tag('local_fusion', 'lidar_obs', start, end)
            output_token = Token.Token(output_tag, lidarcoordinates)
            recipient_name = 'CAV1'
            # send to named device, e.g. CAV1, RSU
            network_send_queue.put((output_token, recipient_name))

            # Prep value to be sent to the part of the program
            output_tag = Tag.Tag('actuate', 'response', start, end)
            output_token = Token.Token(output_tag, [localization, None])
            recipient_name = 'CAV1'
            # send to named device, e.g. CAV1, RSU
            network_send_queue.put((output_token, recipient_name))

            # Log this to a file
            index += 1


class Fusion():
    # minimal wrapper; if you need state, then the function needs to be a method of this function-process class such that self is available when the function runs.
    def __init__(self, network_send_queue, vehicle_id, r_queue):
        self.input_queue = multiprocessing.Queue()
        self.function = self.local_fusion

        self.network_send_queue = network_send_queue

        self.key = secrets.token_urlsafe(32)
        self.vehicle_id = vehicle_id
        self.r_queue = r_queue
        self.register = True

        self.proc = multiprocessing.Process(target=self.listener)
        self.proc.start()

    def listener(self):
        while True:
            try:
                input_args = self.input_queue.get(block=True, timeout=1)
                args = input_args[0]
                kwargs = input_args[1]
                self.local_fusion(*args, **kwargs)
            except queue.Empty:
                continue
            except:
                raise

    def local_fusion(self, camera, localization, lidar_obs, time_overlap=Interval(0, .125)):
        '''
        args supplied by the synchronized WM function
        '''
        if camera == None and lidar == None:
            print(" Missed both! ")
            fused = []
        elif camera == None:
            print(" Missed Camera! ")
            fused = lidar_obs
        elif lidar_obs == None:
            print(" Missed LIDAR! ")
            fused = camera
        else:
            # print(" Got them both! ")
            fused = camera + lidar_obs

        # print('Local fusion function running!')
        # print(time_overlap)
        try:
            rec = self.r_queue.get(block=False, timeout=.001)
            if rec != None:
                print("Got rec ----------------------", rec)
                self.register = False
        except:
            pass

        if self.register:
            # data to be sent to api
            packet = {'key': self.key,
                      'id': self.vehicle_id,
                      'type': 0,
                      'register': True,
                      'timestamp': time.time(),
                      'x': localization[0],
                      'y': localization[1],
                      'z': 0.0,
                      'roll': 0.0,
                      'pitch': 0.0,
                      'yaw': localization[2],
                      'detections': fused}
        else:
            packet = {'key': self.key,
                      'id': self.vehicle_id,
                      'type': 0,
                      'register': False,
                      'timestamp': time.time(),
                      'x': localization[0],
                      'y': localization[1],
                      'z': 0.0,
                      'roll': 0.0,
                      'pitch': 0.0,
                      'yaw': localization[2],
                      'detections': fused}

        # Prep value to be sent to the part of the program
        output_tag = Tag.Tag('global_fusion', 'cav1',
                             time_overlap.begin, time_overlap.end)
        output_token = Token.Token(output_tag, packet)
        recipient_name = 'RSU'
        # send to named device, e.g. CAV1, RSU
        self.network_send_queue.put((output_token, recipient_name))


class Actuation():
    # minimal wrapper; if you need state, then the function needs to be a method of this function-process class such that self is available when the function runs.
    def __init__(self, network_send_queue, vehicle_id, r_queue):
        self.input_queue = multiprocessing.Queue()
        self.function = self.actuate

        self.network_send_queue = network_send_queue

        # Init everything
        # The first thing we should always do is initialize the control module
        # This is important to make sure a rogue signal doesn't drive us away
        self.egoVehicle = motors.Motors()
        self.vehicle_id = vehicle_id
        self.planner = None
        self.r_queue = r_queue

        self.localization = []

        self.proc = multiprocessing.Process(target=self.listener)
        self.proc.start()

    def listener(self):
        while True:
            try:
                input_args = self.input_queue.get(block=True, timeout=1)
                args = input_args[0]
                kwargs = input_args[1]
                self.actuate(*args, **kwargs)
            except queue.Empty:
                continue
            except:
                raise

    def actuate(self, response, time_overlap=Interval(0, .1249)):
        '''
        args supplied by the synchronized WM function
        '''
        try:
            # print ( response )

            if response != None:
                localization = response[0]
                response_json = response[1]
            else:
                print(" Nones " + str(response))
                localization = None
                response_json = None

            if localization == None:
                self.egoVehicle.emergencyStop()
                print("Emergency stop!")
            else:
                # Check if we have been initialized
                if self.planner == None:
                    if response_json != None:
                        print(" settign planner start ")
                        self.planner = planning_control.Planner()
                        self.planner.initialVehicleAtPosition(
                            response_json["t_x"], response_json["t_y"], response_json["t_yaw"], response_json["route_x"], response_json["route_y"], response_json["route_TFL"], 1, False)
                        self.egoVehicle.emergencyStop()
                        self.r_queue.put(True)
                        print(" settign planner end ")
                    else:
                        print(" skipping planner ", str(response_json))
                        self.egoVehicle.emergencyStop()
                        print("Emergency stop!")
                elif response_json != None:
                    # Make sure we have not skipped a step and missed our localization
                    if self.localization != None:
                        # Update our various pieces
                        self.planner.target_velocity_general = float(
                            response_json["v_t"])
                        self.planner.update_localization(
                            [self.localization[0], self.localization[1], self.localization[2]])
                    else:
                        # The clocalization on the packet is used here as a fallback. Should only happen once
                        self.planner.target_velocity_general = float(
                            response_json["v_t"])
                        self.planner.update_localization(
                            [localization[0], localization[1], localization[2]])
                    self.planner.recieve_coordinate_group_commands(
                        response_json["tfl_state"])
                    self.planner.pure_pursuit_control()

                    # Now update our current PID with respect to other vehicles
                    self.planner.check_positions_of_other_vehicles_adjust_velocity(
                        response_json["veh_locations"], self.vehicle_id)

                    # We can't update the PID controls until after all positions are known
                    self.planner.update_pid()

                    # Finally, issue the commands to the motors
                    steering_ppm, motor_pid = self.planner.return_command_package()
                    self.egoVehicle.setControlMotors(steering_ppm, motor_pid)
                else:
                    # Only update our steering, use the last positions of all things as it should not matter!
                    # Update our various pieces
                    self.localization = localization
                    self.planner.update_localization(
                        [localization[0], localization[1], localization[2]])
                    self.planner.pure_pursuit_control()

                    # Finally, issue the commands to the motors
                    steering_ppm, motor_pid = self.planner.return_command_package()
                    self.egoVehicle.setControlMotors(steering_ppm, motor_pid)

                    tag_deadline = Tag.Tag(
                        'actuate', 'DEADLINE', time.time(), time.time() + 0.1000)
                    token_deadline = Token.Token(tag_deadline, None)
                    recipient_name = 'CAV1'
                    # send to named device, e.g. CAV1, RSU
                    self.network_send_queue.put(
                        (token_deadline, recipient_name))

            if localization == None:
                test = "inf"
                test2 = time.time()
            else:
                test = localization[3]
                test2 = time.time() - localization[4]
            print(str(test) + "," + str(test2) + "\n")
            # file1.write(str(test) + "," + str(test2) + "\n")
            print('Actuation achieved!')
        except Exception as e:
            self.egoVehicle.emergencyStop()
            print("    Exception in actuate! ", str(e))


def main():
    # receive to this IP + port
    tx_port = 7167  # TX on this port, RX on 7178 (port+1)
    rx_port = tx_port+1
    self_ip = whoAmI()  # This needs to be THIS device's IP on the 192.168 subnet

    vehicle_id = 1

    # turn network messages into tokens that get sent to the right function
    interface_manager = ProcessManager.TokenInterfaceManager(
        rx_ip=self_ip, rx_port=tx_port)

    with open('routing.json') as json_file:
        data = json.load(json_file)

        print(data['routing']['CAV1'])

        interface_manager.add_route(
            'CAV1', data['routing']['CAV1']+':'+str(rx_port))  # use the RX port
        interface_manager.add_route(
            'CAV2', data['routing']['CAV2']+':'+str(rx_port))  # use the RX port
        interface_manager.add_route(
            'CAM', data['routing']['CAM']+':'+str(rx_port))  # use the RX port
        interface_manager.add_route(
            'RSU', data['routing']['RSU']+':'+str(rx_port))  # use the RX port

    r_queue = multiprocessing.Queue()

    # receive network interface
    rx_network = Network.TTNetwork(
        ip=self_ip, port=rx_port, msg_receiver=interface_manager.receiver_function)

    # wrapped function that needs to be synchronized
    local_fusion_fp = Fusion(
        interface_manager.send_token_queue, vehicle_id, r_queue)
    fusion_sync = WaitingMatching.InputSynchronizedFunction(
        Fusion.local_fusion, local_fusion_fp.input_queue)

    # wrapped function that needs to be synchronized
    actuation_fp = Actuation(
        interface_manager.send_token_queue, vehicle_id, r_queue)
    actuation_sync = WaitingMatching.InputSynchronizedFunction(
        Actuation.actuate, actuation_fp.input_queue, use_deadline=True)

    # register synchronization section with process/interface manager
    interface_manager.add_sync_process(fusion_sync.function_name, fusion_sync)
    interface_manager.add_sync_process(
        actuation_sync.function_name, actuation_sync)

    # Init the camera
    # Setup our various settings
    settings = camera_recognition.Settings()
    settings.darknetPath = '../darknet/'
    camSpecs = camera_recognition.CameraSpecifications()
    camSpecs.cameraHeight = .2
    camSpecs.cameraAdjustmentAngle = 0.0

    # Lidar settings
    pipeFromC = "/home/jetson/Projects/slamware/fifo_queues/fifopipefromc"
    pipeToC = "/home/jetson/Projects/slamware/fifo_queues/fifopipetoc"

    # Set the time to start 10 seconds from now and let it rip!
    start_time = time.time() + 10
    interval = .125

    cameraThread = multiprocessing.Process(target=sourceImagesThread, args=(
        settings, camSpecs, interface_manager.send_token_queue, start_time, interval))
    cameraThread.start()
    lidarThread = multiprocessing.Process(target=sourceLIDARThread, args=(
        pipeFromC, pipeToC, interface_manager.send_token_queue, start_time, interval))
    lidarThread.start()

    # need this to make sure the main process doesn't finish and carry everything into the abyss
    fusion_sync.proc.join()
    actuation_sync.proc.join()


if __name__ == "__main__":
    main()
