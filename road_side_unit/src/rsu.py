import time
import math
import sys
import numpy as np
from sklearn.neighbors import NearestNeighbors
from threading import Lock, Thread
from queue import Queue
import multiprocessing as mp
from shapely.geometry.polygon import Polygon

# CAV and CIS stuff
sys.path.append("../../../")
from connected_autonomous_vehicle.src import cav
from connected_autonomous_vehicle.src import planning_control as vehicle_planning
from connected_infrastructure_sensor.src import cis
from connected_infrastructure_sensor.src import planning_stationary as camera_planning
from shared_library import sensor, global_fusion, shared_math
from road_side_unit.src import mapGenerator, communication, sensor_verification

class RSU():
    def __init__(self, config, unit_test_idx = 0):
        # Trackers for varios things in the simulation
        self.mapSpecs = mapGenerator.MapSpecs(config.map, config.map_length)
        self.vehicles = {}
        self.sensors = {}
        self.trafficLightArray = [0, 2, 0]
        self.lightTime = 0
        self.pause_simulation = True
        self.end = False

        # Settings for the simulation
        self.step_sim_vehicle = False
        self.parameterized_covariance = False
        self.simulate_error = False
        self.real_lidar = False
        self.simulation = config.simulation
        self.time = 1.0 # Time MUST start positive or it will be considered none!
        self.interval = config.interval
        self.use_global_fusion = config.use_global_fusion
        self.intersection_mode = 0
        self.intersection_serving = [-99,-99]
        self.unit_test = config.unit_test
        self.cooperative_monitoring = config.cooperative_monitoring
        self.cooperative_monitoring_update = config.cooperative_monitoring_update
        self.cooperative_monioting_step = 0
        self.rsu_ip = config.rsu_ip
        self.test_one_step_kalman = config.test_one_step_kalman
        self.end_test = False
        self.error_monitoring = []
        self.twenty_percent_error_end_and_print = config.twenty_percent_error_end_and_print
        self.revolving_buffer_size = 200
        self.missed_detection_error = 3.0

        # Init parameters for unit testing
        self.initUnitTestParams()

        # Check the fusion mode from unit tests
        if config.unit_test:
            self.unit_test_config = config.unit_test_config
            self.local_fusion_mode = self.unit_test_config[unit_test_idx][0]
            self.global_fusion_mode = self.unit_test_config[unit_test_idx][1]
            self.full_simulation = True
            self.simulate_error = True
            self.parameterized_covariance = self.unit_test_config[unit_test_idx][2]
            self.pause_simulation = False
            self.real_lidar = False
            self.unit_test_time = config.unit_test_time
            self.unit_test_speed_target = config.unit_test_speed_target
            self.unit_test_idx = unit_test_idx
            if self.twenty_percent_error_end_and_print:
                with open('output.txt', 'a') as f:
                    f.write("Test start " + str(self.unit_test_idx) + "\n")
        else:
            # Default to 1
            # TODO: add a button for this
            self.local_fusion_mode = 0
            self.global_fusion_mode = 0
            self.unit_test_idx = 0

        # init global fusion
        self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
        self.globalFusionList = []

        # For testing performance without interum Kalman Filter
        if self.test_one_step_kalman:
            self.globalFusionOneStepKalman = global_fusion.GlobalFUSION(self.global_fusion_mode)
            self.globalFusionListOneStepKalman = []
            self.global_one_step_differences = []
            self.global_one_step_over_detection_miss = 0
            self.global_one_step_under_detection_miss = 0

        # init trust score method
        self.error_dict = {}

        # Lets create the vehicles
        self.step_sim_vehicle_tracker = []
        self.step_sim_sensor_tracker = []
        for idx, vehicle in enumerate(config.cav):
            new_vehicle = vehicle_planning.Planner()
            new_vehicle.initialVehicleAtPosition(vehicle[0], vehicle[1], vehicle[2], self.mapSpecs.xCoordinates, self.mapSpecs.yCoordinates, self.mapSpecs.vCoordinates, idx, vehicle[3])
            self.vehicles[idx] = new_vehicle
            self.step_sim_vehicle_tracker.append(False)

        # Offset the IDs for the cis sensors
        self.cis_offset = len(config.cav)
        self.localization_offset = len(config.cav) + len(config.cis)

        # Lets create the sensors
        for idx, cis in enumerate(config.cis):
            new_sensor = camera_planning.Planner()
            new_sensor.initialSensorAtPosition(cis[0], cis[1], cis[2], self.mapSpecs.xCoordinates, self.mapSpecs.yCoordinates, self.mapSpecs.vCoordinates, self.cis_offset + idx, cis[3])
            self.sensors[idx] = new_sensor
            self.step_sim_sensor_tracker.append(False)

        # Queue to talk with backend processor so fast replies can be made while results are computed
        self.q = Queue()

        # Sleep for a second while we let flask get up and running
        time.sleep(1)

        # Start the falsk server for communication
        self.initFlask(config.rsu_ip)

        time.sleep(5)

        # If we are in a simulation, this will start the threads
        self.initSimulation(config)

        self.timeout = math.ceil(self.getTime())
        self.last_light = self.getTime()

        # Set unit test velocity targets
        if self.unit_test:
            for idx, each in enumerate(self.vehicles):
                self.vehicles[idx].targetVelocityGeneral = self.unit_test_speed_target

        # Create the special id for localization data from each cav
        self.localizationid = (len(config.cav) + len(config.cis)) * global_fusion.max_id


    def initUnitTestParams(self):
        # Keep track of stats if this is a simulation
        self.unit_test_state = 0
        self.unit_test_local_over_detection_miss_results = []
        self.unit_test_local_under_detection_miss_results = []
        self.unit_test_local_rmse_results = []
        self.unit_test_local_variance_results = []
        self.local_over_detection_miss = 0
        self.local_under_detection_miss = 0
        self.local_differences = []

        # localization stats
        self.unit_test_localization_rmse_results = []
        self.unit_test_localization_variance_results = []
        self.localization_differences = []

        # Global stats
        self.unit_test_global_over_detection_miss_results = []
        self.unit_test_global_under_detection_miss_results = []
        self.unit_test_global_rmse_results = []
        self.unit_test_global_variance_results = []
        self.global_over_detection_miss = 0
        self.global_under_detection_miss = 0
        self.global_differences = []

        self.localization_differences = []
        self.localization_velocity = []

    def getTime(self):
        if self.simulation:
            return self.time
        else:
            return time.time()

    def initFlask(self, rsu_ip):
        # Start up the Flask front end processor as it's own thread
        self.frontend = Thread(target=self.FlaskProccess, args=(self.q, self, rsu_ip, ))
        self.frontend.daemon = True
        self.frontend.start()

        # self.frontend = mp.Process(target=self.FlaskProccess, args=(self.q, self, rsu_ip, ))
        # self.frontend.daemon = True
        # self.frontend.start()

    def FlaskProccess(self, q, rsu_instance, rsu_ip):
        # Startup the web service
        communication.flask_app.config['RSUClass'] = rsu_instance
        communication.flask_app.config['RSUQueue'] = q
        communication.flask_app.run(host=rsu_ip, debug=True, use_reloader=False)

    def initSimulation(self, config):
         # If this is a simulation, we need to start up the CAVs and CISs as threads
        if config.simulation:

            # print("Spinning up the fake process.")
            # fake_thread = mp.Process(target=cav.fake_thread_that_just_prints, args=())
            # fake_thread.daemon = True
            # fake_thread.start()

            self.sim_time = 0.0
            self.thread = dict()
            self.step_sim_vehicle = False
            #mp.set_start_method('spawn')
            for idx, vehicle in self.vehicles.items():
                # Old way that is slow because of the Global Interpreter Lock
                # self.thread["cav"+str(idx)] = Thread(target=cav.cav, args=(config, idx, self.unit_test_idx, ))
                # self.thread["cav"+str(idx)].daemon = True
                # self.thread["cav"+str(idx)].start()

                # New actual threading
                # TODO: Figure out why this does not work!
                self.thread["cav"+str(idx)] = mp.Process(target=cav.cav, args=(config, idx, self.unit_test_idx,))
                self.thread["cav"+str(idx)].daemon = True
                self.thread["cav"+str(idx)].start()

                time.sleep(1)

                print( "RSU Initialized CAV ", idx, " thread" )

            for idx, sensor in self.sensors.items():
                # Old way that is slow because of the Global Interpreter Lock
                # self.thread["cis"+str(idx)] = Thread(target=cis.cis, args=(config, self.cis_offset + idx, self.unit_test_idx, ))
                # self.thread["cis"+str(idx)].daemon = True
                # self.thread["cis"+str(idx)].start()

                # New actual threading
                # TODO: Figure out why this does not work!
                self.thread["cis"+str(idx)] = mp.Process(target=cis.cis, args=(config, self.cis_offset + idx, self.unit_test_idx, ))
                self.thread["cis"+str(idx)].daemon = True
                self.thread["cis"+str(idx)].start()

                time.sleep(1)

                print( "RSU Initialized CIS ", idx, " thread" )

    def register(self, key, id, type, timestamp, x, y, z, roll, pitch, yaw):
        if type == 0:
            # Check if this vehicle ID is taken or not
            if id in self.vehicles:
                print ( " Warning: Vehicle ID already in use!")

            # Set the key so we have some security
            self.vehicles[id].key = key

            # Now init the vehicle at a location
            if not self.simulation:
                self.vehicles[id].update_localization(True, [x, y, yaw, 0.0])

            # Get the last known location of all other vehicles
            vehicleList = []
            for idx, vehicle in self.vehicles.items():
                if idx != id:
                    vehicleList.append(vehicle.get_location())

            # Finally we can create the return messages
            registerResponse = dict(
                v_t=self.vehicles[id].targetVelocityGeneral,
                t_x=self.vehicles[id].positionX_offset,
                t_y=self.vehicles[id].positionY_offset,
                t_z="0.0",
                t_roll="0.0",
                t_pitch="0.0",
                t_yaw=self.vehicles[id].theta_offset,
                route_x=self.mapSpecs.xCoordinates,
                route_y=self.mapSpecs.yCoordinates,
                route_TFL=self.mapSpecs.vCoordinates,
                tfl_state=self.trafficLightArray,
                veh_locations=vehicleList,
                timestep=self.getTime()
            )

            return registerResponse

        elif type == 1:
            # subtract the id offset
            id = id - self.cis_offset

            # Check if this vehicle ID is taken or not
            if id in self.sensors:
                print(" Warning: Sensor ID already in use!")

            # Set the key so we have some security
            self.sensors[id].key = key

            # Now init the vehicle at a location
            if not self.simulation:
                self.sensors[id].update_localization(False,[x, y, yaw, 0.0])

            # Get the last known location of all vehicles
            vehicleList = []
            for idx, vehicle in self.vehicles.items():
                if idx != id:
                    vehicleList.append(vehicle.get_location())

            # Finally we can create the return messages
            registerResponse = dict(
                v_t=self.sensors[id].targetVelocityGeneral,
                t_x=self.sensors[id].positionX_offset,
                t_y=self.sensors[id].positionY_offset,
                t_z="0.0",
                t_roll="0.0",
                t_pitch="0.0",
                t_yaw=self.sensors[id].theta_offset,
                route_x=self.mapSpecs.xCoordinates,
                route_y=self.mapSpecs.yCoordinates,
                route_TFL=self.mapSpecs.vCoordinates,
                tfl_state=self.trafficLightArray,
                veh_locations=vehicleList,
                timestep=self.getTime()
            )

            return registerResponse

    def checkinFastResponse(self, key, id, type, timestamp, x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, targetIntersection, detections):
        if type == 0:
            # Double check our security, this is pretty naive at this point
            #if self.vehicles[id].key == key:
            # TODO: possibly do these calculation after responding to increase response time

            # Lets add the detections to the vehicle class
            self.vehicles[id].cameraDetections = detections["cam_obj"]
            self.vehicles[id].lidarDetections = detections["lidar_obj"]
            self.vehicles[id].fusionDetections = detections["fused_obj"]

            # Update the location of this vehicle
            self.vehicles[id].localizationPositionX = detections["localization"][0]
            self.vehicles[id].localizationPositionY = detections["localization"][1]
            self.vehicles[id].theta = detections["localization"][2]
            self.vehicles[id].velocity = detections["localization"][3]
            self.vehicles[id].localizationCovariance = detections["localization"][4]
            self.vehicles[id].steeringAcceleration = steeringAcceleration
            self.vehicles[id].motorAcceleration = motorAcceleration
            self.vehicles[id].targetIndexX = targetIndexX
            self.vehicles[id].targetIndexY = targetIndexY
            self.vehicles[id].lidarDetectionsRaw = detections["lidar_detection_raw"]

            #self.step_sim_vehicle_tracker[id] = False

            # Get the last known location of all other vehicles
            vehicleList = []
            for idx, vehicle in self.vehicles.items():
                if idx != id:
                    vehicleList.append(vehicle.get_location())

            # Calculate intersection
            if self.intersection_mode == 1:
                # In the tfl 2 directions are considered, but for autonomous they need to be merged
                targetIntersection = int((targetIntersection+1) / 2)
                if targetIntersection > 0:
                    #print("intersection request: ", id, targetIntersection)
                    intersection_pos = self.mapSpecs.iCoordinates[targetIntersection-1]
                    request_distance = math.hypot(self.vehicles[id].localizationPositionX-intersection_pos[0], self.vehicles[id].localizationPositionY-intersection_pos[1])
                    if (-(1/2*math.pi) <= shared_math.angleDifference(math.atan2(self.vehicles[id].localizationPositionY-intersection_pos[1], self.vehicles[id].localizationPositionX-intersection_pos[0]), self.vehicles[id].theta)):
                        request_distance = -request_distance
                    #print("intersection request dist: ", request_distance, shared_math.angleDifference(math.atan2(self.vehicles[id].localizationPositionY-intersection_pos[1], self.vehicles[id].localizationPositionX-intersection_pos[0]), self.vehicles[id].theta))
                    av_intersection_permission = self.intersection_manager(id, request_distance, targetIntersection-1)
                else:
                    # Not approaching an interseciton, allow travel
                    av_intersection_permission = True
            else:
                av_intersection_permission = True

            # Finally we can create the return messages
            response = dict(
                v_t=self.vehicles[id].targetVelocityGeneral,
                tfl_state=self.trafficLightArray,
                veh_locations=vehicleList,
                intersection_mode=self.intersection_mode,
                av_intersection_permission=av_intersection_permission,
                timestep=self.getTime()
            )

            if self.pause_simulation:
                response['v_t'] = 0.0

            # Finally we can create the return message
            return response
        elif type == 1:
            # Double check our security, this is pretty naive at this point
            #if self.sensors[id].key == key:
            id = id - self.cis_offset
            # Lets add the detections to the CIS class
            self.sensors[id].cameraDetections = detections["cam_obj"]
            self.sensors[id].lidarDetections = detections["lidar_obj"]
            self.sensors[id].fusionDetections = detections["fused_obj"]

            # Update the location of this camera
            self.sensors[id].localizationPositionX = detections["localization"][0]
            self.sensors[id].localizationPositionY = detections["localization"][1]
            self.sensors[id].velocity = detections["localization"][3]
            self.sensors[id].theta = detections["localization"][2]
            self.sensors[id].localizationCovariance = detections["localization"][4]

            #self.step_sim_sensor_tracker[id] = False

            # Finally we can create the return messages
            response = dict(
                tfl_state=self.trafficLightArray,
                timestep=self.getTime()
            )

            # Finally we can create the return message
            return response

    def getSimPositions(self, key, id, type):
        vehicleList = []

        if type == 0:
            step_temp = self.step_sim_vehicle_tracker[id] and self.step_sim_vehicle
            if step_temp:
                self.step_sim_vehicle_tracker[id] = False
        elif type == 1:
            id = id - self.cis_offset
            step_temp = self.step_sim_sensor_tracker[id] and self.step_sim_vehicle
            if step_temp:
                self.step_sim_sensor_tracker[id] = False

        # If step_sim_vehicle is false, just send back false
        if step_temp:
            for idx, vehicle in self.vehicles.items():
                if type != 0 or idx != id:
                    vehicleList.append(vehicle.get_location())
            # Finally we can create the return messages
            response = dict(
                step_sim_vehicle=step_temp,
                parameterized_covariance=self.parameterized_covariance,
                simulate_error=self.simulate_error,
                real_lidar=self.real_lidar,
                veh_locations=vehicleList
            )
        else:
            step_temp = False
            response = dict(
                step_sim_vehicle=step_temp
            )

        # Finally we can create the return message
        return response

    def getSimTime(self):
        # Finally we can create the return messages
        response = dict(
            time=self.getTime()
        )

        # Finally we can create the return message
        return response

    def sendSimPositions(self, key, id, type, x, y, z, roll, pitch, yaw, velocity):
        if type == 0:
            # Udpate the location of this vehicle
            self.vehicles[id].localizationPositionX_actual = x
            self.vehicles[id].localizationPositionY_actual = y
            self.vehicles[id].velocity = velocity
            self.vehicles[id].theta = yaw
            #print("     cav ", id, " position ", x, y, yaw, velocity)
            #self.vehicles[id].update_localization(True, [x, y, yaw, velocity])

        # Finally we can create the return messages
        response = dict(
            returned=True
        )

        # Finally we can create the return message
        return response

    def calc_velocity(self, x1, y1, x2, y2, theta):
        velocity = math.hypot(x2 - x1, y2 - y1) * (1/8)
        expected_theta = math.atan2(y2 - y1, x2 - x1)
        if not (theta < (expected_theta + math.radians(45)) and theta > (expected_theta - math.radians(45))):
            # We are traveling backwards so adjust the velocity accordingly
            velocity = -velocity
        return velocity

    def check_state(self):
        # Check if all CAV and CIS threads have returned a result
        continue_blocker_check = False
        for each in self.step_sim_vehicle_tracker:
            if each:
                continue_blocker_check = True
                break
        for each in self.step_sim_sensor_tracker:
            if each:
                continue_blocker_check = True
                break

        # If this is simulation, enter the next state based ont he result from the block
        # checker. But if we are not in simulation, move forward if we have hit the timeout.
        if continue_blocker_check == False or ((not self.simulation) and (self.getTime() > self.timeout)):
            self.step_sim_vehicle = False

            # GLobal fusion time! (if enabled)
            if self.use_global_fusion:
                # First we need to add the localization frame, since it should be the basis
                localizationsList = []
                for idx, vehicle in self.vehicles.items():
                    # Add to the global sensor fusion w/ unique ID
                    localizationsList.append((idx+self.localizationid,
                                              vehicle.localizationPositionX,
                                              vehicle.localizationPositionY,
                                              vehicle.localizationCovariance,
                                              0,
                                              0,
                                              -1))
                    if self.unit_test:
                        self.localization_differences.append(math.hypot(vehicle.localizationPositionX_actual - vehicle.localizationPositionX,
                                                                        vehicle.localizationPositionY_actual - vehicle.localizationPositionY))
                        self.localization_velocity.append(vehicle.velocity)

                if self.cooperative_monitoring and self.cooperative_monioting_step >= self.cooperative_monitoring_update:
                    self.cooperative_monioting_step = 1
                    monitor = True
                else:
                    self.cooperative_monioting_step += 1
                    monitor = False

                #self.globalFusion.processDetectionFrame(self.getTime(), localizationsList, .25, self.parameterized_covariance)

                # If this is simulation, we need to add in the localization error for the CAVs
                if self.simulation:
                    for idx, vehicle in self.vehicles.items():
                        for detection in vehicle.fusionDetections:
                            detection[1] = detection[1] + vehicle.localizationPositionX_actual - vehicle.localizationPositionX
                            detection[2] = detection[2] + vehicle.localizationPositionY_actual - vehicle.localizationPositionY
                            detection[3] = sensor.addBivariateGaussians(np.array(vehicle.localizationCovariance), np.array(detection[3])).tolist()

                # Add CAV fusion results to the global sensor fusion
                for idx, vehicle in self.vehicles.items():
                    self.globalFusion.processDetectionFrame(self.getTime(), vehicle.fusionDetections, .25, self.parameterized_covariance)

                # Add CIS fusion results to the global sensor fusion
                for idx, sensor_ in self.sensors.items():
                    self.globalFusion.processDetectionFrame(self.getTime(), sensor_.fusionDetections, .25, self.parameterized_covariance)

                # Perform the global fusion
                self.globalFusionList, error_data = self.globalFusion.fuseDetectionFrame(self.parameterized_covariance, monitor)

                # Testing to make sure the cascading global fusion method results in the same output as a single step
                if self.test_one_step_kalman:
                    for idx, vehicle in self.vehicles.items():
                        # Add to the global sensor fusion
                        self.globalFusionOneStepKalman.processDetectionFrame(self.getTime(), vehicle.cameraDetections, .25, self.parameterized_covariance)
                        self.globalFusionOneStepKalman.processDetectionFrame(self.getTime(), vehicle.lidarDetections, .25, self.parameterized_covariance)

                    for idx, sensor_ in self.sensors.items():
                        # Add to the global sensor fusion
                        self.globalFusionOneStepKalman.processDetectionFrame(self.getTime(), sensor_.cameraDetections, .25, self.parameterized_covariance)

                    self.globalFusionListOneStepKalman, error_data_one_step = self.globalFusionOneStepKalman.fuseDetectionFrame(self.parameterized_covariance, False)

                # Use the cooperative monitoring method to check the sensors against the global fusion result
                if monitor:
                    monitor_break = self.cooperative_monitoring_process(error_data)

                    if self.twenty_percent_error_end_and_print and monitor_break:
                        return True, self.calculate_unit_test_results(), self.error_monitoring

                if self.unit_test:
                    # Uses true positions of the CAVs to ground truth the sensing.
                    # This mode is only available when in simulation mode and unit testing.
                    ground_truth = self.create_ground_truth()

                    # Ground truth the local fusion result
                    for idx, vehicle in self.vehicles.items():
                        over_detection_miss, under_detection_miss, differences = self.ground_truth_dataset(vehicle.fusionDetections, ground_truth, vehicle.id)
                        self.local_differences = self.local_differences + differences
                        self.local_over_detection_miss += over_detection_miss
                        self.local_under_detection_miss += under_detection_miss

                    for idx, sensor_ in self.sensors.items():
                        over_detection_miss, under_detection_miss, differences = self.ground_truth_dataset(sensor_.fusionDetections, ground_truth)
                        self.local_differences = self.local_differences + differences
                        self.local_over_detection_miss += over_detection_miss
                        self.local_under_detection_miss += under_detection_miss

                    # Ground truth the global fusion result
                    over_detection_miss, under_detection_miss, differences = self.ground_truth_dataset(self.globalFusionList, ground_truth)
                    self.global_differences = self.global_differences + differences
                    self.global_over_detection_miss += over_detection_miss
                    self.global_under_detection_miss += under_detection_miss

                    # Ground truth the one setp global fusion result
                    if self.test_one_step_kalman:
                        over_detection_miss, under_detection_miss, differences = self.ground_truth_dataset(self.globalFusionListOneStepKalman, ground_truth)
                        self.global_one_step_differences = self.global_one_step_differences + differences
                        self.global_one_step_over_detection_miss += over_detection_miss
                        self.global_one_step_under_detection_miss += under_detection_miss

            else:
                # We did not run the global fusion in this mode, set the list to empty
                self.globalFusionList = []

            # We have completed fusion, unblock
            self.stepSim()

            # Update the traffic light state
            self.update_traffic_lights()

            # Move forward the timestep
            self.timeout += self.interval

            # Pack values for when the GUI asks for them
            self.packGuiValues(False)

            # If we are unit testing, check if the test has ended or if we need to display intermediate results
            if self.unit_test:
                if self.time > self.unit_test_time:
                    # If the unit test has ended, returnt he results from the test
                    return True, self.calculate_unit_test_results(), self.error_monitoring
                elif self.time % 3.0 == 0:
                    # If enough time has elapsed, print the intermediate results to the terminal
                    self.calculate_unit_test_results()
                    print(self.error_monitoring)
            else:
                print(self.error_monitoring)
        
        # Return false to indicate the test has not ended
        return False, [], []
            
    def stepSim(self):
        if self.simulation:
            self.step_sim_vehicle = True
            for idx, thing in enumerate(self.step_sim_vehicle_tracker):
                self.step_sim_vehicle_tracker[idx] = True
            for idx, thing in enumerate(self.step_sim_sensor_tracker):
                self.step_sim_sensor_tracker[idx] = True
            self.time += self.interval
            print ( "Sim time Stepped @: " , self.time)

    def sendGuiValues(self, velocity_targets, pause, end, button_states):
        # Check if the user has ended it all
        if end:
            self.end = True

        # Check button states from the GUI if we are not unit testing
        if self.unit_test_state == 0:
            # If we are in unit test mode, do not take any values from the GUI
            if not self.unit_test:
                # Pause state from GUI
                self.pause_simulation = pause

                # Get CAV velocity targets from GUI
                for idx, each in enumerate(velocity_targets):
                    self.vehicles[idx].targetVelocityGeneral = each

                # Get other gui button states
                self.parameterized_covariance = button_states['parameterized_covariance']
                self.simulate_error = button_states['simulate_error']
                self.real_lidar = button_states['full_simulation']
                self.unit_test_state = button_states['unit_test']
                self.intersection_mode = button_states['intersection_mode']

        response = dict(
            returned = True
        )

        return response
    
    def packGuiValues(self, coordinates):
        vehicle_export = []
        camera_fov = []
        camera_center = []
        lidar_detection_centroid = []
        lidar_detection_raw = []
        camera_detection_centroid = []
        sensor_fusion_centroid = []
        localization_centroid = []
        localization_error = []

        sensor_export = []
        sensor_camera_fov = []
        sensor_camera_center = []
        sensor_camera_detection_centroid = []
        sensor_sensor_fusion_centroid = []
        sensor_localization_error = []

        if coordinates:
            map_specs = [self.mapSpecs.map, self.mapSpecs.intersectionStraightLength]
        else:
            map_specs = None

        for idx, vehicle in self.vehicles.items():
            # Add to the global sensor fusion
            vehicle_export.append([vehicle.localizationPositionX_actual,
                            vehicle.localizationPositionY_actual,
                            vehicle.theta,
                            vehicle.velocity,
                            vehicle.wheelbaseWidth,
                            vehicle.wheelbaseLength,
                            vehicle.steeringAcceleration,
                            vehicle.targetIndexX,
                            vehicle.targetIndexY,
                            vehicle.rearAxlePositionX,
                            vehicle.rearAxlePositionY,
                            vehicle.targetVelocity,
                            vehicle.motorAcceleration,
                            vehicle.width,
                            vehicle.length
            ])
            camera_fov.append(vehicle.cameraSensor.field_of_view)
            camera_center.append(vehicle.cameraSensor.center_angle)
            lidar_detection_raw.append(vehicle.lidarDetectionsRaw)
            lidar_detection_centroid.append(vehicle.lidarDetections)
            camera_detection_centroid.append(vehicle.cameraDetections)
            sensor_fusion_centroid.append(vehicle.fusionDetections)
            localization_error.append(vehicle.localizationCovariance)
            localization_centroid.append([vehicle.localizationPositionX,vehicle.localizationPositionY])

        for idx, sensor_ in self.sensors.items():
            # Add to the global sensor fusion
            sensor_export.append([sensor_.localizationPositionX,
                            sensor_.localizationPositionY,
                            sensor_.theta,
                            sensor_.velocity,
                            sensor_.width,
                            sensor_.length
            ])
            sensor_camera_fov.append(sensor_.cameraSensor.field_of_view)
            sensor_camera_center.append(sensor_.cameraSensor.center_angle)
            sensor_camera_detection_centroid.append(sensor_.cameraDetections)
            sensor_sensor_fusion_centroid.append(sensor_.fusionDetections)
            sensor_localization_error.append(sensor_.localizationCovariance)

        # Finally we can create the return messages
        response = dict(
            map_specs=map_specs,
            vehicle=vehicle_export,
            camera_fov=camera_fov,
            camera_center=camera_center,
            lidar_detection_raw=lidar_detection_raw,
            lidar_detection_centroid=lidar_detection_centroid,
            camera_detection_centroid=camera_detection_centroid,
            sensor_fusion_centroid=sensor_fusion_centroid,
            localization_centroid=localization_centroid,
            localization_error=localization_error,
            sensor=sensor_export,
            sensor_camera_fov=sensor_camera_fov,
            sensor_camera_center=sensor_camera_center,
            sensor_camera_detection_centroid=sensor_camera_detection_centroid,
            sensor_sensor_fusion_centroid=sensor_sensor_fusion_centroid,
            sensor_localization_error=sensor_localization_error,
            global_sensor_fusion_centroid=self.globalFusionList,
            traffic_light=self.trafficLightArray,
            error_monitoring=self.error_monitoring,
            end_test=self.end_test,
            returned=True
        )

        self.gui_state = response

    def getGuiValues(self, coordinates):
        if coordinates:
            self.packGuiValues(coordinates)

        return self.gui_state

    def update_traffic_lights(self):
        if (self.getTime() - self.last_light) >= .125:
            #print("checking light", self.lightTime, self.mapSpecs.lightTimePeriod, self.trafficLightArray)
            self.last_light = self.getTime()
            if self.lightTime > self.mapSpecs.lightTimePeriod:
                #print( "changing light" )
                self.lightTime = 0
                if self.trafficLightArray[1] == 2:
                    self.trafficLightArray[1] = 1
                    self.trafficLightArray[2] = 0
                elif self.trafficLightArray[2] == 2:
                    self.trafficLightArray[1] = 0
                    self.trafficLightArray[2] = 1
                elif self.trafficLightArray[1] == 1:
                    self.trafficLightArray[1] = 0
                    self.trafficLightArray[2] = 2
                elif self.trafficLightArray[2] == 1:
                    self.trafficLightArray[1] = 2
                    self.trafficLightArray[2] = 0
            else:
                self.lightTime += 1

    def intersection_manager(self, request_id, request_distance, intersection_id):
        # Check that the current vehicle has not left the intersection yet
        if self.intersection_serving[intersection_id] != -99:
            intersection_pos = self.mapSpecs.iCoordinates[intersection_id]
            intersection_vid = self.intersection_serving[intersection_id]
            request_distance = math.hypot(self.vehicles[intersection_vid].localizationPositionX-intersection_pos[0], self.vehicles[intersection_vid].localizationPositionY-intersection_pos[1])
            if (-(1/2*math.pi) <= shared_math.angleDifference(math.atan2(self.vehicles[intersection_vid].localizationPositionY-intersection_pos[1], self.vehicles[intersection_vid].localizationPositionX-intersection_pos[0]), self.vehicles[intersection_vid].theta)):
                request_distance = -request_distance
            if request_distance < .25:
                # Confirm the vehicle is through and reset the intersection
                self.intersection_serving[intersection_id] = -99
        # Check that we are not already in the intersection
        #print( request_id, " requested at ", request_distance, " current vehicle ", self.intersection_serving) 
        if request_distance >= .25:
            # Check if there is a request, if not then enter
            if self.intersection_serving[intersection_id] == -99 and request_distance > .25:
                self.intersection_serving[intersection_id] = request_id
                return True
            # Check if we are being served, and whether the vehicle has entered the intersection or not
            if self.intersection_serving[intersection_id] == request_id:
                # We are still approaching so continue priority
                return True
            # It is not our turn, brake
            else:
                return False
        # If the vehicle is this close, then it has entered the interseciton and shall be the go ahead
        else:
            # Check if we are the vehicle in the intersection and remove from the queue
            if self.intersection_serving[intersection_id] == request_id:
                self.intersection_serving[intersection_id] = -99
            return True

    def calculate_unit_test_results(self):
        # Calculate the prior results
        results = []

        # Localization
        rmse_val_l = shared_math.RMSE(self.localization_differences)
        variance_l = np.var(self.localization_differences,ddof=1)
        results.append(rmse_val_l)
        results.append(variance_l)

        average_velocity = sum(self.localization_velocity)/len(self.localization_velocity)
        results.append(average_velocity)

        # Onboard
        rmse_val = shared_math.RMSE(self.local_differences)
        variance = np.var(self.local_differences,ddof=1)
        results.append(rmse_val)
        results.append(variance)
        results.append(self.local_over_detection_miss)
        results.append(self.local_under_detection_miss)

        # Global
        rmse_val_g = shared_math.RMSE(self.global_differences)
        variance_g = np.var(self.global_differences,ddof=1)
        results.append(rmse_val_g)
        results.append(variance_g)
        results.append(self.global_over_detection_miss)
        results.append(self.global_under_detection_miss)

        print( "Test: ", self.unit_test_idx, " l_mode:", self.unit_test_config[self.unit_test_idx][0], " g_mode:", self.unit_test_config[self.unit_test_idx][1], " est_cov:", self.unit_test_config[self.unit_test_idx][2] )
        print( "  localization_rmse_val: ", results[0], " variance: ", results[1], " velocity ", results[2])
        print( "  local_rmse_val: ", results[3], " variance: ", results[4], " over misses: ", results[5], " under misses: ", results[6])
        print( "  global_rmse_val: ", results[7], " variance: ", results[8], " over misses: ", results[9], " under misses: ", results[10])
        
        if self.test_one_step_kalman:
            rmse_val_g_o = shared_math.RMSE(self.global_one_step_differences)
            variance_g_o = np.var(self.global_one_step_differences,ddof=1)
            results.append(rmse_val_g_o)
            results.append(variance_g_o)
            results.append(self.global_over_detection_miss)
            results.append(self.global_under_detection_miss)
            print( "  one_setp_rmse_val: ", rmse_val_g_o,
            " variance: ", variance_g_o,
            " over misses: ", self.global_one_step_over_detection_miss,
            " under misses: ", self.global_one_step_under_detection_miss)

        return results

    def reset_unit_test(self):
        # Reset the stats
        self.localization_differences = []
        self.local_over_detection_miss = 0
        self.local_under_detection_miss = 0
        self.local_differences = []
        self.global_over_detection_miss = 0
        self.global_under_detection_miss = 0
        self.global_differences = []

    def end_threads(self):
        # Send the kill signal (this is slightly hacky but doesnt need globals)
        self.time = -99
        self.end_test = True
        time.sleep(5)
        for idx, thread in self.thread.items():
            thread.join()
            time.sleep(1)
        time.sleep(1)
        import requests
        rsu_ip_address = 'http://' + str(self.rsu_ip) + ':5000'
        resp = requests.get(rsu_ip_address+'/shutdown/')

    def create_ground_truth(self):
        # Get the last known location of all other vehicles
        vehicleList = []
        for idx, vehicle in self.vehicles.items():
            vehicleList.append(vehicle.get_location())

        groundTruth = []
        for each in vehicleList:
            sensed_x = each[0]
            sensed_y = each[1]
            groundTruth.append([sensed_x, sensed_y])

        return groundTruth

    def ground_truth_dataset(self, test_list, ground_truth_list, self_id = -1):
        # Ground truth a set of sensor outputs (local or global fusion)
        test_list_converted = []
        over_detection_miss = 0
        under_detection_miss = 0
        differences = []
        
        # Take out our own id from the ground truth (if necessary)
        ground_truth_list_temp = ground_truth_list.copy()
        if self_id >= 0:
            del ground_truth_list_temp[self_id]
        
        for each in test_list:
            sensed_x = each[1]
            sensed_y = each[2]
            test_list_converted.append([sensed_x, sensed_y])
        
        if len(test_list_converted) >= 1 and len(ground_truth_list_temp) >= 1:
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(test_list_converted))
            distances, indices = nbrs.kneighbors(np.array(ground_truth_list_temp))

            # Now calculate the score
            for dist in distances:
                if dist > 1.0:
                    # Too far away to be considered a match, add as a miss instead
                    under_detection_miss += 1
                else:
                    differences.append(dist)
        # Check how much large the test set is from the ground truth and add that as well
        if len(test_list_converted) > len(ground_truth_list_temp):
            # Overdetection case
            over_detection_miss += len(test_list_converted) - len(ground_truth_list_temp)
        elif len(test_list_converted) < len(ground_truth_list_temp):
            # Underdetection case, we count this differently because it may be from obstacle blocking
            under_detection_miss += len(ground_truth_list_temp) - len(test_list_converted)

        return over_detection_miss, under_detection_miss, differences

    def cooperative_monitoring_process(self, error_data):
        # We have who saw what, but now we need to see who should have seen what
        object_polygons = []
        length = .6
        width = .6
        for idx, vehicle in enumerate(self.globalFusionList):
            # Create a bounding box for vehicle vehicle that is length + 2*buffer long and width + 2*buffer wide
            x = vehicle[1]
            y = vehicle[2]
            theta = math.atan2(vehicle[5], vehicle[4])
            x1 = x + ((length/2.0)*math.cos(theta+math.radians(90)) + ((width/2.0)*math.cos(theta-math.radians(180))))
            y1 = y + ((length/2.0)*math.sin(theta+math.radians(90)) + ((width/2.0)*math.sin(theta-math.radians(180))))
            x2 = x + ((length/2.0)*math.cos(theta-math.radians(90)) + ((width/2.0)*math.cos(theta-math.radians(180))))
            y2 = y + ((length/2.0)*math.sin(theta-math.radians(90)) + ((width/2.0)*math.sin(theta-math.radians(180))))
            x3 = x + ((length/2.0)*math.cos(theta-math.radians(90)) + ((width/2.0)*math.cos(theta)))
            y3 = y + ((length/2.0)*math.sin(theta-math.radians(90)) + ((width/2.0)*math.sin(theta)))
            x4 = x + ((length/2.0)*math.cos(theta+math.radians(90)) + ((width/2.0)*math.cos(theta)))
            y4 = y + ((length/2.0)*math.sin(theta+math.radians(90)) + ((width/2.0)*math.sin(theta)))
            polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
            object_polygons.append(polygon)

        detectors = []
        for idx, cav in self.vehicles.items():
            sublist = sensor.check_visble_objects([cav.localizationPositionX, cav.localizationPositionY, cav.theta],
                cav.cameraSensor.center_angle, cav.cameraSensor.max_distance, cav.cameraSensor.field_of_view, object_polygons)
            sublist += sensor.check_visble_objects([cav.localizationPositionX, cav.localizationPositionY, cav.theta],
                cav.lidarSensor.center_angle, cav.lidarSensor.max_distance, cav.lidarSensor.field_of_view, object_polygons)
            detectors.append(sublist)
        for idx, cis in self.sensors.items():
            sublist = sensor.check_visble_objects([cis.localizationPositionX, cis.localizationPositionY, cis.theta],
                cis.cameraSensor.center_angle, cis.cameraSensor.max_distance, cis.cameraSensor.field_of_view, object_polygons)
            detectors.append(sublist)

        # append an empty set for the localizers
        detectors.append([])

        # Add our covariance data to the global sensor list
        for object_id, object_trackers in enumerate(error_data):
            for error_frame in object_trackers:
                # Check if this is a localizer or a sensor
                if error_frame[0]/self.localizationid >= 1:
                    sensor_platform_id = error_frame[0]
                else:
                    sensor_platform_id = math.floor(error_frame[0]/10000)
                self.add_error_frame(sensor_platform_id, error_frame[1], error_frame[2])
                
                # Check off detected objects if they exist
                if sensor_platform_id < self.localizationid:
                    for objects_should_be_seen_id in reversed(range(len(detectors[sensor_platform_id]))):
                        if object_id == detectors[sensor_platform_id][objects_should_be_seen_id]:
                            detectors[sensor_platform_id].pop(objects_should_be_seen_id)

        # Add the error for missed detections
        # for sensor_platform_id in self.error_dict.keys():
        #     # Check off what we have seen
        #     if sensor_platform_id < self.localizationid:
        #         for seen_obj_id in range(len(detectors[sensor_platform_id])):
        #             self.add_error_frame(sensor_platform_id, self.missed_detection_error, .057)

        # Normalize all the data to 0 (hopefully)
        normalization_numerator = 0.0
        normalization_denominator = 0.0
        for key in self.error_dict.keys():
            if self.error_dict[key][0] > 5 and int(key) < self.localization_offset:
                normalization_numerator += sum(self.error_dict[key][2])
                normalization_denominator += self.error_dict[key][0]
        
        # Make sure the fenominator is greater than 0
        if normalization_denominator != 0.0:
            error_monitoring_normalizer = normalization_numerator / normalization_denominator
        else:
            error_monitoring_normalizer = 1.0

        self.error_monitoring = []
        twenty_percent_break_check = False
        for key in self.error_dict.keys():
            if self.error_dict[key][0] > 5:
                average_error = sum(self.error_dict[key][2])/self.error_dict[key][0]
                average_Expected_error = sum(self.error_dict[key][3])/self.error_dict[key][0]
                if int(key) < self.localization_offset:
                    average_error = average_error / error_monitoring_normalizer
                self.error_monitoring.append([key, average_error, average_Expected_error, self.error_dict[key][0]])
                
                # Only break once the revolving buffer is full
                if self.twenty_percent_error_end_and_print and self.error_dict[key][0] >= self.revolving_buffer_size and average_error > 1.2:
                    #twenty_percent_break_check = True
                    with open('output.txt', 'a') as f:
                        f.write(str(self.time) + "," + str(average_error) + "\n")
                        print("breaking test!" + str(self.time-.125) + "," + str(average_error) + "\n")

                # if self.time > 95 and int(key) == 0:
                #     with open('output.txt', 'a') as f:
                #         f.write(str(self.time) + "," + str(average_error) + "\n")
                #         print("writing to file!" + str(self.time-.125) + "," + str(average_error) + "\n")

        return twenty_percent_break_check

    def add_error_frame(self, sensor_platform_id, error_std, num_error):
        # Allow time for test warmup
        if self.time > 5.0 and num_error >= 3:
            if sensor_platform_id in self.error_dict:
                # Moving revolving_buffer_size place average
                if self.error_dict[sensor_platform_id][0] < self.revolving_buffer_size:
                    self.error_dict[sensor_platform_id][0] += 1
                    self.error_dict[sensor_platform_id][2].append(error_std)
                    self.error_dict[sensor_platform_id][3].append(num_error)
                    self.error_dict[sensor_platform_id][1] += 1
                # We have filled revolving_buffer_size places, time to revolve the buffer now
                else:
                    if self.error_dict[sensor_platform_id][1] < self.revolving_buffer_size:
                        # Replace the element with the next one
                        self.error_dict[sensor_platform_id][2][self.error_dict[sensor_platform_id][1]] = error_std
                        self.error_dict[sensor_platform_id][3][self.error_dict[sensor_platform_id][1]] = num_error
                        self.error_dict[sensor_platform_id][1] += 1
                    else:
                        self.error_dict[sensor_platform_id][1] = 0
                        self.error_dict[sensor_platform_id][2][self.error_dict[sensor_platform_id][1]] = error_std
                        self.error_dict[sensor_platform_id][3][self.error_dict[sensor_platform_id][1]] = num_error
                        self.error_dict[sensor_platform_id][1] += 1
            else:
                self.error_dict[sensor_platform_id] = [1, 1, [error_std], [num_error]]
