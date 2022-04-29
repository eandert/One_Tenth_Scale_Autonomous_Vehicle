import time
import math
import sys
import numpy as np
from sklearn.neighbors import NearestNeighbors
from threading import Lock, Thread
from queue import Queue
import multiprocessing as mp

# CAV and CIS stuff
sys.path.append("../../../")
from connected_autonomous_vehicle.src import cav
from connected_autonomous_vehicle.src import planning_control as vehicle_planning
from connected_infrastructure_sensor.src import cis
from connected_infrastructure_sensor.src import planning_stationary as camera_planning
from shared_library import sensor, global_fusion, shared_math
from road_side_unit.src import mapGenerator, communication, sensor_verification

class RSU():
    def __init__(self, config):
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
        self.no_global_fusion = config.no_global_fusion
        self.intersection_mode = 0
        self.intersection_serving = [-99,-99]

        # Unit test settings
        self.unit_test_state = 0

        # Init parameters for unit testing
        self.initUnitTestParams()

        # Check the fusion mode from unit tests
        if config.unit_test:
            self.unitTest = config.unit_test_config
            self.local_fusion_mode = self.unitTest[0][0]
            self.global_fusion_mode = self.unitTest[0][1]
        else:
            # Default to 1
            # TODO: add a button for this
            self.local_fusion_mode = 0
            self.global_fusion_mode = 0

        # init global fusion
        self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
        self.globalFusionList = []

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

        # Create the special id for localization data from each cav
        self.localizationid = (1 + len(config.cav) + len(config.cis)) * global_fusion.max_id


    def initUnitTestParams(self):
        # Keep track of stats if this is a simulation
        self.unit_test_state = 0
        self.unit_test_idx = 0
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
        frontend = Thread(target=self.FlaskProccess, args=(self.q, self, rsu_ip, ))
        frontend.daemon = True
        frontend.start()

    def FlaskProccess(self, q, rsu_instance, rsu_ip):
        # Startup the web service
        communication.flask_app.config['RSUClass'] = rsu_instance
        communication.flask_app.config['RSUQueue'] = q
        communication.flask_app.run(host=rsu_ip, debug=True, use_reloader=False)

    def initSimulation(self, config):
         # If this is a simulation, we need to start up the CAVs and CISs as threads
        if config.simulation:
            self.sim_time = 0.0
            self.thread = dict()
            self.step_sim_vehicle = False
            #mp.set_start_method('spawn')
            for idx, vehicle in self.vehicles.items():
                # Old way that is slow because of the Global Interpreter Lock
                self.thread["cav"+str(idx)] = Thread(target=cav.cav, args=(config, idx, ))
                self.thread["cav"+str(idx)].daemon = True
                self.thread["cav"+str(idx)].start()

                # New actual threading
                # self.thread["cav"+str(idx)] = mp.Process(target=cav.cav, args=(config, idx))
                # self.thread["cav"+str(idx)].daemon = True
                # self.thread["cav"+str(idx)].start()

                # time.sleep(1)

                print( "RSU Initialized CAV ", idx, " thread" )

            for idx, sensor in self.sensors.items():
                # Old way that is slow because of the Global Interpreter Lock
                self.thread["cis"+str(idx)] = Thread(target=cis.cis, args=(config, self.cis_offset + idx, ))
                self.thread["cis"+str(idx)].daemon = True
                self.thread["cis"+str(idx)].start()

                # New actual threading
                # self.thread["cav"+str(idx)] = mp.Process(target=cav.cav, args=(config, idx))
                # self.thread["cav"+str(idx)].daemon = True
                # self.thread["cav"+str(idx)].start()

                # time.sleep(1)

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

            # Get the last known location of all other sensors
            vehicleList = []
            for idx, vehicle in self.sensors.items():
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
            if not self.simulation:
                self.vehicles[id].localizationPositionX = detections["localization"][0]
                self.vehicles[id].localizationPositionY = detections["localization"][1]
                self.vehicles[id].velocity = detections["localization"][3]
                self.vehicles[id].theta = detections["localization"][2]
            self.vehicles[id].localizationCovariance = detections["localization"][4]
            self.vehicles[id].steeringAcceleration = steeringAcceleration
            self.vehicles[id].motorAcceleration = motorAcceleration
            self.vehicles[id].targetIndexX = targetIndexX
            self.vehicles[id].targetIndexY = targetIndexY
            self.vehicles[id].lidarDetectionsRaw = detections["lidar_detection_raw"]

            self.step_sim_vehicle_tracker[id] = False

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
            if not self.simulation:
                self.sensors[id].localizationPositionX = detections["localization"][0]
                self.sensors[id].localizationPositionY = detections["localization"][1]
                self.sensors[id].velocity = detections["localization"][3]
                self.sensors[id].theta = detections["localization"][2]
            self.sensors[id].localizationCovariance = detections["localization"][4]

            self.step_sim_sensor_tracker[id] = False

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
            self.vehicles[id].localizationPositionX = x
            self.vehicles[id].localizationPositionY = y
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
        # Check if we are ready for sensor fusion
        continue_blocker_check = False
        for each in self.step_sim_vehicle_tracker:
            if each:
                continue_blocker_check = True
        for each in self.step_sim_sensor_tracker:
            if each:
                continue_blocker_check = True

        if continue_blocker_check == False or (not self.simulation and self.getTime() > self.timeout):
            self.step_sim_vehicle = False

            if not self.no_global_fusion:
                # Fusion time!
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
                self.globalFusion.processDetectionFrame(self.getTime(), localizationsList, .25, self.parameterized_covariance)

                for idx, vehicle in self.vehicles.items():
                    # Add to the global sensor fusion
                    self.globalFusion.processDetectionFrame(self.getTime(), vehicle.fusionDetections, .25, self.parameterized_covariance)

                for idx, sensor in self.sensors.items():
                    # Add to the global sensor fusion
                    self.globalFusion.processDetectionFrame(self.getTime(), sensor.fusionDetections, .25, self.parameterized_covariance)

                self.globalFusionList, error_data = self.globalFusion.fuseDetectionFrame(self.parameterized_covariance)

                # Add our covariance data to the global sensor list
                revolving_buffer_size = 1000
                for error_frame in error_data:
                    # Check if this is a localizer or a sensor
                    if error_frame[0]/self.localizationid >= 1:
                        sensor_platform_id = error_frame[0]
                    else:
                        sensor_platform_id = math.floor(error_frame[0]/10000)
                    if sensor_platform_id in self.error_dict:
                        # Moving revolving_buffer_size place average
                        if self.error_dict[sensor_platform_id][0] < revolving_buffer_size:
                            self.error_dict[sensor_platform_id][0] += 1
                            self.error_dict[sensor_platform_id][2].append(error_frame[2])
                        # We have filled revolving_buffer_size places, time to revolve the buffer now
                        else:
                            if self.error_dict[sensor_platform_id][1] < revolving_buffer_size:
                                # Replace the element with the next one
                                self.error_dict[sensor_platform_id][2][self.error_dict[sensor_platform_id][1]] = error_frame[2]
                                self.error_dict[sensor_platform_id][1] += 1
                            else:
                                self.error_dict[sensor_platform_id][1] = 0
                                self.error_dict[sensor_platform_id][2][self.error_dict[sensor_platform_id][1]] = error_frame[2]
                                self.error_dict[sensor_platform_id][1] += 1
                    else:
                        self.error_dict[sensor_platform_id] = [1,0,[error_frame[2]]]

                # Ground truth to the original dataset
                # Get the last known location of all other vehicles
                vehicleList = []
                for idx, vehicle in self.vehicles.items():
                    vehicleList.append(vehicle.get_location())
                testSetGlobal = []
                groundTruthGlobal = []
                for each in self.globalFusionList:
                    sensed_x = each[1]
                    sensed_y = each[2]
                    testSetGlobal.append([sensed_x, sensed_y])
                for each in vehicleList:
                    sensed_x = each[0]
                    sensed_y = each[1]
                    groundTruthGlobal.append([sensed_x, sensed_y])
                if len(testSetGlobal) >= 1 and len(groundTruthGlobal) >= 1:
                    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(testSetGlobal))
                    distances, indices = nbrs.kneighbors(np.array(groundTruthGlobal))

                    # Now calculate the score
                    for dist in distances:
                        if dist > 1.0:
                            # Too far away to be considered a match, add as a miss instead
                            self.global_under_detection_miss += len(groundTruthGlobal) - len(testSetGlobal)
                        else:
                            self.global_differences.append(dist)
                # Check how much large the test set is from the ground truth and add that as well
                if len(testSetGlobal) > len(groundTruthGlobal):
                    # Overdetection case
                    self.global_over_detection_miss += len(testSetGlobal) - len(groundTruthGlobal)
                elif len(testSetGlobal) < len(groundTruthGlobal):
                    # Underdetection case, we count this differently because it may be from obstacle blocking
                    self.global_under_detection_miss += len(groundTruthGlobal) - len(testSetGlobal)
            else:
                self.globalFusionList = []

            # We have completed fusion, unblock
            self.stepSim()
            self.update_traffic_lights()

            self.timeout += self.interval
            

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
            # Pause state from GUI
            self.pause_simulation = pause

            # Get CAV velocity targets from GUI
            for idx, each in enumerate(velocity_targets):
                self.vehicles[idx].targetVelocityGeneral = each

            #print( " trying to get values from gui! ")

            # Get other gui button states
            self.parameterized_covariance = button_states['parameterized_covariance']
            self.simulate_error = button_states['simulate_error']
            self.real_lidar = button_states['full_simulation']
            self.unit_test_state = button_states['unit_test']
            self.intersection_mode = button_states['intersection_mode']

            #print( " got values from gui! ")

        response = dict(
            returned = True
        )

        return response
    
    def getGuiValues(self, coordinates):
        vehicle_export = []
        camera_fov = []
        camera_center = []
        lidar_detection_centroid = []
        lidar_detection_raw = []
        camera_detection_centroid = []
        sensor_fusion_centroid = []
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

        error_monitoring = []
        for key in self.error_dict.keys():
            if self.error_dict[key][0] > 5:
                average_error = sum(self.error_dict[key][2])/self.error_dict[key][0]
                error_monitoring.append(key, average_error, self.error_dict[key][0])
                #print(" ID ", key, " error ", average_error, " len ", self.error_dict[key][0])

        for idx, vehicle in self.vehicles.items():
            # Add to the global sensor fusion
            vehicle_export.append([vehicle.localizationPositionX,
                            vehicle.localizationPositionY,
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

        for idx, sensor in self.sensors.items():
            # Add to the global sensor fusion
            sensor_export.append([sensor.localizationPositionX,
                            sensor.localizationPositionY,
                            sensor.theta,
                            sensor.velocity,
                            sensor.width,
                            sensor.length
            ])
            sensor_camera_fov.append(sensor.cameraSensor.field_of_view)
            sensor_camera_center.append(sensor.cameraSensor.center_angle)
            sensor_camera_detection_centroid.append(sensor.cameraDetections)
            sensor_sensor_fusion_centroid.append(sensor.fusionDetections)
            sensor_localization_error.append(sensor.localizationCovariance)

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
            localization_error=localization_error,
            sensor=sensor_export,
            sensor_camera_fov=sensor_camera_fov,
            sensor_camera_center=sensor_camera_center,
            sensor_camera_detection_centroid=sensor_camera_detection_centroid,
            sensor_sensor_fusion_centroid=sensor_sensor_fusion_centroid,
            sensor_localization_error=sensor_localization_error,
            global_sensor_fusion_centroid=self.globalFusionList,
            traffic_light=self.trafficLightArray,
            error_monitoring=error_monitoring,
            returned=True
        )

        return response

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


# def BackendProcessor(q, vehicles, sensors, trafficLightArray):
#     while True:
#         message = q.get()
#         #print ( message )
#         key, id, type, timestamp, x, y, yaw, detections = message

#         # See if we are dealing with a sensor or a vehicle
#         if type == 1:
#             # Double check our security, this is pretty naive at this point
#             if sensors[id].key == key:
#                 # Lets add the detections to the sensor class
#                 sensors[id].cameraDetections = detections["cam_obj"]
#                 # with open("output.csv", "a") as file:
#                 #     file.write("cam," + str(id) + "," + str(x)
#                 #                + "," + str(y)
#                 #                + "," + str(yaw)
#                 #                + "," + str(detections)
#                 #                + "\n")
#         elif type == 0:
#             # Double check our security, this is pretty naive at this point
#             if vehicles[id].key == key:
#                 # We do these calculation after responding to increase response time
#                 vehicles[id].recieve_coordinate_group_commands(trafficLightArray)

#                 # We update this just for the visualizer
#                 vehicles[id].pure_pursuit_control()

#                 # Lets add the detections to the vehicle class
#                 vehicles[id].cameraDetections = detections["cam_obj"]
#                 vehicles[id].lidarDetections = detections["lidar_obj"]
#                 vehicles[id].fusion_result = detections["fused_obj"]

#                 # Update the location of this vehicle
#                 vehicles[id].localizationPositionX = detections["localization"][0]
#                 vehicles[id].localizationPositionY = detections["localization"][1]
#                 vehicles[id].velocity = detections["localization"][3]
#                 vehicles[id].theta = detections["localization"][2]

#                 # Get the last known location of all other vehicles
#                 vehicleList = []
#                 for idx, vehicle in vehicles.items():
#                     if idx != id:
#                         vehicleList.append(vehicle.get_location())

#                 # Now update our current PID with respect to other vehicles
#                 vehicles[id].check_positions_of_other_vehicles_adjust_velocity(vehicleList)

#                 # We can't update the PID controls until after all positions are known
#                 # We still do this here just for debugging as it should match the PID controls
#                 # on the actual car and then it will be displayed on the UI
#                 vehicles[id].update_pid()

                # with open("output.csv", "a") as file:
                #     file.write("cav," + str(id) + "," + str(x)
                #                + "," + str(y)
                #                + "," + str(yaw)
                #                + "," + str(detections)
                #                +"\n")

# def updateVehicleInSim(self, idx, full_simulation, vehicleList, thread_idx):
#         # Update ourself
#         vehicle = self.vehicles[idx]
#         vehicle.update_localization()
#         vehicle.recieve_coordinate_group_commands(self.trafficLightArray)
#         vehicle.pure_pursuit_control()

#         # Filter out ourself
#         tempList = vehicleList.copy()
#         tempList.pop(idx)

#         localization_error = [0.0, 0.0]

#         if full_simulation:
#             # Create that fake LIDAR
#             if self.lidarRecognitionList[idx] != None:
#                 localization_error_gaussian, localization_error = vehicle.localization.getErrorParamsAtVelocity(abs(vehicle.velocity), vehicle.theta)
#                 if self.parameterized_covariance:
#                     temp_covariance = localization_error_gaussian
#                 else:
#                     temp_covariance = sensor.BivariateGaussian(0.175, 0.175, 0)
#                 point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(vehicle, tempList, [], 15.0, 15.0, 0.0, 160.0, l_error = localization_error, l_error_gauss = temp_covariance)
#                 if self.simulate_error:
#                     vehicle.cameraDetections = camera_error_array
#                     vehicle.localizationError = localization_error_gaussian
#                     if self.real_lidar:
#                         lidarcoordinates, lidartimestamp = self.lidarRecognitionList[idx].processLidarFrame(point_cloud_error, self.time/1000.0,
#                             vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, vehicle.lidarSensor)
#                         vehicle.rawLidarDetections = point_cloud_error
#                         vehicle.lidarDetections = lidarcoordinates
#                     else:
#                         vehicle.lidarDetections = lidar_detected_error
#                 else:
#                     vehicle.cameraDetections = camera_array
#                     lidarcoordinates, lidartimestamp = self.lidarRecognitionList[idx].processLidarFrame(point_cloud, self.time/1000.0,
#                         vehicle.localizationPositionX, vehicle.localizationPositionY, vehicle.theta, vehicle.lidarSensor)
#                     vehicle.rawLidarDetections = point_cloud
#                     vehicle.lidarDetections = lidarcoordinates
#                 vehicle.groundTruth = camera_array

#                 # Vehicle position can be the map centroid in sim
#                 # because we are generating the detection WRT the centroid
#                 #pos = [vehicle.localizationPositionX - vehicle.positionX_offset, vehicle.localizationPositionY - vehicle.positionY_offset, vehicle.theta - vehicle.theta_offset]
#                 #pos = [0,0,0]

#                 # Lets add the detections to the vehicle class
#                 # vehicle.lidarDetections = []
#                 # for each in lidarcoordinates:
#                 #     new = rotate((0, 0), (float(each[1]), float(each[2])), pos[2])
#                 #     sensed_x = new[0] + pos[0]
#                 #     sensed_y = new[1] + pos[1]
#                 #     vehicle.lidarDetections.append((sensed_x, sensed_y, each[6]))

#                 # Raw LIDAR for debug
#                 vehicle.lidarPoints = point_cloud

#                 # Do the local fusion like we would on the vehicle
#                 self.localFusionCAV[idx].processDetectionFrame(local_fusion.CAMERA, self.time/1000.0, vehicle.cameraDetections, .25, self.parameterized_covariance)
#                 self.localFusionCAV[idx].processDetectionFrame(local_fusion.LIDAR, self.time/1000.0, vehicle.lidarDetections, .25, self.parameterized_covariance)
#                 results = self.localFusionCAV[idx].fuseDetectionFrame(self.parameterized_covariance, vehicle)

#                 # Add to the GUI
#                 vehicle.fusionDetections = []
#                 for each in results:
#                     sensed_x = each[1]
#                     sensed_y = each[2]
#                     vehicle.fusionDetections.append((sensed_x, sensed_y, each[3], each[4], each[5], each[0]))
#                 # Add ourself to this
#                 self.localizationsList.append((vehicle.localizationPositionX + localization_error[0],
#                                                 vehicle.localizationPositionY + localization_error[1],
#                                                 localization_error_gaussian.covariance, 0, 0, -1))

#         else:
#             # Quick fake of sensor values
#             vehicle.fusionDetections = []
#             for each in tempList:
#                 sensed_x = each[0]
#                 sensed_y = each[1]
#                 vehicle.fusionDetections.append((sensed_x, sensed_y, np.array([[1.0, 0.0],[0.0, 1.0]]), 0, 0, each[5]))

#         # Now update our current PID with respect to other vehicles
#         vehicle.check_positions_of_other_vehicles_adjust_velocity(tempList)

#         # We can't update the PID controls until after all positions are known
#         vehicle.update_pid()

#         # Ground truth to the original dataset
#         testSet = []
#         groundTruth = []
#         for each in vehicle.fusionDetections:
#             sensed_x = each[0]
#             sensed_y = each[1]
#             testSet.append([sensed_x, sensed_y])
#         for each in vehicle.groundTruth:
#             sensed_x = each[0]
#             sensed_y = each[1]
#             groundTruth.append([sensed_x, sensed_y])

#         local_differences = []
#         local_over_detection_miss = 0
#         local_under_detection_miss = 0

#         if len(testSet) >= 1 and len(groundTruth) >= 1:
#             nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(testSet))
#             distances, indices = nbrs.kneighbors(np.array(groundTruth))

#             # Now calculate the score
#             for dist in distances:
#                 local_differences.append(dist)

#         # Check how much large the test set is from the ground truth and add that as well
#         if len(testSet) > len(groundTruth):
#             # Overdetection case
#             local_over_detection_miss += len(testSet) - len(groundTruth)
#         elif len(testSet) < len(groundTruth):
#             # Underdetection case, we count this differently because it may be from obstacle blocking
#             local_under_detection_miss += len(groundTruth) - len(testSet)

#         if self.useThreading:
#             self.threads_results[thread_idx] = [local_differences, local_over_detection_miss, local_under_detection_miss, localization_error]
#         else:
#             return local_differences, local_over_detection_miss, local_under_detection_miss, localization_error

#     def updateCISInSim(self, idx, full_simulation, vehicleList, thread_idx):
#         cis = self.cis[idx]
#         # Don't filter the list at all
#         tempList = vehicleList.copy()
#         if full_simulation:
#             # Create that fake camera
#             if self.lidarRecognitionList[idx] != None:
#                 point_cloud, point_cloud_error, camera_array, camera_error_array, lidar_detected_error = sensor.fake_lidar_and_camera(cis, tempList, [], 15.0, 15.0, 0.0, 160.0)
#                 if self.simulate_error:
#                     cis.cameraDetections = camera_error_array
#                 else:
#                     cis.cameraDetections = camera_array
#                 cis.groundTruth = camera_array
                
#                 # f = open("data_" + str(idx) + ".txt", "a")
#                 # f.write(str(idx) + "," + str(self.time))
#                 # for each in cis.cameraDetections:
#                 #     f.write("," + str(each[0]) + "," + str(each[1]))
#                 # f.write("\n")
#                 # f.close()

#                 # CIS position can be the map centroid in sim
#                 # because we are generating the detection WRT the centroid
#                 # pos = [cis.localizationPositionX - cis.positionX_offset, cis.localizationPositionY - cis.positionY_offset, cis.theta - cis.theta_offset]
#                 pos = [0, 0, 0]

#                 # Fusion detection frame is the same as single camera (for now)
#                 # Add to the GUI
#                 # Do the local fusion like we would on the vehicle
#                 self.localFusionCIS[idx].processDetectionFrame(local_fusion.CAMERA, self.time/1000.0, cis.cameraDetections, .25, self.parameterized_covariance)
#                 results = self.localFusionCIS[idx].fuseDetectionFrame(self.parameterized_covariance, cis)

#                 # Add to the GUI
#                 cis.fusionDetections = []
#                 for each in results:
#                     sensed_x = each[1]
#                     sensed_y = each[2]
#                     cis.fusionDetections.append((sensed_x, sensed_y, each[3], each[4], each[5], each[0]))
#         else:
#             # Quick fake of sensor values
#             cis.fusionDetections = []
#             cis.groundTruth = []
#             for each in vehicleList:
#                 sensed_x = each[0]
#                 sensed_y = each[1]
#                 cis.fusionDetections.append((sensed_x, sensed_y, np.array([[1.0, 0.0],[0.0, 1.0]]), 0, 0, each[5]))

#         # Ground truth to the original dataset
#         testSet = []
#         groundTruth = []
#         for each in cis.fusionDetections:
#             sensed_x = each[0]
#             sensed_y = each[1]
#             testSet.append([sensed_x, sensed_y])
#         for each in cis.groundTruth:
#             sensed_x = each[0]
#             sensed_y = each[1]
#             groundTruth.append([sensed_x, sensed_y])

#         local_differences = []
#         local_over_detection_miss = 0
#         local_under_detection_miss = 0

#         if len(testSet) >= 1 and len(groundTruth) >= 1:
#             nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(testSet))
#             distances, indices = nbrs.kneighbors(np.array(groundTruth))

#             # Now calculate the score
#             for dist in distances:
#                 local_differences.append(dist)

#         # Check how much large the test set is from the ground truth and add that as well
#         if len(testSet) > len(groundTruth):
#             # Overdetection case
#             local_over_detection_miss += len(testSet) - len(groundTruth)
#         elif len(testSet) < len(groundTruth):
#             # Underdetection case, we count this differently because it may be from obstacle blocking
#             local_under_detection_miss += len(groundTruth) - len(testSet)

#         if self.useThreading:
#             self.threads_results[thread_idx] = [local_differences, local_over_detection_miss, local_under_detection_miss, [0.0, 0.0]]
#         else:
#             return local_differences, local_over_detection_miss, local_under_detection_miss

#     def calcResultsOfUnitTest(self):
#         # Calculate the prior results
#         # Localization
#         differences_squared_l = np.array(self.localization_differences) ** 2
#         mean_of_differences_squared_l = differences_squared_l.mean()
#         rmse_val_l = np.sqrt(mean_of_differences_squared_l)
#         variance_l = np.var(self.localization_differences,ddof=1)

#         self.unit_test_localization_rmse_results.append(rmse_val_l)
#         self.unit_test_localization_variance_results.append(variance_l)

#         # Onboard
#         differences_squared = np.array(self.local_differences) ** 2
#         mean_of_differences_squared = differences_squared.mean()
#         rmse_val = np.sqrt(mean_of_differences_squared)
#         variance = np.var(self.local_differences,ddof=1)

#         self.unit_test_local_rmse_results.append(rmse_val)
#         self.unit_test_local_variance_results.append(variance)
#         self.unit_test_local_under_detection_miss_results.append(self.local_under_detection_miss)
#         self.unit_test_local_over_detection_miss_results.append(self.local_over_detection_miss)

#         # Global
#         differences_squared_g = np.array(self.global_differences) ** 2
#         mean_of_differences_squared_g = differences_squared_g.mean()
#         rmse_val_g = np.sqrt(mean_of_differences_squared_g)
#         variance_g = np.var(self.global_differences,ddof=1)

#         self.unit_test_global_rmse_results.append(rmse_val_g)
#         self.unit_test_global_variance_results.append(variance_g)
#         self.unit_test_global_under_detection_miss_results.append(self.global_under_detection_miss)
#         self.unit_test_global_over_detection_miss_results.append(self.global_over_detection_miss)

#     def printUnitTestStats(self):
#         idx = 0
#         fails = 0
#         for l_rmse, l_var, o_rmse, o_var, o_u_miss, o_o_miss, g_rmse, g_var, g_u_miss, g_o_miss in zip(self.unit_test_localization_rmse_results, self.unit_test_localization_variance_results, 
#             self.unit_test_local_rmse_results, self.unit_test_local_variance_results,
#             self.unit_test_local_under_detection_miss_results, self.unit_test_local_over_detection_miss_results,
#             self.unit_test_global_rmse_results, self.unit_test_global_variance_results,
#             self.unit_test_global_under_detection_miss_results, self.unit_test_global_over_detection_miss_results):
#             print( "Test: ", idx, " g_mode:", self.unitTest[idx][0], " l_mode:", self.unitTest[idx][1], " est_cov:", self.unitTest[idx][2] )
#             print( "  localization_rmse_val: ", l_rmse, " variance: ", l_var)
#             print( "  onboard_rmse_val: ", o_rmse, " variance: ", o_var, " over misses: ", o_o_miss, " under misses: ", o_u_miss)
#             print( "  global_rmse_val: ", g_rmse, " variance: ", g_var, " over misses: ", g_o_miss, " under misses: ", g_u_miss)
#             # if idx == 0:
#             #     if rmse < .18 or rmse > 20 or O_miss > (50 * test_time):
#             #         fails += 1
#             # elif idx == 1:
#             #     if rmse < .18 or rmse > 20 or O_miss > (50 * test_time):
#             #         fails += 1
#             idx += 1

#     def resetUnitTestStats(self):
#         # Reset the stats
#         self.localization_differences = []
#         self.local_over_detection_miss = 0
#         self.local_under_detection_miss = 0
#         self.local_differences = []
#         self.global_over_detection_miss = 0
#         self.global_under_detection_miss = 0
#         self.global_differences = []

#     def resetTest(self):
#         # Reset vehicle positions and filters
#         for idx, vehicle in self.vehicles.items():
#             # Clear sensors
#             vehicle.cameraDetections = []
#             vehicle.lidarDetections = []
#             vehicle.fusionDetections = []
#             vehicle.fusionDetectionsCovariance = []
#             vehicle.rawLidarDetections = []
#             vehicle.groundTruth = []

#             # Raw LIDAR for gui debug
#             vehicle.lidarPoints = []

#             # Move back to start location
#             reverse_theta = vehicle.theta_offset-math.radians(180)
#             vehicle.rearAxlePositionX = vehicle.positionX_offset + (vehicle.axleFromCenter * math.cos(reverse_theta))
#             vehicle.rearAxlePositionY = vehicle.positionY_offset + (vehicle.axleFromCenter * math.sin(reverse_theta))
#             vehicle.localizationPositionX = vehicle.positionX_offset
#             vehicle.localizationPositionY = vehicle.positionY_offset
#             vehicle.positionX_sim = vehicle.rearAxlePositionX
#             vehicle.positionY_sim = vehicle.rearAxlePositionY
#             vehicle.velocity = 0
#             vehicle.theta = vehicle.theta_offset
#             vehicle.lastPointIndex = None

#         # Reset detections from CIS sensors
#         # Reset vehicle positions and filters
#         for idx, cis in self.cis.items():
#             # Clear sensors
#             cis.cameraDetections = []
#             cis.fusionDetections = []
#             cis.fusionDetectionsCovariance = []
#             cis.groundTruth = []

#         # Clear the global fusion
#         self.globalFusion.trackedList = []

#         # Reset the light
#         self.trafficLightArray = [0, 2, 0]

#     def stepTime(self):
#         if self.unit_test:
#             test_time = 60000
#             test_time_print = 10000
#             if self.time % test_time_print == 0:
#                 print("Test: ", 100 * (self.time % test_time)/test_time, "% num:", self.unit_test_idx)
#             if self.time % test_time == 0:
#                 # Reset the map, unit testing has been selected
#                 self.resetTest()

#                 # Determing mode
#                 if self.unit_test_state == 0:
#                     for idx, vehicle in self.vehicles.items():
#                         self.lineVehicleSpeed[idx].setText("0.5")
#                         self.lineVehicleSpeed[idx].setReadOnly(True)
#                     self.sensorsButton.setEnabled(False)
#                     self.errorButton.setEnabled(False)
#                     self.covarianceButton.setEnabled(False)
#                     self.endButton.setEnabled(True)
#                     self.pauseButton.setEnabled(False)
#                     self.startButton.setEnabled(False)
#                     self.unitTestButton.setEnabled(False)
#                     self.full_simulation = True
#                     self.simulate_error = True
#                     self.parameterized_covariance = False
#                     self.pause_simulation = False
#                     self.real_lidar = False
#                     self.unit_test_idx = 0

#                     # Set the fusion modes
#                     self.local_fusion_mode = self.unitTest[self.unit_test_idx][0]
#                     self.global_fusion_mode = self.unitTest[self.unit_test_idx][1]
#                     self.parameterized_covariance = self.unitTest[self.unit_test_idx][2]
#                     self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
#                     for idx, veh in self.vehicles.items():
#                         if veh.simVehicle:
#                             self.localFusionCAV[idx].fusion_mode = self.local_fusion_mode
#                     for idx, sens in self.cis.items():
#                         if sens.simCIS:
#                             self.localFusionCIS[idx].fusion_mode = self.local_fusion_mode

#                     # Reset the stats
#                     self.resetUnitTestStats()
    
#                     # Increment the unit test counter for those long tests
#                     self.unit_test_state = 1
#                     self.unit_test_idx += 1
#                 elif len(self.unitTest) <= self.unit_test_idx:
#                     # Calculate the prior results
#                     self.calcResultsOfUnitTest()
#                     self.resetUnitTestStats()
#                     self.printUnitTestStats()
                    
#                     # Set everythign back to normal
#                     self.real_lidar = True

#                     for idx, vehicle in self.vehicles.items():
#                         self.lineVehicleSpeed[idx].setText("0.0")
#                         self.lineVehicleSpeed[idx].setReadOnly(False)

#                     # Test over
#                     self.sensorsButton.setEnabled(True)
#                     self.errorButton.setEnabled(True)
#                     self.covarianceButton.setEnabled(True)
#                     self.endButton.setEnabled(True)
#                     self.pauseButton.setEnabled(False)
#                     self.startButton.setEnabled(True)
#                     self.unit_test_state = 0
#                     self.full_simulation = False
#                     self.simulate_error = False
#                     self.parameterized_covariance = False
#                     self.pause_simulation = True
#                     self.unitTestButton.setEnabled(True)
#                     self.unit_test = False
#                     self.unitTestButton.setText('Unit Test Off')
#                     self.unitTestButton.setEnabled(True)

#                     sys.exit()
#                 else:
#                     # Calculate the prior results
#                     self.calcResultsOfUnitTest()
#                     self.resetUnitTestStats()

#                     for idx, vehicle in self.vehicles.items():
#                         self.lineVehicleSpeed[idx].setText("0.5")
#                     self.full_simulation = True
#                     self.simulate_error = True
#                     self.pause_simulation = False

#                     # Set the fusion modes
#                     self.local_fusion_mode = self.unitTest[self.unit_test_idx][0]
#                     self.global_fusion_mode = self.unitTest[self.unit_test_idx][1]
#                     self.parameterized_covariance = self.unitTest[self.unit_test_idx][2]
#                     self.globalFusion = global_fusion.GlobalFUSION(self.global_fusion_mode)
#                     for idx, veh in self.vehicles.items():
#                         if veh.simVehicle:
#                             self.localFusionCAV[idx].fusion_mode = self.local_fusion_mode
#                     for idx, sens in self.cis.items():
#                         if sens.simCIS:
#                             self.localFusionCIS[idx].fusion_mode = self.local_fusion_mode

#                     # Incrememt the unit test state
#                     self.unit_test_idx += 1


#         if not self.pause_simulation:
#             if self.full_simulation:
#                     self.time += 125
#             else:
#                 self.time = round(time.time() * 1000)

#         # 8HZ
#         if self.full_simulation:
#             if (self.time - self.lastPositionUpdate) >= 125:
#                 self.lastPositionUpdate = self.time
#                 self.vehiclesLock.acquire()
#                 for key, vehicle in self.vehicles.items():
#                     # Update vehicle position based on physics
#                     if vehicle.simVehicle:
#                         vehicle.updatePosition(.125)
#                 self.vehiclesLock.release()
#                 self.drawTrafficLight = True

#         # 8HZ
#         if (self.time - self.lastLocalizationUpdate) >= 125:
#             # print ( self.time )
#             self.lastLocalizationUpdate = self.time
#             # Traffic light update sequence
#             if self.lightTime > self.mapSpecs.lightTimePeriod:
#                 self.lightTime = 0
#                 if self.trafficLightArray[1] == 2:
#                     self.trafficLightArray[1] = 1
#                     self.trafficLightArray[2] = 0
#                     lightTimePeriod = 0 * 8
#                 elif self.trafficLightArray[2] == 2:
#                     self.trafficLightArray[1] = 0
#                     self.trafficLightArray[2] = 1
#                     lightTimePeriod = 0 * 8
#                 elif self.trafficLightArray[1] == 1:
#                     self.trafficLightArray[1] = 0
#                     self.trafficLightArray[2] = 2
#                     lightTimePeriod = 5 * 8
#                 elif self.trafficLightArray[2] == 1:
#                     self.trafficLightArray[1] = 2
#                     self.trafficLightArray[2] = 0
#                     lightTimePeriod = 5 * 8
#             else:
#                 self.lightTime += 1

#             #self.vehiclesLock.acquire()

#             start_vehicles = time.time()

#             # Make the vehicle list before we move the vehicles
#             # Get the last known location of all other vehicles
#             vehicleList = []
#             for otherIdx, otherVehicle in self.vehicles.items():
#                 vehicleList.append(otherVehicle.get_location())

#             self.localizationsList = []
#             for idx, vehicle in self.vehicles.items():
#                 if self.pause_simulation:
#                     # Make sure we relay the pause to our vehicles
#                     vehicle.targetVelocityGeneral = 0.0
#                 elif self.lineVehicleSpeed[idx].text() == "" or self.lineVehicleSpeed[idx].text() == ".":
#                     # Need to pass here in case the user is still typing
#                     pass
#                 else:
#                     vehicle.targetVelocityGeneral = float(self.lineVehicleSpeed[idx].text())
#                 if vehicle.simVehicle:
#                     if self.pause_simulation:
#                         vehicle.update_localization()
#                         vehicle.distance_pid_control_overide = True
#                         vehicle.targetVelocity = 0.0
#                         vehicle.update_pid()
#                     else:
#                         if self.useThreading:
#                             self.threads[idx] = threading.Thread(target=self.updateVehicleInSim, args=(idx, self.full_simulation, vehicleList, idx))      
#                             self.threads[idx].start() # start the thread we just created
#                         else:
#                             local_differences_c, local_over_detection_miss_c, local_under_detection_miss_c, localization_error_c = self.updateVehicleInSim(idx, self.full_simulation, vehicleList, idx)
#                             self.local_under_detection_miss += local_under_detection_miss_c
#                             self.local_over_detection_miss += local_over_detection_miss_c
#                             self.local_differences += local_differences_c
#                             self.localization_differences.append(math.hypot(localization_error_c[0], localization_error_c[1]))
#                         # self.localization_velocity.append(vehicle.velocity)

#             for idx, cis in self.cis.items():
#                 #print ( " CIS:", idx )
#                 # Do this if we are not in a full sim
#                 if not self.full_simulation and cis.simCIS:
#                     # CISs should not move but we can do this anyway just in case
#                     cis.updatePosition(.125)
#                 if cis.simCIS:
#                     if self.pause_simulation:
#                         cis.update_localization()
#                     else:
#                         # Update ourself
#                         cis.update_localization()

#                         if self.useThreading:
#                             thread_num = len(self.vehicles) + idx
#                             self.threads[thread_num] = threading.Thread(target=self.updateCISInSim, args=(idx, self.full_simulation, vehicleList, thread_num))
#                             self.threads[thread_num].start() # start the thread we just created  
#                         else:
#                             local_differences_c, local_over_detection_miss_c, local_under_detection_miss_c = self.updateCISInSim(idx, self.full_simulation, vehicleList, idx)

#                             self.local_under_detection_miss += local_under_detection_miss_c
#                             self.local_over_detection_miss += local_over_detection_miss_c
#                             self.local_differences += local_differences_c

#             if self.useThreading:
#                 # wait for all threads to finish                                            
#                 for idx, t in enumerate(self.threads):                                                           
#                     t.join()
#                     if self.threads_results[idx] != None:
#                         local_differences_c, local_over_detection_miss_c, local_under_detection_miss_c, localization_error_c = self.threads_results[idx]
#                         self.local_under_detection_miss += local_under_detection_miss_c
#                         self.local_over_detection_miss += local_over_detection_miss_c
#                         self.local_differences += local_differences_c

#             #print ( "v:", time.time() - start_vehicles )

#             #self.vehiclesLock.release()

#             start_global = time.time()

#             # First we need to add the localization frame, since it should be the basis
#             self.globalFusion.processDetectionFrame(-1, self.time/1000.0, self.localizationsList, .25, self.parameterized_covariance)

#             for idx, vehicle in self.vehicles.items():
#                 # Add to the global sensor fusion
#                 self.globalFusion.processDetectionFrame(idx, self.time/1000.0, vehicle.fusionDetections, .25, self.parameterized_covariance)

#             for idx, cis in self.cis.items(): 
#                 # Add to the global sensor fusion
#                 self.globalFusion.processDetectionFrame(idx, self.time/1000.0, cis.fusionDetections, .25, self.parameterized_covariance)

#             self.globalFusionList = self.globalFusion.fuseDetectionFrame(self.parameterized_covariance)

#             # Ground truth to the original dataset
#             testSetGlobal = []
#             groundTruthGlobal = []
#             for each in self.globalFusionList:
#                 sensed_x = each[1]
#                 sensed_y = each[2]
#                 testSetGlobal.append([sensed_x, sensed_y])
#             for each in vehicleList:
#                 sensed_x = each[0]
#                 sensed_y = each[1]
#                 groundTruthGlobal.append([sensed_x, sensed_y])
#             if len(testSetGlobal) >= 1 and len(groundTruthGlobal) >= 1:
#                 nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(np.array(testSetGlobal))
#                 distances, indices = nbrs.kneighbors(np.array(groundTruthGlobal))

#                 # Now calculate the score
#                 for dist in distances:
#                     if dist > 1.0:
#                         # Too far away to be considered a match, add as a miss instead
#                         self.global_under_detection_miss += len(groundTruthGlobal) - len(testSetGlobal)
#                     else:
#                         self.global_differences.append(dist)
#             # Check how much large the test set is from the ground truth and add that as well
#             if len(testSetGlobal) > len(groundTruthGlobal):
#                 # Overdetection case
#                 self.global_over_detection_miss += len(testSetGlobal) - len(groundTruthGlobal)
#             elif len(testSetGlobal) < len(groundTruthGlobal):
#                 # Underdetection case, we count this differently because it may be from obstacle blocking
#                 self.global_under_detection_miss += len(groundTruthGlobal) - len(testSetGlobal)

#             #print ( "g:", time.time() - start_global )

#             #print ( " over misses: ", self.local_over_detection_miss, " under misses: ", self.local_under_detection_miss )
#             #print ( " over misses: ", self.global_over_detection_miss, " under misses: ", self.global_under_detection_miss )

#             self.drawTrafficLight = True

#             # if self.time == 599875:
#             #     f = open("localization.txt", "a")
#             #     for d, v in zip(self.localization_differences, self.localization_velocity):
#             #         f.write(str(v) + "," + str(d) + "\n")
#             #     f.close()
