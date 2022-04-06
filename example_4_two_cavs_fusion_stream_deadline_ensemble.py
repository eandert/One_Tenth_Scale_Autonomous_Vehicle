# Copyright 2021 Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from logging import root
from SQ import STREAMify, GRAPHify
from Clock import TTClock
from Instructions import *
from tt.Deadline import TTDeadline
from tt.PlanB import TTPlanB
# I don't understand why but shapely must be imported in this file
# or the import will fail in the included files!
from shapely.geometry import box
from shapely.affinity import rotate, translate


@STREAMify
def camera_sampler(trigger, cav_num):
    import sys, time
    sys.path.insert(0, '/content/ticktalkpython/libraries')
    import camera_recognition
    global sq_state

    if sq_state.get('camera', None) == None: #TTPYTHON
        # Setup our various camera settings
        camera_specifications = camera_recognition.Settings()
        camera_specifications.darknetPath = '/content/darknet/'
        camera_specifications.useCamera = False
        if cav_num == 0:
          camera_specifications.inputFilename = '/content/yolofiles/cav0/live_test_output.avi'
          camera_specifications.camTimeFile = '/content/yolofiles/cav0/cam_output.txt'
        else:
          camera_specifications.inputFilename = '/content/yolofiles/cav1/live_test_output.avi'
          camera_specifications.camTimeFile = '/content/yolofiles/cav1/cam_output.txt'
        camera_specifications.cameraHeight = .2
        camera_specifications.cameraAdjustmentAngle = 0.0
        camera_specifications.fps = 60
        camera_specifications.width = 1280
        camera_specifications.height = 720
        camera_specifications.flip = 2
        sq_state['camera'] = camera_recognition.Camera(camera_specifications)
    frame_read, camera_timestamp = sq_state['camera'].takeCameraFrame()

    return [frame_read, camera_timestamp, time.time()]

@SQify
def process_camera(cam_sample):
    import sys, time
    sys.path.insert(0, '/content/ticktalkpython/libraries')
    import camera_recognition
    global sq_state

    camera_frame = cam_sample[0]
    camera_timestamp = cam_sample[1]
    if sq_state.get('camera_recognition', None) == None: #TTPYTHON
        # Setup our various camera settings
        camera_specifications = camera_recognition.Settings()
        camera_specifications.darknetPath = '/content/darknet/'
        camera_specifications.useCamera = False
        camera_specifications.inputFilename = '/content/yolofiles/cav0/live_test_output.avi'
        camera_specifications.camTimeFile = '/content/yolofiles/cav0/cam_output.txt'
        camera_specifications.cameraHeight = .2
        camera_specifications.cameraAdjustmentAngle = 0.0
        camera_specifications.fps = 60
        camera_specifications.width = 1280
        camera_specifications.height = 720
        camera_specifications.flip = 2
        sq_state['camera_recognition'] = camera_recognition.ProcessCamera(camera_specifications)

    coordinates, processed_timestamp = sq_state['camera_recognition'].processCameraFrame(camera_frame, camera_timestamp)

    return [coordinates, processed_timestamp, cam_sample[2], time.time()]

@STREAMify
def lidar_sampler(trigger, cav_num):
    import json, time
    global sq_state

    if sq_state.get('lidar', None) == None: #TTPYTHON
        # LIDAR filename
        if cav_num == 0:
          lidar_file = '/content/yolofiles/cav0/lidar_output.txt'
        else:
          lidar_file = '/content/yolofiles/cav1/lidar_output.txt'
        f = open(lidar_file, 'r+')
        sq_state['lidar'] = f.readlines() #TTPYTHON
        sq_state['lidar_time_idx'] = 0 #TTPYTHON
        f.close()
    lidar_timestamp = float(sq_state['lidar'][sq_state['lidar_time_idx']])
    localization = json.loads(sq_state['lidar'][sq_state['lidar_time_idx'] + 1])
    lidar_frame = json.loads(sq_state['lidar'][sq_state['lidar_time_idx'] + 2])
    sq_state['lidar_time_idx'] += 3

    return [localization, lidar_frame, lidar_timestamp, time.time()]

@SQify
def process_lidar(lidar_package):
    import sys, time
    sys.path.insert(0, '/content/ticktalkpython/libraries')
    import lidar_recognition, sensor
    global sq_state

    localization = lidar_package[0]
    lidar_frame = lidar_package[1]
    lidar_timestamp = lidar_package[2]
    if sq_state.get('lidar_recognition', None) == None: #TTPYTHON
        sq_state['lidar_recognition'] = lidar_recognition.LIDAR(lidar_timestamp)
        sq_state['lidarsensor'] = sensor.Sensor("M1M1", 0.0, 360, 15.0,
                                               0, .05, .05, .083)
        lidarcoordinates, lidartimestamp = sq_state['lidar_recognition'].processLidarFrame(lidar_frame,
                                                                                lidar_timestamp,
                                                                                localization[0],
                                                                                localization[1],
                                                                                localization[2],
                                                                                sq_state['lidarsensor'])
    else:
        lidarcoordinates, lidartimestamp = sq_state['lidar_recognition'].processLidarFrame(lidar_frame,
                                                                                lidar_timestamp,
                                                                                localization[0],
                                                                                localization[1],
                                                                                localization[2],
                                                                                sq_state['lidarsensor'])

    return [localization, lidarcoordinates, lidar_timestamp, lidar_package[3], time.time()]

@SQify
def local_fusion(processed_camera, processed_lidar, cav_num):
    import sys, time
    sys.path.insert(0, '/content/ticktalkpython/libraries')
    import local_fusion, planning_control, shared_math
    global sq_state

    # Positional offsets for the vehicles since lidar inits to 0,0,0.
    localization_offsets = [[-.75,0.0,0.], [-1.5,0.0,0.]] #BOTH
    localization = processed_lidar[0] #BOTH
    localization[0] += localization_offsets[cav_num][0] #BOTH
    localization[1] += localization_offsets[cav_num][1] #BOTH
    localization[2] += localization_offsets[cav_num][2] #BOTH

    lidar_output = processed_lidar[1] #TTPYTHON
    lidar_timestamp = processed_lidar[2] #TTPYTHON
    cam_output = processed_camera[0] #TTPYTHON
    camera_timestamp = processed_camera[1] #TTPYTHON
    if sq_state.get('fusion', None) == None: #TTPYTHON
        # Fusion node
        sq_state['fusion'] = local_fusion.FUSION(0, 0) #BOTH
        # Planner node
        sq_state['planner'] = planning_control.Planner() #BOTH
    fusion_result = [] #BOTH
    sq_state['fusion'].processDetectionFrame(local_fusion.CAMERA, camera_timestamp, cam_output, .25, 1) #BOTH
    sq_state['fusion'].processDetectionFrame(local_fusion.LIDAR, lidar_timestamp, lidar_output, .25, 1) #BOTH
    results = sq_state['fusion'].fuseDetectionFrame(1, sq_state['planner']) #BOTH

    # Convert to world coordinate system and redy for fusion
    fusion_result = [] #BOTH
    for each in results: #BOTH
        new = shared_math.rotate((0, 0), (float(each[1]), float(each[2])), float(localization[2])) #BOTH
        sensed_x = new[0] + localization[0] #BOTH
        sensed_y = new[1] + localization[1] #BOTH
        fusion_result.append((sensed_x, sensed_y, each[3], each[4], each[5], each[0])) #BOTH

    time_watch = [processed_camera[2], processed_lidar[3], processed_camera[3], processed_lidar[4], time.time()] 
    return [fusion_result, camera_timestamp, localization, time_watch] #BOTH

@SQify
def global_fusion(fusion_result_cav0, fusion_result_cav1):
    import sys, time, numpy
    sys.path.insert(0, '/content/ticktalkpython/libraries')
    import global_fusion
    global sq_state

    cav0_fusion = fusion_result_cav0[0] #TTPYTHON
    cav0_timestamp = fusion_result_cav0[1] #TTPYTHON
    cav1_fusion = fusion_result_cav1[0] #TTPYTHON
    cav1_timestamp = fusion_result_cav1[1] #TTPYTHON
    if sq_state.get('global_fusion', None) == None: #TTPYTHON
        # Fusion node
        sq_state['global_fusion'] = global_fusion.GlobalFUSION(0) #BOTH
    # add our own positions
    localization = [] #BOTH
    localization.append((fusion_result_cav0[2][0], fusion_result_cav0[2][1],
      numpy.array([[.2,0],[0,.2]]), 0, 0, -1)) #BOTH
    localization.append((fusion_result_cav1[2][0], fusion_result_cav1[2][1],
      numpy.array([[.2,0],[0,.2]]), 0, 0, -1)) #BOTH
    fusion_result = [] #BOTH
    sq_state['global_fusion'].processDetectionFrame(0, cav0_timestamp, localization, .25, 1) #BOTH
    sq_state['global_fusion'].processDetectionFrame(0, cav0_timestamp, cav0_fusion, .25, 1) #BOTH
    sq_state['global_fusion'].processDetectionFrame(1, cav1_timestamp, cav1_fusion, .25, 1) #BOTH
    fusion_result = sq_state['global_fusion'].fuseDetectionFrame(1) #BOTH

    # Convert to redable format
    results = [] #BOTH
    for each in fusion_result: #BOTH
        sensed_x = each[1] #BOTH
        sensed_y = each[2] #BOTH
        results.append([sensed_x, sensed_y]) #BOTH

    return [results, cav0_timestamp, cav1_timestamp, fusion_result_cav0[3], fusion_result_cav1[3], time.time()] #BOTH

@SQify
def deadline_check(): #TTPYTHON
    fusion_result = None #TTPYTHON

@SQify
def write_to_file(fusion_result):
    # Output filename
    import time
    outfile = "/content/ticktalkpython/output/example_4_output.txt"
    with open(outfile, 'a') as file:
        for cav in fusion_result:
            file.write(str(fusion_result) + "\n")
        print("Processed global fusion @ ", time.time())
    return 1

@SQify
def drop_nth_data(data, nth):
    from Empty import TTEmpty

    global sq_state
    if sq_state.get('count', None) == None:
        sq_state['count'] = 0

    sq_state['count'] += 1

    if sq_state['count'] == nth:
        sq_state['count'] = 0
        return TTEmpty()

    return data

@GRAPHify
def streamify_test(trigger): #TTPYTHON
    A_1 = 1 #TTPYTHON
    with TTClock.root() as root_clock: #TTPYTHON
        # Workaround
        # collect a timestamp from a clock; needs a trigger whose arrival will make the timestamp be taken. This is for setting the start-tick of the STREAMify's periodic firing rule
        start_time = READ_TTCLOCK(trigger, TTClock=root_clock) #TTPYTHON
        N = 30 #TTPYTHON
        # Setup the stop-tick of the STREAMify's firing rule
        stop_time = start_time + (2500000 * N) # sample for N seconds #TTPYTHON

        # create a sampling interval by copying the start and stop tick from token values to the token time interval
        sampling_time = VALUES_TO_TTTIME(start_time, stop_time) #TTPYTHON

        # copy the sampling interval to the input values to the STREAMify node; these input values will be treated as sticky tokens, and define the duration over which STREAMify'd nodes must run
        sample_window = COPY_TTTIME(A_1, sampling_time) #TTPYTHON

        # Cav 0
        with TTConstraint(name="cav0"): #TTPYTHON
            cav_0 = 0
            sample_time = READ_TTCLOCK(cam_sample, TTClock=root_clock) + 1500000 #TTPYTHON
            cam_sample = camera_sampler(cav_0, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #TTPYTHON
            lidar_sample = lidar_sampler(cav_0, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #TTPYTHON
            cam_output = process_camera(cam_sample) #BOTH
            lidar_output = process_lidar(lidar_sample) #BOTH
            fusion_result = local_fusion(cam_output, lidar_output, cav_0) #BOTH
            fusion_result_drop = drop_nth_data(fusion_result, 5) #SPECIAL_UNCOUNTED
            actuation_commands = TTFinishByOtherwise(global_fusion_result, TTTimeDeadline=sample_time, TTPlanB=deadline_check2(), TTWillContinue=True) #if deadline fails, it produces a separate value (similar to ternary operation: a = y if clock.now < t else None) #TTPYTHON
            # Update our various pieces
            planner.targetVelocityGeneral = actuation_commands[0] #BOTH
            planner.recieve_coordinate_group_commands(actuation_commands[1]) #BOTH
            planner.pure_pursuit_control() #BOTH

            # Now update our current PID with respect to other vehicles
            planner.check_positions_of_other_vehicles_adjust_velocity(actuation_commands[2]) #BOTH
            # We can't update the PID controls until after all positions are known
            planner.update_pid() #BOTH
            
            # Finally, issue the commands to the motors
            steering_ppm, motor_pid = planner.return_command_package() #BOTH
            if not config.simulation: #BOTH
                egoVehicle.setControlMotors(steering_ppm, motor_pid) #BOTH

        # CAV 1
        with TTConstraint(name="cav1"): #TTPYTHON
            cav_1 = 1
            sample_time2 = READ_TTCLOCK(cam_sample, TTClock=root_clock) + 1500000 #TTPYTHON
            cam_sample2 = camera_sampler(cav_1, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #TTPYTHON
            lidar_sample2 = lidar_sampler(cav_1, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #TTPYTHON
            cam_output2 = process_camera(cam_sample2) #BOTH
            lidar_output2 = process_lidar(lidar_sample2) #BOTH
            fusion_result2 = local_fusion(cam_output2, lidar_output2, cav_1) #BOTH
            actuation_commands = TTFinishByOtherwise(global_fusion_result, TTTimeDeadline=sample_time2, TTPlanB=deadline_check2(), TTWillContinue=True) #if deadline fails, it produces a separate value (similar to ternary operation: a = y if clock.now < t else None) #TTPYTHON
            # Update our various pieces
            planner.targetVelocityGeneral = actuation_commands[0] #BOTH
            planner.recieve_coordinate_group_commands(actuation_commands[1]) #BOTH
            planner.pure_pursuit_control() #BOTH

            # Now update our current PID with respect to other vehicles
            planner.check_positions_of_other_vehicles_adjust_velocity(actuation_commands[2]) #BOTH
            # We can't update the PID controls until after all positions are known
            planner.update_pid() #BOTH
            
            # Finally, issue the commands to the motors
            steering_ppm, motor_pid = planner.return_command_package() #BOTH
            if not config.simulation: #BOTH
                egoVehicle.setControlMotors(steering_ppm, motor_pid) #BOTH

        # Global fusion
        with TTConstraint(name="rsu"): #TTPYTHON
            fusion_result_deadline = TTFinishByOtherwise(fusion_result_drop, TTTimeDeadline=sample_time, TTPlanB=deadline_check(), TTWillContinue=True) #if deadline fails, it produces a separate value (similar to ternary operation: a = y if clock.now < t else None) #TTPYTHON
            fusion_result_deadline2 = TTFinishByOtherwise(fusion_result2, TTTimeDeadline=sample_time, TTPlanB=deadline_check(), TTWillContinue=True) #if deadline fails, it produces a separate value (similar to ternary operation: a = y if clock.now < t else None) #TTPYTHON
            global_fusion_result = global_fusion(fusion_result_deadline, fusion_result_deadline2) #BOTH
            result = write_to_file(global_fusion_result) #BOTH
