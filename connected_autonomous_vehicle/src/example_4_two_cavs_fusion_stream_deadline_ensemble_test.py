# Copyright 2021 Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from logging import root #APP
from SQ import STREAMify, GRAPHify #APP
from Clock import TTClock #APP
from Instructions import * #APP
# I don't understand why but shapely must be imported in this file
# or the import will fail in the included files!
from shapely.geometry import box #APP
from shapely.affinity import rotate, translate #APP
from Empty import TTEmpty #APP


@STREAMify #TTS
def camera_sampler(trigger, cav_num): #APP
    import sys, time
    sys.path.insert(0, '/content/ticktalkpython/libraries') #APP
    import camera_recognition #APP
    global sq_state #APP

    if sq_state.get('camera', None) == None: #APP
        # Setup our various camera settings
        camera_specifications = camera_recognition.Settings() #APP
        camera_specifications.darknetPath = '/content/darknet/' #APP
        camera_specifications.useCamera = False #APP
        if cav_num == 0: #APP
          camera_specifications.inputFilename = '/content/yolofiles/cav0/live_test_output.avi' #APP
          camera_specifications.camTimeFile = '/content/yolofiles/cav0/cam_output.txt' #APP
        else: #APP
          camera_specifications.inputFilename = '/content/yolofiles/cav1/live_test_output.avi' #APP
          camera_specifications.camTimeFile = '/content/yolofiles/cav1/cam_output.txt' #APP
        camera_specifications.cameraHeight = .2 #APP
        camera_specifications.cameraAdjustmentAngle = 0.0 #APP
        camera_specifications.fps = 60 #APP
        camera_specifications.width = 1280 #APP
        camera_specifications.height = 720 #APP
        camera_specifications.flip = 2 #APP
        sq_state['camera'] = camera_recognition.Camera(camera_specifications) #APP
    frame_read, camera_timestamp = sq_state['camera'].takeCameraFrame() #APP

    return [frame_read, camera_timestamp, time.time()] #APP

@SQify #TTS
def process_camera(cam_sample): #APP
    import sys, time #APP
    sys.path.insert(0, '/content/ticktalkpython/libraries') #APP
    import camera_recognition #APP
    global sq_state #APP

    camera_frame = cam_sample[0] #APP
    camera_timestamp = cam_sample[1] #APP
    if sq_state.get('camera_recognition', None) == None: #APP
        # Setup our various camera settings
        camera_specifications = camera_recognition.Settings() #APP
        camera_specifications.darknetPath = '/content/darknet/' #APP
        camera_specifications.useCamera = False #APP
        camera_specifications.inputFilename = '/content/yolofiles/cav0/live_test_output.avi' #APP
        camera_specifications.camTimeFile = '/content/yolofiles/cav0/cam_output.txt' #APP
        camera_specifications.cameraHeight = .2 #APP
        camera_specifications.cameraAdjustmentAngle = 0.0 #APP
        camera_specifications.fps = 60 #APP
        camera_specifications.width = 1280 #APP
        camera_specifications.height = 720 #APP
        camera_specifications.flip = 2 #APP
        sq_state['camera_recognition'] = camera_recognition.ProcessCamera(camera_specifications) #APP

    coordinates, processed_timestamp = sq_state['camera_recognition'].processCameraFrame(camera_frame, camera_timestamp) #APP

    return [coordinates, processed_timestamp, cam_sample[2], time.time()] #APP

@STREAMify #TTS
def lidar_sampler(trigger, cav_num): #APP
    import json, time #APP
    global sq_state #APP

    if sq_state.get('lidar', None) == None: #APP
        # LIDAR filename
        if cav_num == 0: #APP
          lidar_file = '/content/yolofiles/cav0/lidar_output.txt' #APP
        else: #APP
          lidar_file = '/content/yolofiles/cav1/lidar_output.txt' #APP
        f = open(lidar_file, 'r+') #APP
        sq_state['lidar'] = f.readlines() #APP
        sq_state['lidar_time_idx'] = 0 #APP
        f.close() #APP
    lidar_timestamp = float(sq_state['lidar'][sq_state['lidar_time_idx']]) #APP
    localization = json.loads(sq_state['lidar'][sq_state['lidar_time_idx'] + 1]) #APP
    lidar_frame = json.loads(sq_state['lidar'][sq_state['lidar_time_idx'] + 2]) #APP
    sq_state['lidar_time_idx'] += 3 #APP

    return [localization, lidar_frame, lidar_timestamp, time.time()] #APP

@SQify #TTS
def process_lidar(lidar_package): #APP
    import sys, time #APP
    sys.path.insert(0, '/content/ticktalkpython/libraries') #APP
    import lidar_recognition, sensor #APP
    global sq_state #APP

    localization = lidar_package[0] #APP
    lidar_frame = lidar_package[1] #APP
    lidar_timestamp = lidar_package[2] #APP
    if sq_state.get('lidar_recognition', None) == None: #APP
        sq_state['lidar_recognition'] = lidar_recognition.LIDAR(lidar_timestamp) #APP
        sq_state['lidarsensor'] = sensor.Sensor("M1M1", 0.0, 360, 15.0, #APP
                                               0, .05, .05, .083)
        lidarcoordinates, lidartimestamp = sq_state['lidar_recognition'].processLidarFrame(lidar_frame, #APP
                                                                                lidar_timestamp,
                                                                                localization[0],
                                                                                localization[1],
                                                                                localization[2],
                                                                                sq_state['lidarsensor'])
    else: #APP
        lidarcoordinates, lidartimestamp = sq_state['lidar_recognition'].processLidarFrame(lidar_frame, #APP
                                                                                lidar_timestamp,
                                                                                localization[0],
                                                                                localization[1],
                                                                                localization[2],
                                                                                sq_state['lidarsensor'])

    return [localization, lidarcoordinates, lidar_timestamp, lidar_package[3], time.time()] #APP

@SQify #TTS
def local_fusion(processed_camera, processed_lidar, cav_num): #APP
    import sys, time #APP
    from Empty import TTEmpty #APP
    sys.path.insert(0, '/content/ticktalkpython/libraries') #APP
    import local_fusion, planning_control, shared_math #APP
    global sq_state #APP

    # Positional offsets for the vehicles since lidar inits to 0,0,0.
    localization_offsets = [[-.75,0.0,0.], [-1.5,0.0,0.]] #APP
    localization = processed_lidar[0] #APP
    localization[0] += localization_offsets[cav_num][0] #APP
    localization[1] += localization_offsets[cav_num][1] #APP
    localization[2] += localization_offsets[cav_num][2] #APP

    lidar_output = processed_lidar[1] #APP
    lidar_timestamp = processed_lidar[2] #APP
    cam_output = processed_camera[0] #APP
    camera_timestamp = processed_camera[1] #APP
    if sq_state.get('fusion', None) == None: #APP
        # Fusion node
        sq_state['fusion'] = local_fusion.FUSION(0, 0) #APP
        # Planner node
        sq_state['planner'] = planning_control.Planner() #APP
        sq_state['iterations'] = 0 #APP
    fusion_result = [] #APP
    sq_state['fusion'].processDetectionFrame(local_fusion.CAMERA, camera_timestamp, cam_output, .25, 1) #APP
    sq_state['fusion'].processDetectionFrame(local_fusion.LIDAR, lidar_timestamp, lidar_output, .25, 1) #APP
    results = sq_state['fusion'].fuseDetectionFrame(1, sq_state['planner']) #APP

    # Convert to world coordinate system and redy for fusion
    fusion_result = [] #APP
    for each in results: #APP
        new = shared_math.rotate((0, 0), (float(each[1]), float(each[2])), float(localization[2])) #APP
        sensed_x = new[0] + localization[0] #APP
        sensed_y = new[1] + localization[1] #APP
        fusion_result.append((sensed_x, sensed_y, each[3], each[4], each[5], each[0])) #APP

    time_watch = [processed_camera[2], processed_lidar[3], processed_camera[3], processed_lidar[4], time.time()] #APP
    return [fusion_result, camera_timestamp, localization, time_watch] #APP

@SQify #TTS
def global_fusion(fusion_result_cav0, fusion_result_cav1): #APP
    import sys, time, numpy #APP
    sys.path.insert(0, '/content/ticktalkpython/libraries') #APP
    import global_fusion #APP
    global sq_state #APP

    cav0_fusion = fusion_result_cav0[0] #APP
    cav0_timestamp = fusion_result_cav0[1] #APP
    cav1_fusion = fusion_result_cav1[0] #APP
    cav1_timestamp = fusion_result_cav1[1] #APP
    if sq_state.get('global_fusion', None) == None: #APP
        # Fusion node
        sq_state['global_fusion'] = global_fusion.GlobalFUSION(0) #APP
    # add our own positions
    localization = [] #APP
    localization.append((fusion_result_cav0[2][0], fusion_result_cav0[2][1], #APP
      numpy.array([[.2,0],[0,.2]]), 0, 0, -1))
    localization.append((fusion_result_cav1[2][0], fusion_result_cav1[2][1], #APP
      numpy.array([[.2,0],[0,.2]]), 0, 0, -1))
    fusion_result = [] #APP
    sq_state['global_fusion'].processDetectionFrame(0, cav0_timestamp, localization, .25, 1) #APP
    sq_state['global_fusion'].processDetectionFrame(0, cav0_timestamp, cav0_fusion, .25, 1) #APP
    sq_state['global_fusion'].processDetectionFrame(1, cav1_timestamp, cav1_fusion, .25, 1) #APP
    fusion_result = sq_state['global_fusion'].fuseDetectionFrame(1) #APP

    # Convert to redable format
    results = [] #APP
    for each in fusion_result: #APP
        sensed_x = each[1] #APP
        sensed_y = each[2] #APP
        results.append([sensed_x, sensed_y]) #APP

    return [results, cav0_timestamp, cav1_timestamp, fusion_result_cav0[3], fusion_result_cav1[3], time.time()] #APP

@SQify #TTS
def deadline_check():
    fusion_result = []

@SQify #TTS
def write_to_file(fusion_result): #APP
    # Output filename
    import time #APP
    outfile = "/content/ticktalkpython/output/example_4_output.txt" #APP
    with open(outfile, 'a') as file: #APP
        for cav in fusion_result: #APP
            file.write(str(fusion_result) + "\n") #APP
        print("Processed global fusion @ ", time.time()) #APP
    return 1 #APP

@GRAPHify #TTS
def streamify_test(trigger): #APP
    A_1 = 1 #APP
    with TTClock.root() as root_clock: #APP
        # Workaround
        # collect a timestamp from a clock; needs a trigger whose arrival will make the timestamp be taken. This is for setting the start-tick of the STREAMify's periodic firing rule
        start_time = READ_TTCLOCK(trigger, TTClock=root_clock) #TTS
        N = 30 #TTS
        # Setup the stop-tick of the STREAMify's firing rule
        stop_time = start_time + (2500000 * N) #TTS # sample for N seconds

        # create a sampling interval by copying the start and stop tick from token values to the token time interval
        sampling_time = VALUES_TO_TTTIME(start_time, stop_time) #TTS

        # copy the sampling interval to the input values to the STREAMify node; these input values will be treated as sticky tokens, and define the duration over which STREAMify'd nodes must run
        sample_window = COPY_TTTIME(A_1, sampling_time) #TTS

        # Cav 0
        with TTConstraint(name="cav0"): #TTS
            cav_0 = 0 #APP
            cam_sample = camera_sampler(cav_0, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #APP
            lidar_sample = lidar_sampler(cav_0, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #APP
            cam_output = process_camera(cam_sample) #APP
            lidar_output = process_lidar(lidar_sample) #APP
            fusion_result = local_fusion(cam_output, lidar_output, cav_0) #APP

            sample_time = READ_TTCLOCK(cam_sample, TTClock=root_clock) + 15000000 #TTS

        # CAV 1
        with TTConstraint(name="cav1"): #TTS
            cav_1 = 1 #APP
            cam_sample2 = camera_sampler(cav_1, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #APP
            lidar_sample2 = lidar_sampler(cav_1, sample_window, TTClock=root_clock, TTPeriod=2500000, TTPhase=0, TTDataIntervalWidth=1250000) #APP
            cam_output2 = process_camera(cam_sample2) #APP
            lidar_output2 = process_lidar(lidar_sample2) #APP
            fusion_result2 = local_fusion(cam_output2, lidar_output2, cav_1) #APP

        # Global fusion
        with TTConstraint(name="rsu"): #TTS
            fusion_result_deadline = TTFinishByOtherwise(fusion_result, TTTimeDeadline=sample_time, TTPlanB=deadline_check(), TTWillContinue=True)  #APP #if deadline fails, it produces a separate value (similar to ternary operation: a = y if clock.now < t else None)
            fusion_result_deadline2 = TTFinishByOtherwise(fusion_result2, TTTimeDeadline=sample_time, TTPlanB=deadline_check(), TTWillContinue=True) #APP #if deadline fails, it produces a separate value (similar to ternary operation: a = y if clock.now < t else None)
            global_fusion_result = global_fusion(fusion_result_deadline, fusion_result_deadline2) #APP
            result = write_to_file(global_fusion_result) #APP
