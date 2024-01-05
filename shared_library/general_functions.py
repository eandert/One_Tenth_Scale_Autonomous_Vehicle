import math
import numpy as np
from sklearn.neighbors import NearestNeighbors
from shapely.geometry.polygon import Polygon
from shared_library import sensor


def ground_truth_dataset(test_list, ground_truth_list, self_id = -1):
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

def cooperative_monitoring_process_conclave(conclave_data, globaFusionListConclave, cav_pos, cis_pos, current_time, conclave_dict, error_injection_time, error_at_100, twenty_percent_error_hit):
    missed_detection_error = 3.0
    localizationid = (4 + 2) * 10000
    revolving_buffer_size = 200
    error_target_vehicle = 1

    # We have who saw what, but now we need to see who should have seen what
    object_polygons = []
    length = .6
    width = .6
    for idx, vehicle in enumerate(globaFusionListConclave):
        # Rule of 3s, make sure 3 things have seen this
        if vehicle[7] >= 3:
            # Create a bounding box for vehicle that is length + 2*buffer long and width + 2*buffer wide
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

    # For conclave
    detectors = []
    for cav in cav_pos:
        sublist_new = sensor.check_visble_objects([cav[0], cav[1], cav[2], 1.0],
            cav[3], cav[4], cav[5], object_polygons)
        sublist_new += sensor.check_visble_objects([cav[0], cav[1], cav[2], 1.0],
            cav[6], cav[7], cav[8], object_polygons)
        # Look for the same thing 2x
        seen = set()
        sublist_mid = []
        # Look for repeated detections from cam/lidar
        for x in sublist_new:
            if x[0] not in seen: 
                sublist_mid.append(x)
            seen.add(x[0])
        sublist = []
        # Check that this is not ourself -- doesn't make sense we don't know ID #s
        for each in sublist_mid:
            if each != idx:
                sublist.append(each)
        detectors.append(sublist)
    for cis in cis_pos:
        sublist = sensor.check_visble_objects([cis[0], cis[1], cis[2], .5],
           cis[3], cis[4], cis[5], object_polygons)
        detectors.append(sublist)

    # append an empty set for the localizers
    detectors.append([])

    # Conclave method
    # Add our covariance data to the global sensor list
    for object_id, object_trackers in enumerate(conclave_data):
        for error_frame in object_trackers:
            # Check if this is a localizer or a sensor
            if error_frame[0]/localizationid >= 1:
                sensor_platform_id = error_frame[0]
            else:
                sensor_platform_id = math.floor(error_frame[0]/10000)
            add_error_frame(current_time, sensor_platform_id, error_frame[1], error_frame[2], conclave_dict, revolving_buffer_size)
            
            # Check off detected objects if they exist
            if sensor_platform_id < localizationid:
                for objects_should_be_seen_id in reversed(range(len(detectors[sensor_platform_id]))):
                    if object_id == detectors[sensor_platform_id][objects_should_be_seen_id][0]:
                        detectors[sensor_platform_id].pop(objects_should_be_seen_id)

    # Add the error for missed detections
    for sensor_platform_id in conclave_dict.keys():
        # Check off what we have seen
        if sensor_platform_id < localizationid:
            for seen_obj_id in range(len(detectors[sensor_platform_id])):
                add_error_frame(current_time, sensor_platform_id, missed_detection_error, 10, conclave_dict, revolving_buffer_size)
                #print("+++++++++++++++++++++++++++++++++ adding missed detection for ", sensor_platform_id, detectors[sensor_platform_id][seen_obj_id])

    # Normalize all the data to 0 (hopefully)
    normalization_numerator = 0.0
    normalization_denominator = 0.0
    print(conclave_dict.keys())
    for key in conclave_dict.keys():
        if key <= 3:
            normalization_numerator += sum(conclave_dict[key][2])
            normalization_denominator += conclave_dict[key][0]
    
    # Make sure the fenominator is greater than 0
    if normalization_denominator != 0.0 and current_time < error_injection_time:
        error_monitoring_normalizer = normalization_numerator / normalization_denominator
        # error_monitoring_normalizer = 1.0
    else:
        error_monitoring_normalizer = 1.0

    print("normalizer", error_monitoring_normalizer)

    conclave_error = 1.0
    for key in conclave_dict.keys():
        if conclave_dict[key][0] > (revolving_buffer_size / 2.0):
            try:
                average_error_std = sum(conclave_dict[key][2])/conclave_dict[key][0]
                print(key, "avg", average_error_std)
                average_error_normalized_conclave = average_error_std / error_monitoring_normalizer
                print(key, "norm", average_error_normalized_conclave)

                if int(key) == error_target_vehicle:
                    # Write to one file the SDSS vs. baseline at 100 seconds
                    if error_at_100 == -99.0 and current_time >= error_injection_time:
                        error_at_100 = average_error_normalized_conclave
                        if error_at_100 == 0.0:
                            error_at_100 = 0.001

                    average_error_normalized_conclave = average_error_normalized_conclave / error_at_100

                    if current_time > error_injection_time:
                        conclave_error = average_error_normalized_conclave
                        with open('test_output/output_conclave.txt', 'a') as f:
                            f.write(str(average_error_normalized_conclave) + ",")
                            print("writing to file!" + str(current_time-.125) + "," + str(average_error_normalized_conclave) + "\n")

                    # Only break once the revolving buffer is full
                    if current_time > error_injection_time:
                        if conclave_error >= 1.2 and not twenty_percent_error_hit:
                            with open('test_output/twenty_percent_break_point.txt', 'a') as f:
                                f.write(str(current_time - error_injection_time) + ",")
            except:
                print("Division error")

    return conclave_error, error_at_100, twenty_percent_error_hit

def cooperative_monitoring_process_truepercept(trupercept_data, globaFusionListTrupercept, cav_pos, cis_pos, current_time, trupercept_dict, error_injection_time, error_at_100, twenty_percent_error_hit):
    missed_detection_error = 3.0
    localizationid = (4 + 2) * 10000
    trupercept_freshness = 200
    error_target_vehicle = 1
    
    object_polygons_tp = []
    length = .6
    width = .6
    for idx, vehicle in enumerate(globaFusionListTrupercept):
        # Rule of 3s, make sure 3 things have seen this
        if vehicle[7] >= 3:
            # Create a bounding box for vehicle that is length + 2*buffer long and width + 2*buffer wide
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
            object_polygons_tp.append(polygon)

    # For trupercept
    detectors_tp = []
    for cav in cav_pos:
        sublist_new = sensor.check_visble_objects([cav[0], cav[1], cav[2], 1.0],
            cav[3], cav[4], cav[5], object_polygons_tp)
        sublist_new += sensor.check_visble_objects([cav[0], cav[1], cav[2], 1.0],
            cav[6], cav[7], cav[8], object_polygons_tp)
        # Look for the same thing 2x
        seen = set()
        sublist_mid = []
        # Look for repeated detections from cam/lidar
        for x in sublist_new:
            if x[0] not in seen: 
                sublist_mid.append(x)
            seen.add(x[0])
        sublist = []
        # Check that this is not ourself -- doesn't make sense we don't know ID #s
        for each in sublist_mid:
            if each != idx:
                sublist.append(each)
        detectors_tp.append(sublist)
    for cis in cis_pos:
        sublist = sensor.check_visble_objects([cis[0], cis[1], cis[2], .5],
        cis[3], cis[4], cis[5], object_polygons_tp)
        detectors_tp.append(sublist)

    # append an empty set for the localizers
    detectors_tp.append([])

    # Trupercept
    # Add add iou data
    # [id_test, id, iou, confidence_test, confidence]
    # print(trupercept_data)
    visibility_threshold = .5
    for object_id, trupercept_object_frame in enumerate(trupercept_data):
        for detector_test in trupercept_object_frame: 
            detection_numerator = 0.0
            detection_denominator = 0.0
            if detector_test[0]/localizationid >= 1:
                sensor_platform_id = detector[0]
            else:
                vid_self = math.floor(detector_test[0]/10000)
                confidence_self = detector_test[1]
                # Algorithm 3 in trupercept paper
                for detector in trupercept_object_frame:
                    # Make sure we do not record ourself vs. ourself
                    if vid_self != detector[0]:
                        # Check if this is a localizer or a sensor
                        if detector[0]/localizationid >= 1:
                            sensor_platform_id = detector[0]
                        else:
                            sensor_platform_id = math.floor(detector[0]/10000)
                            confidence_other = detector[1]
                            visibility = -1
                            # print(object_id, sensor_platform_id)
                            # print(detectors)
                            for objects_should_be_seen_id in reversed(range(len(detectors_tp[sensor_platform_id]))):
                                if object_id == detectors_tp[sensor_platform_id][objects_should_be_seen_id][0]:
                                    visibility = detectors_tp[sensor_platform_id][objects_should_be_seen_id][1]
                            # print(vid_self, sensor_platform_id, visibility, confidence_other)
                            if visibility != -1:
                                if visibility >= visibility_threshold:
                                    detection_numerator += visibility * confidence_other
                                    detection_denominator += visibility
                if detection_denominator != 0.0 and confidence_self != 0.0:
                    detection_trust = detection_numerator / detection_denominator
                    vehicle_detection_trust_score = confidence_self * detection_trust / confidence_self
                    add_trupercept_frame(current_time, vid_self, vehicle_detection_trust_score, trupercept_dict, trupercept_freshness)

    # Add the error for missed detections
    for sensor_platform_id in trupercept_dict.keys():
        # Check off what we have seen
        if sensor_platform_id < localizationid:
            for seen_obj_id in range(len(detectors_tp[sensor_platform_id])):
                #print("+++++++++++++++++++++++++++++++++ adding missed detection for ", sensor_platform_id, detectors[sensor_platform_id][seen_obj_id])
                add_trupercept_frame(current_time, sensor_platform_id, 0, trupercept_dict, trupercept_freshness)

    # Trupercept Algorihtm 4
    truepercept_error = 1.0
    trupercept_monitoring = []
    for key in trupercept_dict.keys():
        if trupercept_dict[key][0] > (trupercept_freshness / 2.0):
            trupercept_score = sum(trupercept_dict[key][2])/trupercept_dict[key][0]
            trupercept_monitoring.append([key, trupercept_score, trupercept_dict[key][0]])

            print("avg tp", trupercept_score)

            # Write to one file the SDSS vs. baseline at 100 seconds
            if error_at_100 == -99.0 and current_time >= error_injection_time and int(key) == error_target_vehicle:
                error_at_100 = trupercept_score
                if error_at_100 == 0.0:
                    error_at_100 = 0.001

            average_error_normalized_tp = trupercept_score / error_at_100
            print("norm", average_error_normalized_tp, error_at_100)

            if current_time >= error_injection_time and int(key) == error_target_vehicle:
                if average_error_normalized_tp != 0.0:
                    truepercept_error = 1.0 / average_error_normalized_tp
                else:
                    truepercept_error = 2.0
                with open('test_output/output_tp.txt', 'a') as f:
                    f.write(str(average_error_normalized_tp) + ",")
                    print("writing to file!" + str(current_time-.125) + "," + str(average_error_normalized_tp) + "\n")

            # Only break once the revolving buffer is full
            if current_time >= error_injection_time and int(key) == error_target_vehicle:
                if average_error_normalized_tp <= 0.8 and not twenty_percent_error_hit:
                    with open('test_output/twenty_percent_break_point_tp.txt', 'a') as f:
                        f.write(str(current_time - error_injection_time) + ",")
                    twenty_percent_error_hit = True

    return truepercept_error, error_at_100, twenty_percent_error_hit

def add_error_frame(current_time, sensor_platform_id, error_std, num_error, conclave_dict, revolving_buffer_size):
    # Allow time for test warmup
    if current_time > 25.0 and num_error >= 3:
        if sensor_platform_id in conclave_dict:
            # Moving revolving_buffer_size place average
            if conclave_dict[sensor_platform_id][0] < revolving_buffer_size:
                conclave_dict[sensor_platform_id][0] += 1
                conclave_dict[sensor_platform_id][2].append(error_std)
                conclave_dict[sensor_platform_id][3].append(num_error)
                conclave_dict[sensor_platform_id][1] += 1
            # We have filled revolving_buffer_size places, time to revolve the buffer now
            else:
                if conclave_dict[sensor_platform_id][1] < revolving_buffer_size:
                    # Replace the element with the next one
                    conclave_dict[sensor_platform_id][2][conclave_dict[sensor_platform_id][1]] = error_std
                    conclave_dict[sensor_platform_id][3][conclave_dict[sensor_platform_id][1]] = num_error
                    conclave_dict[sensor_platform_id][1] += 1
                else:
                    conclave_dict[sensor_platform_id][1] = 0
                    conclave_dict[sensor_platform_id][2][conclave_dict[sensor_platform_id][1]] = error_std
                    conclave_dict[sensor_platform_id][3][conclave_dict[sensor_platform_id][1]] = num_error
                    conclave_dict[sensor_platform_id][1] += 1
        else:
            conclave_dict[sensor_platform_id] = [1, 1, [error_std], [num_error]]
    if current_time > 25.0 and num_error == 1:
        # Check for lack of detections
        add_error_frame(current_time, sensor_platform_id, 3.0, 10, conclave_dict, revolving_buffer_size)

def add_trupercept_frame(current_time, sensor_platform_id, detection_trust, trupercept_dict, trupercept_freshness):
    # Allow time for test warmup
    if current_time > 25.0:
        if sensor_platform_id in trupercept_dict:
            # Moving trupercept_freshness place average
            if trupercept_dict[sensor_platform_id][0] < trupercept_freshness:
                trupercept_dict[sensor_platform_id][0] += 1
                trupercept_dict[sensor_platform_id][1] += 1
                trupercept_dict[sensor_platform_id][2].append(detection_trust)
            # We have filled trupercept_freshness places, time to revolve the buffer now
            else:
                if trupercept_dict[sensor_platform_id][1] < trupercept_freshness:
                    # Replace the element with the next one
                    trupercept_dict[sensor_platform_id][2][trupercept_dict[sensor_platform_id][1]] = detection_trust
                    trupercept_dict[sensor_platform_id][1] += 1
                else:
                    trupercept_dict[sensor_platform_id][1] = 0
                    trupercept_dict[sensor_platform_id][2][trupercept_dict[sensor_platform_id][1]] = detection_trust
                    trupercept_dict[sensor_platform_id][1] += 1
        else:
            trupercept_dict[sensor_platform_id] = [1, 1, [detection_trust]]