import json
import os
import time
import math

def initial_communication(sensor_id, sensor_platform_ids, data):
    bosco_id = sensor_id
    # Arbitrary based on the underlying consensus we use (itc, 0.5)
    # For pure & naive homogenous voting (which is Byzantine fault-tolerant), fault tolerance is 51%
    fault_tolerance_level = math.floor(len(sensor_platform_ids) * 0.5)
    successful_message_counter = 0

    # --- [STEP] broadcast <VOTE, V_p> to all processors -----------------------------------------
    # Create a "message" using a file for each of our other vehicles
    for platform_id in range(sensor_platform_ids):
        if platform_id != bosco_id:
            with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_init.txt", 'w') as f:
                json.dump(data, f, sort_keys=True)

    # Wait some arbitrary time so everyone can write their files (this is hacky)
    time.sleep(1)
    # --------------------------------------------------------------------------------------------

    # --- [!!! TODO][STEP] wait until n-t VOTE messages have been received ---------------------------------
    # Read the messages from the other vehicles and delete after reading
    recieved_data_init = []
    for platform_id in range(sensor_platform_ids):
        # Break out of loop upon n - t successful receptions
        if successful_message_counter >= len(sensor_platform_ids) - fault_tolerance_level:
            break
        if platform_id == bosco_id:
            recieved_data_init.append(data)
            successful_message_counter += 1
        else:
            if os.path.exists("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt"):
                with open("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt", 'r') as f:
                    recieved_data_init.append(json.load(f))
                os.remove("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt")
            else:
                print("The file does not exist")

    return recieved_data_init
    # --------------------------------------------------------------------------------------------

def concatinated_communication(sensor_id, sensor_platform_ids, recieved_data_init):
    bosco_id = sensor_id
    fault_tolerance_level = math.floor(len(sensor_platform_ids) * 0.5)
    consensus_strength_counter = 0
    decided_v_value = 0

    strongly_one_step = consensus_strength_counter > (len(sensor_platform_ids) + 3*fault_tolerance_level)/2
    weakly_one_step = consensus_strength_counter > (len(sensor_platform_ids) - fault_tolerance_level)/2

    # --- [STEP] If more than (n+3t)/2 same, DECIDE ----------------------------------------------
    # Now send these larger data packets to all vehicles again
    for platform_id in range(sensor_platform_ids):
        if platform_id != bosco_id:
            with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_final.txt", 'w') as f:
                json.dump(recieved_data_init, f, sort_keys=True)

    # Wait some arbitrary time so everyone can write their files (this is hacky)
    time.sleep(1)

    # Read the messages from the other vehicles and delete after reading
    recieved_data_final = []
    for platform_id in range(sensor_platform_ids):
        # Check if consensus is strongly one-step (n+3t)/2
        if strongly_one_step:
            decided_v_value = platform_id
            break
        # Check if consensus is weakly one-step (n-2)
        if weakly_one_step:
            decided_v_value = platform_id
        if platform_id == bosco_id:
            consensus_strength_counter += 1
        # Only push to result list to run consensus if either strongly or weakly one-step
        if platform_id == bosco_id and (strongly_one_step or weakly_one_step):
            recieved_data_final.append(str(recieved_data_init))
        else:
            if os.path.exists("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt"):
                with open("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt", 'r') as f:
                    recieved_data_final.append(str(json.load(f)))
                os.remove("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt")
            else:
                print("The file does not exist")

    return recieved_data_final

def bosco(sensor_id, sensor_platform_ids, recieved_data_final):
    bosco_id = sensor_id

    # Add bosco here using the values stored in recieved_data_final
    # Underlying Consensus Mechanism: Naive Voting (Byzantine Fault Tolerant)
    results_dictionary = {}
    for check_value in recieved_data_final:
        if check_value in results_dictionary:
            results_dictionary[check_value] += 1
        else:
            results_dictionary[check_value] = 1

    d_sorted = sorted(results_dictionary.items(), key=lambda kv: kv[1], reverse=True)
    if(d_sorted[0][1] >= (sensor_platform_ids - math.floor(sensor_platform_ids/3))):
        bosco_output = d_sorted[0][0]
    # Not sure what this does, update and re-add
    # elif (d_sorted[0][1] > (sensor_platform_ids - self.env.get_f())/2):
    #     if len(d_sorted) > 1 and d_sorted[1][1] > (sensor_platform_ids - self.env.get_f())/2:
    #         bosco_output = recieved_data_final
    #     else:
    #         bosco_output = d_sorted[0][0]
    else:
        bosco_output = "invalid"

    # Instead of sending to all parties involved, we are going to send the results to the RSU (which is trusted)
    return bosco_output