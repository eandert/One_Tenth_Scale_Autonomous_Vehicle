import json
import os
import time
import math

def initial_communication(sensor_id, sensor_platform_ids_count, data):
    bosco_id = sensor_id
    # Arbitrary based on the underlying consensus we use (itc, 0.5)
    # For pure & naive homogenous voting (which is Byzantine fault-tolerant), fault tolerance is 51%
    # sensor_platform_ids_count = n messages PER NODE, as each node runs individually
    # Cumulatively across network, generating n^2 messages
    fault_tolerant_node_limit = math.floor(sensor_platform_ids_count * 0.5)
    successful_message_counter = 0

    # --- [STEP] broadcast <VOTE, V_p> to all processors -----------------------------------------
    # Create a "message" using a file for each of our other vehicles
    for platform_id in range(sensor_platform_ids_count):
        if platform_id != bosco_id:
            with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_init.txt", 'w') as f:
                json.dump(data, f, sort_keys=True)

    # Wait some arbitrary time so everyone can write their files (this is hacky)
    time.sleep(1)
    # --------------------------------------------------------------------------------------------

    # --- [STEP] wait until n-t VOTE messages have been received ---------------------------------
    # Read the messages from the other vehicles and delete after reading
    recieved_data_init = []
    for platform_id in range(sensor_platform_ids_count):
        # Break out of loop upon n - t successful receptions
        if successful_message_counter >= (sensor_platform_ids_count - fault_tolerant_node_limit):
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

def concatinated_communication(sensor_id, sensor_platform_ids_count, recieved_data_init):
    bosco_id = sensor_id
    fault_tolerant_node_limit = math.floor(sensor_platform_ids_count * 0.5)

    # --- [STEP] If more than (n+3t)/2 same, DECIDE ----------------------------------------------
    # Now send these larger data packets to all vehicles again
    for platform_id in range(sensor_platform_ids_count):
        if platform_id != bosco_id:
            with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_final.txt", 'w') as f:
                json.dump(recieved_data_init, f, sort_keys=True)

    # Wait some arbitrary time so everyone can write their files (this is hacky)
    time.sleep(1)

    # Read the messages from the other vehicles and delete after reading
    recieved_data_final = []
    for platform_id in range(sensor_platform_ids_count):
        if platform_id == bosco_id:
            recieved_data_final.append(str(recieved_data_init))
        else:
            if os.path.exists("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt"):
                with open("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt", 'r') as f:
                    recieved_data_final.append(str(json.load(f)))
                os.remove("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt")
            else:
                print("The file does not exist")

    return recieved_data_final

def bosco_decide(sensor_id, sensor_platform_ids_count, recieved_data_final):
    bosco_id = sensor_id

    # Conditions for strongly and weakly one-step
    fault_tolerant_node_limit = math.floor(sensor_platform_ids_count * 0.5)

    # Add bosco here using the values stored in recieved_data_final
    # Underlying Consensus Mechanism: Naive Voting (Byzantine Fault Tolerant)
    results_dictionary = {}
    for check_value in recieved_data_final:
        if check_value in results_dictionary:
            results_dictionary[check_value] += 1
        else:
            results_dictionary[check_value] = 1

    # Dictionary stores values with their corresponding votes. Descending order of votes
    # {key : value} = {sensor value : number of votes} 
    d_sorted = sorted(results_dictionary.items(), key=lambda kv: kv[1], reverse=True)
    strongly_one_step = d_sorted[0][1] > (sensor_platform_ids_count + 3*fault_tolerant_node_limit)/2
    weakly_one_step = d_sorted[0][1] > (sensor_platform_ids_count - fault_tolerant_node_limit)/2

    print("D_SORTED: " + str(d_sorted))

    # Check if strongly one-step
    if strongly_one_step:
        # Store the actual value
        decided_v = d_sorted[0][0]
        print(str(bosco_id) + ": Decided as STRONGLY ONE-STEP")

    # Check if weakly one-step (there is no dissension)
    elif weakly_one_step and len(d_sorted) == 1:
        decided_v = d_sorted[0][0]
        print(str(bosco_id) + ": Decided as WEAKLY ONE-STEP")

    else:
        decided_v = "invalid"

    # Instead of sending to all parties involved, we are going to send the results to the RSU (which is trusted)
    return decided_v

def underlying_bft_naive_voting_consensus(sensor_platform_ids_count, decided_v):
    fault_tolerant_node_limit = math.floor(sensor_platform_ids_count * 0.5)

    # Add bosco here using the values stored in recieved_data_final
    # Underlying Consensus Mechanism: Naive Voting (Byzantine Fault Tolerant)
    results_dictionary = {}
    for check_value in decided_v:
        if check_value in results_dictionary:
            results_dictionary[check_value] += 1
        else:
            results_dictionary[check_value] = 1

    # Dictionary stores values with their corresponding votes. Descending order of votes
    # {key : value} = {sensor value : number of votes} 
    d_sorted = sorted(results_dictionary.items(), key=lambda kv: kv[1], reverse=True)

    # Do you have at least 51% of votes agree with you?
    if(d_sorted[0][1] >= fault_tolerant_node_limit):
        confirmed_consensus_value = d_sorted[0][0]
    else:
        confirmed_consensus_value = "invalid"

    # Instead of sending to all parties involved, we are going to send the results to the RSU (which is trusted)
    return confirmed_consensus_value

# ===========================================================================================================
# Separate method to fudge sensor values to engineer a Byzantine Fault
def malicious_concatinated_communication(sensor_id, sensor_platform_ids_count, recieved_data_init):
    bosco_id = sensor_id
    fault_tolerant_node_limit = math.floor(sensor_platform_ids_count * 0.5)

    # --- [STEP] If more than (n+3t)/2 same, DECIDE ----------------------------------------------
    # Now send these larger data packets to all vehicles again
    for platform_id in range(sensor_platform_ids_count):
        if platform_id != bosco_id:
            with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_final.txt", 'w') as f:
                json.dump(recieved_data_init, f, sort_keys=True)

    # Wait some arbitrary time so everyone can write their files (this is hacky)
    time.sleep(1)

    # Read the messages from the other vehicles and delete after reading
    malicious_recieved_data_final = []
    for platform_id in range(sensor_platform_ids_count):
        if platform_id == bosco_id:
            # Artificially injected arbitrary value (69420) into sensors (nice)
            malicious_recieved_data_final.append(recieved_data_init.append(str(69420)))
        else:
            if os.path.exists("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt"):
                with open("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt", 'r') as f:
                    malicious_recieved_data_final.append(str(json.load(f)))
                os.remove("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_final.txt")
            else:
                print("The file does not exist")

    return malicious_recieved_data_final