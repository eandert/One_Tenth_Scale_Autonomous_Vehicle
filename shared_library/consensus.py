import json
import os
import time
import math


def initial_communication(sensor_id, sensor_platform_ids, data):
    bosco_id = sensor_id

    # Create a "message" using a file for each of our other vehicles
    for platform_id in range(sensor_platform_ids):
        if platform_id != bosco_id:
            with open("comms_folder/" + str(platform_id) + "_" + str(bosco_id) + "_init.txt", 'w') as f:
                json.dump(data, f, sort_keys=True)

    # Wait some arbitrary time so everyone can write their files (this is hacky)
    time.sleep(1)

    # Read the messages from the other vehicles and delete after reading
    recieved_data_init = []
    for platform_id in range(sensor_platform_ids):
        if platform_id == bosco_id:
            recieved_data_init.append(data)
        else:
            if os.path.exists("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt"):
                with open("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt", 'r') as f:
                    recieved_data_init.append(json.load(f))
                os.remove("comms_folder/" + str(bosco_id) + "_" + str(platform_id) + "_init.txt")
            else:
                print("The file does not exist")

    return recieved_data_init

def concatinated_communication(sensor_id, sensor_platform_ids, recieved_data_init):
    bosco_id = sensor_id

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

def bosco(sensor_id, sensor_platform_ids, recieved_data_final):
    bosco_id = sensor_id

    # Add bosco here using the values stored in recieved_data_final
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