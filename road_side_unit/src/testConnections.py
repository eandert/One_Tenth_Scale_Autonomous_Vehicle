# This file tests the connection to the server using the same API that a vehicle would use

import requests
import time

url = 'http://192.168.1.162:5000/'

register = dict(
    key="12345678",
    vehicle_id="1",
    timestamp="0.0",
    x="0.0",
    y="0.0",
    z="0.0",
    roll="0.0",
    pitch="0.0",
    yaw="0.0"
)

# registerResponse = dict(
#     v_t=self.vehicles[vehicle_id].targetVelocity,
#     t_x=init_x,
#     t_y=init_y,
#     t_z="0.0",
#     t_roll="0.0",
#     t_pitch="0.0",
#     t_yaw=init_yaw,
#     route_x=self.mapSpecs.xCoordinates,
#     route_y=self.mapSpecs.yCoordinates,
#     route_TFL=self.mapSpecs.vCoordinates,
#     tfl_state=trafficLightArray,
#     veh_locations=vehicleList,
#     timestep=time.time()
# )

checkin = dict(
    key="12345678",
    vehicle_id="1",
    timestamp="0.0",
    x="0.0",
    y="1.0",
    z="0.0",
    roll="0.0",
    pitch="0.0",
    yaw="0.0",
    detections="null"
)

resp = requests.get(url=url + "RSU/register/", json=register)
data = resp.json() # Check the JSON Response Content documentation below
print(data)

time.sleep(1)

resp2 = requests.get(url=url + "RSU/checkin/", json=checkin)
data2 = resp2.json() # Check the JSON Response Content documentation below
print(data2)