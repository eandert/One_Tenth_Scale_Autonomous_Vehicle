#!/bin/sh
source /usr/local/bin/virtualenvwrapper.sh
workon iot
cd /home/jetson/Projects/cooperative_one_tenth_scale_connected_autonomus_vehicle/
python3 /home/jetson/Projects/cooperative_one_tenth_scale_connected_autonomus_vehicle/connected_infrastructure_sensor/src/main.py
