import socket
import time
import sys
import fcntl
import struct
import math
import os
import signal
import subprocess
import psutil
import secrets
import requests
from io import StringIO
import csv

# Start importing cryptographic libraries
import hashlib


def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,
        struct.pack('256s', ifname[:15])
    )[20:24])

#TODO: Protect everything within TLS
    # RECEIVE: pk_RSU, symk_session
    # SEND: pk_CAV
class connectServer:
    def __init__(self):
        self.key = secrets.token_urlsafe(32)
        self.rsu_ip_address = 'http://192.168.1.162:5000'

    def register(self, vehicle_id, x, y, z, roll, pitch, yaw):
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id,
                'type':1,
                'timestamp':time.time(),
                'x':x,
                'y':y,
                'z':z,
                'roll':roll,
                'pitch':pitch,
                'yaw':yaw} 
  
        #print("Sending:%s"%packet)
        while True:
            try:
                # sending post request
                r = requests.get(url = self.rsu_ip_address + "/RSU/register/", json = packet, timeout = 5) 
  
                # extracting response text
                response = r.json()
                # TODO: Verify this better

                return response
            except:
                print ( "Error: Failed to message RSU, trying again" )

    def checkin(self, vehicle_id, x, y, z, roll, pitch, yaw, detections):
  
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id,
                'type':1,
                'timestamp':time.time(),
                'x':x,
                'y':y,
                'z':z,
                'roll':roll,
                'pitch':pitch,
                'yaw':yaw,
                'detections':detections}
  
        try:
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/checkin/", json = packet, timeout = .1)
            # extracting response text
            response = r.json()

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response
        except requests.exceptions.Timeout as e:
            print ( "Timeout! TODO: add fallback option" + str(e) )
            response = None
