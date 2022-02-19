import socket
import time
import sys
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
import timeout_decorator


''' A utility function for determining our IP address for printing or watnot via python.'''
# import fcntl
# def get_ip_address(ifname):
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     return socket.inet_ntoa(fcntl.ioctl(
#         s.fileno(),
#         0x8915,
#         struct.pack('256s', ifname[:15])
#     )[20:24])


''' This class creates a connection with the RSU server which is set using an IP address.
The hopse is that one day this will be more automated but for now the IP must be set manually.
The communication backend is Flask and timouts are set here so that we do not drive
infinitely into a wall if a connection times out or a response is not received.'''
class connectServer:
    # TODO: Protect everything within TLS
    # RECEIVE: pk_RSU, symk_session
    # SEND: pk_CAV
    def __init__(self, rsu_ip):
        self.key = secrets.token_urlsafe(32)
        #self.rsu_ip_address = 'http://192.168.100.198:5000'
        self.rsu_ip_address = 'http://' + str(rsu_ip) + ':5000'

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
                time.sleep(.01)

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
                'steeringAcceleration':0.0, 
                'motorAcceleration':0.0, 
                'targetIndexX':0, 
                'targetIndexY':0,
                'detections':detections}
  
        try:
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/checkin/", json = packet, timeout = 1)
            # extracting response text
            response = r.json()

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response
        except Exception as e:
            print ( "Timeout! TODO: add fallback option" + str(e) )
            response = None

    def getSimPositions(self, vehicle_id):
  
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id, 
                'type':1}
  
        try:
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/getsimpositions/", json = packet, timeout = 1)
            # extracting response text
            response = r.json()

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response
        except Exception as e:
            print ( "Timeout! TODO: add fallback option" + str(e) )
            response = {}

    def getSimTime(self):
  
        # data to be sent to api 
        packet = {}
  
        try:
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/getsimtime/", json = packet, timeout = 1)
            # extracting response text
            response = r.json()

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response
        except Exception as e:
            print ( "Timeout! TODO: add fallback option" + str(e) )
            response = {'time':-99}
            return response

    def sendSimPosition(self, vehicle_id, x, y, z, roll, pitch, yaw, velocity):
  
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id,
                'type':1,
                'x':x,
                'y':y,
                'z':z,
                'roll':roll,
                'pitch':pitch,
                'yaw':yaw,
                'velocity':velocity}
  
        try:
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/sendsimposition/", json = packet, timeout = 1)
            # extracting response text
            response = r.json()

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response
        except Exception as e:
            print ( "Timeout! TODO: add fallback option" + str(e) )
            response = None
