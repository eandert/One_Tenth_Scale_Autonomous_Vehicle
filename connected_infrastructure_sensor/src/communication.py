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
class connectServer: #ORIG_NETWORKING
    # TODO: Protect everything within TLS
    # RECEIVE: pk_RSU, symk_session
    # SEND: pk_CAV
    def __init__(self, rsu_ip): #ORIG_NETWORKING
        self.key = secrets.token_urlsafe(32) #ORIG_NETWORKING
        #self.rsu_ip_address = 'http://192.168.100.198:5000'
        self.rsu_ip_address = 'http://' + str(rsu_ip) + ':5000' #ORIG_NETWORKING

    def register(self, vehicle_id, x, y, z, roll, pitch, yaw): #ORIG_NETWORKING
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
                'yaw':yaw}  #ORIG_NETWORKING
  
        #print("Sending:%s"%packet)
        while True: #ORIG_NETWORKING
            try: #ORIG_NETWORKING
                # sending post request
                r = requests.get(url = self.rsu_ip_address + "/RSU/register/", json = packet, timeout = 5)  #ORIG_NETWORKING
  
                # extracting response text
                response = r.json() #ORIG_NETWORKING
                # TODO: Verify this better

                return response #ORIG_NETWORKING
            except: #ORIG_NETWORKING
                print ( "Error: Failed to message RSU, trying again" ) #ORIG_NETWORKING
                time.sleep(.01) #ORIG_NETWORKING

    def checkin(self, vehicle_id, x, y, z, roll, pitch, yaw, detections): #ORIG_NETWORKING
  
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
                'detections':detections} #ORIG_NETWORKING
  
        try: #ORIG_NETWORKING
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/checkin/", json = packet, timeout = 1) #ORIG_NETWORKING
            # extracting response text
            response = r.json() #ORIG_NETWORKING

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            print ( "Timeout! TODO: add fallback option" + str(e) ) #ORIG_NETWORKING
            response = None #ORIG_NETWORKING

    def getSimPositions(self, vehicle_id): #ORIG_NETWORKING
  
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id, 
                'type':1} #ORIG_NETWORKING
  
        try: #ORIG_NETWORKING
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/getsimpositions/", json = packet, timeout = 1) #ORIG_NETWORKING
            # extracting response text
            response = r.json() #ORIG_NETWORKING

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            print ( "Timeout! TODO: add fallback option" + str(e) ) #ORIG_NETWORKING
            response = {} #ORIG_NETWORKING

    def getSimTime(self): #ORIG_NETWORKING
  
        # data to be sent to api 
        packet = {} #ORIG_NETWORKING
  
        try: #ORIG_NETWORKING
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/getsimtime/", json = packet, timeout = 1) #ORIG_NETWORKING
            # extracting response text
            response = r.json() #ORIG_NETWORKING

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            print ( "Timeout! TODO: add fallback option" + str(e) ) #ORIG_NETWORKING
            response = {'time':-99} #ORIG_NETWORKING
            return response #ORIG_NETWORKING

    def sendSimPosition(self, vehicle_id, x, y, z, roll, pitch, yaw, velocity): #ORIG_NETWORKING
  
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
                'velocity':velocity} #ORIG_NETWORKING
  
        try: #ORIG_NETWORKING
            # sending post request
            r = requests.get(url = self.rsu_ip_address + "/RSU/sendsimposition/", json = packet, timeout = 1) #ORIG_NETWORKING
            # extracting response text
            response = r.json() #ORIG_NETWORKING

            # TODO: Verify this better
            #print("The response is:%s"%response)

            return response #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            print ( "Timeout! TODO: add fallback option" + str(e) ) #ORIG_NETWORKING
            response = None #ORIG_NETWORKING
