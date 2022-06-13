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

    def register(self, vehicle_id, x, y, z, roll, pitch, yaw, timestamp):
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id,
                'type':0,
                'timestamp':timestamp,
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

    def checkin(self, vehicle_id, x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, intersection_id, detections, timestamp):
  
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id, 
                'type':0,
                'timestamp':timestamp,
                'x':x,
                'y':y,
                'z':z,
                'roll':roll,
                'pitch':pitch,
                'yaw':yaw,
                'steeringAcceleration':steeringAcceleration, 
                'motorAcceleration':motorAcceleration, 
                'targetIndexX':targetIndexX, 
                'targetIndexY':targetIndexY,
                'targetIntersection':intersection_id,
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
                'type':0}
  
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
            response = {'time':None}
            return response

    def sendSimPosition(self, vehicle_id, x, y, z, roll, pitch, yaw, velocity):
  
        # data to be sent to api 
        packet = {'key':self.key, 
                'id':vehicle_id,
                'type':0,
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


''' This class starts up the process that will read the LIDAR using C++.
Eddie note: The reason this is not done using Python C/C++ binding is that the
proprietary driver for the LIDAR is only compiled for 32 bit ARM and 
short of a miracle I have not been able to figure out how to get Python to
bind with a 32 bit .so file. So for now we are running a process and killing
and restarting it if there are any issues, which has been very rare luckily.'''
class connectLIDAR:
    def __init__(self, pipeFromC, pipeToC):
        self.pipeFromC = pipeFromC
        self.pipeToC =  pipeToC
        self.lidarTimeout = 1
        self.time = time.time()
        self.killMapdemo()
        self.debug =  False
        self.localizationX = 0.0
        self.localizationY = 0.0
        self.localizationYaw = 0.0
        time.sleep(1)
        try:
            os.mkfifo(pipeFromC)
        except OSError as oe:
            print ( "  Warning: pipeFromC exists, that is cool we will use it" )
        try:
            os.mkfifo(pipeToC)
        except OSError as oe:
            print ( "  Warning: pipeToC exists, that is cool we will use it" )
        self.lidarProc = self.runLIDARCode()
        time.sleep(1)
        lidarTimeout = 0
        self.connectLIDAR()

    def connectLIDAR(self):
        tries = 0
        while True:
            try:
                tries += 1
                # Now start the opening process
                toc = self.tocCheckWrapper()
                if self.debug:
                    print ( "Opened toc pipe" )
                toc.flush()
                toc.write("S")
                toc.close()
                if self.debug:
                    print ( "Wrote toc pipe" )
                fromc = self.fromcCheckWrapper()
                if self.debug:
                    print ( "Opened fromc pipe" )
                testString = self.fromcReadWrapper(fromc)
                if self.debug:
                    print ( "Wrote fromc pipe" )
                if "A" in testString:
                    fromc.close()
                    print ("Sucess, LIDAR started!")
                    # Sleep long enough for the data to start
                    time.sleep(1)
                    return
                else:
                    print (" Error: LIDAR not started.")
                    fromc.close()
            except Exception as e:
                print ( " Error: Cannot talk to the LIDAR, retrying..", str(e) )
                # Set tries to 11 so that it reconnects, probably a seg fault
                tries = 11
                time.sleep(.1)
            if tries > 10:
                self.killMapdemo()
                time.sleep(1)
                self.lidarProc = self.runLIDARCode()                 
                time.sleep(1)
                tries = 0

    # This function exist just so we can wrap it with the timout function
    @timeout_decorator.timeout(1)
    def tocCheckWrapper(self):
        return open(self.pipeToC,'w')

    # This function exist just so we can wrap it with the timout function
    @timeout_decorator.timeout(1)
    def fromcCheckWrapper(self):
        return open(self.pipeFromC,'r')

    # This function exist just so we can wrap it with the timout function
    @timeout_decorator.timeout(1)
    def fromcReadWrapper(self, fromc):
        return fromc.read()

    def runLIDARCode(self):
        cmd = "/home/jetson/Projects/slamware/slamware_sdk_linux-armv7hf-gcc4.8/linux-armv7hf-release/output/ttcomp"
        pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                           shell=True, preexec_fn=os.setsid) 
        return pro

    def killMapdemo(self):
        PROCNAME = "mapdemo"
        for proc in psutil.process_iter():
        # check whether the process name matches
            if proc.name() == PROCNAME:
                proc.kill()

    @timeout_decorator.timeout(1)
    def checkFromC(self):
        if self.debug:
            print("Opening FIFO...")
        fromc=open(self.pipeFromC,'r')
        self.time = time.time()
        self.datastore = fromc.read()
        if self.debug:
            print('Read: "{0}"'.format(self.datastore))
        fromc.close()

    @timeout_decorator.timeout(1)
    def getFromC(self):
        start = time.time()
        toc = self.tocCheckWrapper()
        if self.debug:
            print ( "Opened toc pipe" )
        toc.flush()
        toc.write("S")
        toc.close()
        if self.debug:
            print("Opening FIFO...")
        fromc=open(self.pipeFromC,'r')
        self.time = time.time()
        self.datastore = fromc.read()
        if self.debug:
            print('Read: "{0}"'.format(self.datastore))
        fromc.close()
        end = time.time()
        return start, end

    def parseFromC(self):
        reader = csv.reader(self.datastore.split('\n'), delimiter=',')
        notfirst = False
        lidarpoints = []
        try:
            for idx, row in enumerate(reader):
                if notfirst:
                    if row[0] == "1":
                        angle = float(row[1])
                        distance = float(row[2])
                        newRow = []
                        newRow.append(distance * math.cos(angle))
                        newRow.append(distance * math.sin(angle))
                        lidarpoints.append(newRow)
                else:
                    notfirst = True
                    self.localizationX = float(row[0])
                    self.localizationY = float(row[1])
                    self.localizationYaw = float(row[2]) 
        except Exception as e:
            if "out of range" not in str(e):
                print ( " Error: ", str(e) )
        return lidarpoints

    def parseFromCIdx(self, index):
        self.localizationIdx = index
        return self.parseFromC()
