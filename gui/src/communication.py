import time
import requests


''' This class creates a connection with the RSU server which is set using an IP address.
We can then get gui information from that server for display.'''
class connectServer:  #ORIG_NETWORKING
    # TODO: Protect everything within TLS
    def __init__(self, rsu_ip): #ORIG_NETWORKING
        self.rsu_ip_address = 'http://' + str(rsu_ip) + ':5000' #ORIG_NETWORKING

    def getGuiValues(self, coordinates): #ORIG_NETWORKING
        # data to be sent to api 
        packet = {'coordinates':coordinates}  #ORIG_NETWORKING
  
        #print("Sending:%s"%packet)
        while True: #ORIG_NETWORKING
            try: #ORIG_NETWORKING
                # sending post request
                r = requests.get(url = self.rsu_ip_address + "/RSU/guiread/", json = packet, timeout = 5)  #ORIG_NETWORKING
  
                # extracting response text
                response = r.json() #ORIG_NETWORKING
                # TODO: Verify this better

                return response #ORIG_NETWORKING
            except: #ORIG_NETWORKING
                print ( "Error: Failed to message RSU, trying again" ) #ORIG_NETWORKING
                time.sleep(.01) #ORIG_NETWORKING

    def sendGuiValues(self, velocity_targets, pause, end, button_states): #ORIG_NETWORKING
        # data to be sent to api 
        packet = {'velocity_targets': velocity_targets, #ORIG_NETWORKING
                  'pause': pause, #ORIG_NETWORKING
                  'end': end, #ORIG_NETWORKING
                  'button_states': button_states}  #ORIG_NETWORKING
  
        #print("Sending:%s"%packet)
        while True: #ORIG_NETWORKING
            try: #ORIG_NETWORKING
                # sending post request
                r = requests.get(url = self.rsu_ip_address + "/RSU/guisend/", json = packet, timeout = 5)  #ORIG_NETWORKING
  
                # extracting response text
                response = r.json() #ORIG_NETWORKING
                # TODO: Verify this better

                return response #ORIG_NETWORKING
            except: #ORIG_NETWORKING
                print ( "Error: Failed to message RSU, trying again" ) #ORIG_NETWORKING
                time.sleep(.01) #ORIG_NETWORKING
