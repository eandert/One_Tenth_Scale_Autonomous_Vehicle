import time
import requests


''' This class creates a connection with the RSU server which is set using an IP address.
We can then get gui information from that server for display.'''
class connectServer:
    # TODO: Protect everything within TLS
    def __init__(self, rsu_ip):
        self.rsu_ip_address = 'http://' + str(rsu_ip) + ':5000'

    def getGuiValues(self, coordinates):
        # data to be sent to api 
        packet = {'coordinates':coordinates} 
  
        #print("Sending:%s"%packet)
        while True:
            try:
                # sending post request
                r = requests.get(url = self.rsu_ip_address + "/RSU/guiread/", json = packet, timeout = 5) 
  
                # extracting response text
                response = r.json()
                # TODO: Verify this better

                return response
            except:
                print ( "Error: Failed to message RSU, trying again" )
                time.sleep(.01)

    def sendGuiValues(self, velocity_targets, pause, end, button_states):
        # data to be sent to api 
        packet = {'velocity_targets': velocity_targets,
                  'pause': pause,
                  'end': end,
                  'button_states': button_states} 
  
        #print("Sending:%s"%packet)
        while True:
            try:
                # sending post request
                r = requests.get(url = self.rsu_ip_address + "/RSU/guisend/", json = packet, timeout = 5) 
  
                # extracting response text
                response = r.json()
                # TODO: Verify this better

                return response
            except:
                print ( "Error: Failed to message RSU, trying again" )
                time.sleep(.01)
