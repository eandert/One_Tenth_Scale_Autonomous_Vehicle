#!/usr/bin/env python3
# We have to force python 3 or flask fails miserably!

from flask import Flask, jsonify, request #ORIG_NETWORKING
from flask_restx import Api, Resource, fields #ORIG_NETWORKING
import time #ORIG_NETWORKING

import logging #ORIG_NETWORKING
log = logging.getLogger('werkzeug') #ORIG_NETWORKING
log.setLevel(logging.ERROR) #ORIG_NETWORKING

flask_app = Flask(__name__) #ORIG_NETWORKING
app = Api(app = flask_app, version='1.0', title='One Tenth Scale RSU',
          description='This is the API for the one tenth scale road side unit controller') #ORIG_NETWORKING

name_space = app.namespace('RSU', description='One Tenth Scale RSU operations') #ORIG_NETWORKING

RSUVehicleCheckinResponse = app.model('RSUVehicleCheckinResponse', {
                                'v_t': fields.Float(required=True,
                                                        description="Lets us set velocity target from the app, 0 Simulation is paused.",
                                                        example=0.1,
                                                        help="Pause cannot be blank."),
                                't_x': fields.Float(required=True,
                                                         description="Transform X shift to global map.",
                                                         example=1.01),
                                't_y': fields.Float(required=True,
                                                         description="Transform Y shift to global map.",
                                                         example=1.01),
                                't_z': fields.Float(required=True,
                                                         description="Transform Z shift to global map.",
                                                         example=1.01),
                                't_yaw': fields.Float(required=True,
                                                    description="Transform Yaw shift to global map.",
                                                    example=1.01),
                                't_pitch': fields.Float(required=True,
                                                    description="Transform Pitch shift to global map.",
                                                    example=1.01),
                                't_roll': fields.Float(required=True,
                                                    description="Transform Roll shift to global map.",
                                                    example=1.01),
                                'route_x': fields.List(fields.Float,
                                                        required=True,
                                                        description="X values of route corrdinates in order.",
                                                        example=1.01),
                                'route_y': fields.List(fields.Float,
                                                        required=True,
                                                        description="Y values of route corrdinates in order.",
                                                        example=1.01),
                                'route_TFL': fields.List(fields.Integer,
                                                        required=True,
                                                        description="TFL control values for points in route.",
                                                        example=1.01),
                                'tfl_state': fields.List(fields.Integer,
                                                        required=True,
                                                        description="State of traffic lighs.",
                                                        example=1),
                                'veh_locations': fields.List(fields.Float,
                                                        required=True,
                                                        description="Location of vehicles x.",
                                                        example=0.0),
                                'timestep': fields.Float(required=True,
                                                        description="Current timestep of RSU",
                                                        example=1.01)
}) #ORIG_NETWORKING

RSUVehicleRegisterResponse = app.model('RSUVehicleRegisterResponse', {
                                't_v': fields.Float(required=True,
                                                        description="Lets us set velocity target from the app, 0 Simulation is paused.",
                                                        example=0.1,
                                                        help="Target velocity cannot be blank."),
                                'tfl_state': fields.List(fields.Integer,
                                                        required=True,
                                                        description="State of traffic lighs.",
                                                        example=1),
                                'veh_locations': fields.List(fields.Float,
                                                         required=True,
                                                         description="Location of vehicles x.",
                                                         example=0.0),
                                'timestep': fields.Float(required=True,
                                                        description="Current timestep of RSU",
                                                        example=1.01)
}) #ORIG_NETWORKING

RSUVehicleGetSimPositions = app.model('RSUVehicleGetSimPositions', {
                                'veh_locations': fields.Float(required=True,
                                                        description="Lets us set velocity target from the app, 0 Simulation is paused.",
                                                        example=0.1,
                                                        help="Target velocity cannot be blank.")
}) #ORIG_NETWORKING

RSUVehicleGetSimTime = app.model('RSUVehicleGetSimTime', {
                                'time': fields.Float(required=True,
                                                        description="Simulation time",
                                                        example=0.1,
                                                        help="Time cannot be blank.")
}) #ORIG_NETWORKING

RSUVehicleSendSimPosition = app.model('RSUVehicleSendSimPosition', {
                                'returned': fields.Boolean(required=True,
                                                        description="Returned",
                                                        example=True,
                                                        help="Returned cannot be blank.")
}) #ORIG_NETWORKING

@name_space.route("/register/", methods=['GET']) #ORIG_NETWORKING
class MainClass(Resource): #ORIG_NETWORKING

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={}) #ORIG_NETWORKING
    @app.doc(description="This method can be called while a RSU instance is running to resgister a cav and get the coordinate transpose and route for said vehicle.") #ORIG_NETWORKING
    @app.response(200, 'Success', RSUVehicleCheckinResponse) #ORIG_NETWORKING
    def get(self): #ORIG_NETWORKING
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json() #ORIG_NETWORKING
        try: #ORIG_NETWORKING
            #print("data:", request_data)
            if request_data: #ORIG_NETWORKING
                key = request_data['key'] #ORIG_NETWORKING
                id = int(request_data['id']) #ORIG_NETWORKING
                type = int(request_data['type']) #ORIG_NETWORKING
                timestamp = float(request_data['timestamp']) #ORIG_NETWORKING
                x = float(request_data['x']) #ORIG_NETWORKING
                y = float(request_data['y']) #ORIG_NETWORKING
                z = float(request_data['z']) #ORIG_NETWORKING
                roll = float(request_data['roll']) #ORIG_NETWORKING
                pitch = float(request_data['pitch']) #ORIG_NETWORKING
                yaw = float(request_data['yaw']) #ORIG_NETWORKING

                #print("recieved")

                returnObject = flask_app.config['RSUClass'].register(key, id, type, timestamp, x, y, z, roll, pitch, yaw) #ORIG_NETWORKING

                #print("replying")

                if type == 0: #ORIG_NETWORKING
                    print ( "Registered vehicle: " + str(id) + " at " + str(timestamp) ) #ORIG_NETWORKING
                elif type == 1: #ORIG_NETWORKING
                    print ( "Registered sensor: " + str(id) + " at " + str(timestamp) ) #ORIG_NETWORKING


                return jsonify( #ORIG_NETWORKING
                    returnObject #ORIG_NETWORKING
                ) #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500") #ORIG_NETWORKING
            print ( str(e) ) #ORIG_NETWORKING


@name_space.route("/checkin/", methods=['GET']) #ORIG_NETWORKING
class MainClass(Resource): #ORIG_NETWORKING

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={}) #ORIG_NETWORKING
    @app.doc(description="This method can be called after a CAV is registered to update the position of the vehicle, log detections, and get the state of traffic lighs.") #ORIG_NETWORKING
    @app.response(200, 'Success', RSUVehicleCheckinResponse) #ORIG_NETWORKING
    def get(self): #ORIG_NETWORKING
        time1 = time.time() #ORIG_NETWORKING
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json() #ORIG_NETWORKING
        try: #ORIG_NETWORKING
            #print("data:", request_data)
            if request_data: #ORIG_NETWORKING
                key = request_data['key'] #ORIG_NETWORKING
                id = int(request_data['id']) #ORIG_NETWORKING
                type = int(request_data['type']) #ORIG_NETWORKING
                timestamp = float(request_data['timestamp']) #ORIG_NETWORKING
                x = float(request_data['x']) #ORIG_NETWORKING
                y = float(request_data['y']) #ORIG_NETWORKING
                z = float(request_data['z']) #ORIG_NETWORKING
                roll = float(request_data['roll']) #ORIG_NETWORKING
                pitch = float(request_data['pitch']) #ORIG_NETWORKING
                yaw = float(request_data['yaw']) #ORIG_NETWORKING
                steeringAcceleration = float(request_data['steeringAcceleration']) #ORIG_NETWORKING
                motorAcceleration = float(request_data['motorAcceleration']) #ORIG_NETWORKING
                targetIndexX = float(request_data['targetIndexX']) #ORIG_NETWORKING
                targetIndexY = float(request_data['targetIndexY']) #ORIG_NETWORKING
                detections = request_data['detections'] #ORIG_NETWORKING

                returnObject = flask_app.config['RSUClass'].checkinFastResponse(key, id, type, timestamp, x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, detections) #ORIG_NETWORKING

                #flask_app.config['RSUQueue'].put([key, id, type, timestamp, x, y, yaw, detections])

                if type == 0: #ORIG_NETWORKING
                    print("Vehicle: " + str(id) + " updated at " + str(timestamp)) #ORIG_NETWORKING
                elif type == 1: #ORIG_NETWORKING
                    print("Sensor: " + str(id) + " updated at " + str(timestamp)) #ORIG_NETWORKING

                #print ( "Response took: ", time.time() - time1)

                return jsonify( #ORIG_NETWORKING
                    returnObject #ORIG_NETWORKING
                ) #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500") #ORIG_NETWORKING

@name_space.route("/getsimpositions/", methods=['GET']) #ORIG_NETWORKING
class MainClass(Resource): #ORIG_NETWORKING

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={}) #ORIG_NETWORKING
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..") #ORIG_NETWORKING
    @app.response(200, 'Success', RSUVehicleGetSimPositions) #ORIG_NETWORKING
    def get(self): #ORIG_NETWORKING
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json() #ORIG_NETWORKING
        try: #ORIG_NETWORKING
            #print("data:", request_data)
            if request_data: #ORIG_NETWORKING
                key = request_data['key'] #ORIG_NETWORKING
                vid = int(request_data['id']) #ORIG_NETWORKING
                vtype = int(request_data['type']) #ORIG_NETWORKING

                returnObject = flask_app.config['RSUClass'].getSimPositions(key, vid, vtype) #ORIG_NETWORKING

                return jsonify(
                    returnObject
                ) #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500") #ORIG_NETWORKING

@name_space.route("/getsimtime/", methods=['GET']) #ORIG_NETWORKING
class MainClass(Resource): #ORIG_NETWORKING

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={}) #ORIG_NETWORKING
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..") #ORIG_NETWORKING
    @app.response(200, 'Success', RSUVehicleGetSimTime) #ORIG_NETWORKING
    def get(self): #ORIG_NETWORKING
        #print("got request")
        #print(request.is_json)
        try: #ORIG_NETWORKING
            returnObject = flask_app.config['RSUClass'].getSimTime() #ORIG_NETWORKING

            return jsonify(
                returnObject
            ) #ORIG_NETWORKING
        except Exception as e:#ORIG_NETWORKING
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")#ORIG_NETWORKING

@name_space.route("/sendsimposition/", methods=['GET'])#ORIG_NETWORKING
class MainClass(Resource):#ORIG_NETWORKING

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})#ORIG_NETWORKING
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation.")#ORIG_NETWORKING
    @app.response(200, 'Success', RSUVehicleSendSimPosition)#ORIG_NETWORKING
    def get(self):#ORIG_NETWORKING
        #time1 = flask_app.config['RSUClass'].getTime()
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json() #ORIG_NETWORKING
        try: #ORIG_NETWORKING
            #print("data:", request_data)
            if request_data: #ORIG_NETWORKING
                key = request_data['key'] #ORIG_NETWORKING
                id = int(request_data['id']) #ORIG_NETWORKING
                type = int(request_data['type']) #ORIG_NETWORKING
                x = float(request_data['x']) #ORIG_NETWORKING
                y = float(request_data['y']) #ORIG_NETWORKING
                z = float(request_data['z']) #ORIG_NETWORKING
                roll = float(request_data['roll']) #ORIG_NETWORKING
                pitch = float(request_data['pitch']) #ORIG_NETWORKING
                yaw = float(request_data['yaw']) #ORIG_NETWORKING
                velocity = request_data['velocity'] #ORIG_NETWORKING
                try: #ORIG_NETWORKING
                    returnObject = flask_app.config['RSUClass'].sendSimPositions(key, id, type, x, y, z, roll, pitch, yaw, velocity) #ORIG_NETWORKING

                    return jsonify(
                        returnObject
                    ) #ORIG_NETWORKING
                except Exception as e: #ORIG_NETWORKING
                    name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500") #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500") #ORIG_NETWORKING

@name_space.route("/guiread/", methods=['GET']) #ORIG_NETWORKING
class MainClass(Resource): #ORIG_NETWORKING

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'}, params={}) #ORIG_NETWORKING
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..") #ORIG_NETWORKING
    @app.response(200, 'Success', RSUVehicleGetSimTime) #ORIG_NETWORKING
    def get(self): #ORIG_NETWORKING
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json() #ORIG_NETWORKING
        try: #ORIG_NETWORKING
            #print("data:", request_data)
            if request_data: #ORIG_NETWORKING
                coordinates = request_data['coordinates'] #ORIG_NETWORKING
            else: #ORIG_NETWORKING
                coordinates = False #ORIG_NETWORKING

            returnObject = flask_app.config['RSUClass'].getGuiValues(coordinates) #ORIG_NETWORKING

            #print ( returnObject )

            return jsonify(
                returnObject
            ) #ORIG_NETWORKING
        except Exception as e: #ORIG_NETWORKING
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500") #ORIG_NETWORKING

@name_space.route("/guisend/", methods=['GET']) #ORIG_NETWORKING
class MainClass(Resource): #ORIG_NETWORKING

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'}, params={}) #ORIG_NETWORKING
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..") #ORIG_NETWORKING
    @app.response(200, 'Success', RSUVehicleGetSimTime) #ORIG_NETWORKING
    def get(self): #ORIG_NETWORKING
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json() #ORIG_NETWORKING
        try: #ORIG_NETWORKING
            #print("data:", request_data)
            if request_data: #ORIG_NETWORKING
                velocity_targets = request_data['velocity_targets'] #ORIG_NETWORKING
                pause = request_data['pause'] #ORIG_NETWORKING
                end = request_data['end'] #ORIG_NETWORKING
                button_states = request_data['button_states'] #ORIG_NETWORKING

                returnObject = flask_app.config['RSUClass'].sendGuiValues(velocity_targets, pause, end, button_states) #ORIG_NETWORKING

                #print ( returnObject )
        except Exception as e: #ORIG_NETWORKING
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500") #ORIG_NETWORKING