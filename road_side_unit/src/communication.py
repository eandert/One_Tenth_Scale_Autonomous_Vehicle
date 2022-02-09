#!/usr/bin/env python3
# We have to force python 3 or flask fails miserably!

from flask import Flask, jsonify, request
from flask_restx import Api, Resource, fields
import time

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

flask_app = Flask(__name__)
app = Api(app = flask_app, version='1.0', title='One Tenth Scale RSU',
          description='This is the API for the one tenth scale road side unit controller')

name_space = app.namespace('RSU', description='One Tenth Scale RSU operations')

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
})

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
})

RSUVehicleGetSimPositions = app.model('RSUVehicleGetSimPositions', {
                                'veh_locations': fields.Float(required=True,
                                                        description="Lets us set velocity target from the app, 0 Simulation is paused.",
                                                        example=0.1,
                                                        help="Target velocity cannot be blank.")
})

RSUVehicleGetSimTime = app.model('RSUVehicleGetSimTime', {
                                'time': fields.Float(required=True,
                                                        description="Simulation time",
                                                        example=0.1,
                                                        help="Time cannot be blank.")
})

RSUVehicleSendSimPosition = app.model('RSUVehicleSendSimPosition', {
                                'returned': fields.Boolean(required=True,
                                                        description="Returned",
                                                        example=True,
                                                        help="Returned cannot be blank.")
})

@name_space.route("/register/", methods=['GET'])
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})
    @app.doc(description="This method can be called while a RSU instance is running to resgister a cav and get the coordinate transpose and route for said vehicle.")
    @app.response(200, 'Success', RSUVehicleCheckinResponse)
    def get(self):
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json()
        try:
            #print("data:", request_data)
            if request_data:
                key = request_data['key']
                id = int(request_data['id'])
                type = int(request_data['type'])
                timestamp = float(request_data['timestamp'])
                x = float(request_data['x'])
                y = float(request_data['y'])
                z = float(request_data['z'])
                roll = float(request_data['roll'])
                pitch = float(request_data['pitch'])
                yaw = float(request_data['yaw'])

                #print("recieved")

                returnObject = flask_app.config['RSUClass'].register(key, id, type, timestamp, x, y, z, roll, pitch, yaw)

                #print("replying")

                if type == 0:
                    print ( "Registered vehicle: " + str(id) + " at " + str(timestamp) )
                elif type == 1:
                    print ( "Registered sensor: " + str(id) + " at " + str(timestamp) )


                return jsonify(
                    returnObject
                )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")
            print ( str(e) )


@name_space.route("/checkin/", methods=['GET'])
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})
    @app.doc(description="This method can be called after a CAV is registered to update the position of the vehicle, log detections, and get the state of traffic lighs.")
    @app.response(200, 'Success', RSUVehicleCheckinResponse)
    def get(self):
        time1 = time.time()
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json()
        try:
            #print("data:", request_data)
            if request_data:
                key = request_data['key']
                id = int(request_data['id'])
                type = int(request_data['type'])
                timestamp = float(request_data['timestamp'])
                x = float(request_data['x'])
                y = float(request_data['y'])
                z = float(request_data['z'])
                roll = float(request_data['roll'])
                pitch = float(request_data['pitch'])
                yaw = float(request_data['yaw'])
                steeringAcceleration = float(request_data['steeringAcceleration'])
                motorAcceleration = float(request_data['motorAcceleration'])
                targetIndexX = float(request_data['targetIndexX'])
                targetIndexY = float(request_data['targetIndexY'])
                detections = request_data['detections']

                returnObject = flask_app.config['RSUClass'].checkinFastResponse(key, id, type, timestamp, x, y, z, roll, pitch, yaw, steeringAcceleration, motorAcceleration, targetIndexX, targetIndexY, detections)

                #flask_app.config['RSUQueue'].put([key, id, type, timestamp, x, y, yaw, detections])

                if type == 0:
                    print("Vehicle: " + str(id) + " updated at " + str(timestamp))
                elif type == 1:
                    print("Sensor: " + str(id) + " updated at " + str(timestamp))

                #print ( "Response took: ", time.time() - time1)

                return jsonify(
                    returnObject
                )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")

@name_space.route("/getsimpositions/", methods=['GET'])
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..")
    @app.response(200, 'Success', RSUVehicleGetSimPositions)
    def get(self):
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json()
        try:
            #print("data:", request_data)
            if request_data:
                key = request_data['key']
                vid = int(request_data['id'])
                vtype = int(request_data['type'])

                returnObject = flask_app.config['RSUClass'].getSimPositions(key, vid, vtype)

                return jsonify(
                    returnObject
                )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")

@name_space.route("/getsimtime/", methods=['GET'])
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..")
    @app.response(200, 'Success', RSUVehicleGetSimTime)
    def get(self):
        #print("got request")
        #print(request.is_json)
        try:
            returnObject = flask_app.config['RSUClass'].getSimTime()

            return jsonify(
                returnObject
            )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")

@name_space.route("/sendsimposition/", methods=['GET'])
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation.")
    @app.response(200, 'Success', RSUVehicleSendSimPosition)
    def get(self):
        #time1 = flask_app.config['RSUClass'].getTime()
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json()
        try:
            #print("data:", request_data)
            if request_data:
                key = request_data['key']
                id = int(request_data['id'])
                type = int(request_data['type'])
                x = float(request_data['x'])
                y = float(request_data['y'])
                z = float(request_data['z'])
                roll = float(request_data['roll'])
                pitch = float(request_data['pitch'])
                yaw = float(request_data['yaw'])
                velocity = request_data['velocity']
                try:
                    returnObject = flask_app.config['RSUClass'].sendSimPositions(key, id, type, x, y, z, roll, pitch, yaw, velocity)

                    return jsonify(
                        returnObject
                    )
                except Exception as e:
                    name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")

@name_space.route("/guiread/", methods=['GET'])
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'}, params={})
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..")
    @app.response(200, 'Success', RSUVehicleGetSimTime)
    def get(self):
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json()
        try:
            #print("data:", request_data)
            if request_data:
                coordinates = request_data['coordinates']
            else:
                coordinates = False

            returnObject = flask_app.config['RSUClass'].getGuiValues(coordinates)

            #print ( returnObject )

            return jsonify(
                returnObject
            )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")

@name_space.route("/guisend/", methods=['GET'])
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'}, params={})
    @app.doc(description="This method is called during simulation to get the locations of other vehicels within the simulation..")
    @app.response(200, 'Success', RSUVehicleGetSimTime)
    def get(self):
        #print("got request")
        #print(request.is_json)
        request_data = request.get_json()
        try:
            #print("data:", request_data)
            if request_data:
                velocity_targets = request_data['velocity_targets']
                pause = request_data['pause']
                end = request_data['end']
                button_states = request_data['button_states']

                returnObject = flask_app.config['RSUClass'].sendGuiValues(velocity_targets, pause, end, button_states)

                #print ( returnObject )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")