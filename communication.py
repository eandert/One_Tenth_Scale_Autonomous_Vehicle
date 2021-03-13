#!/usr/bin/env python3
# We have to force python 3 or flask fails miserably!

from flask import Flask, jsonify, request
from flask_restx import Api, Resource, fields


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
                detections = request_data['detections']

                returnObject = flask_app.config['RSUClass'].checkinFastResponse(key, id, type, timestamp, x, y, z, roll, pitch, yaw)

                flask_app.config['RSUQueue'].put([key, id, type, timestamp, detections])

                if type == 0:
                    print("Vehicle: " + str(id) + " updated at " + str(timestamp))
                elif type == 1:
                    print("Sensor: " + str(id) + " updated at " + str(timestamp))

                return jsonify(
                    returnObject
                )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")