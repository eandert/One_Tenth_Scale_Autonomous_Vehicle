#!/usr/bin/env python3
# We have to force python 3 or flask fails miserably!

from flask import Flask, jsonify, request
from flask_restx import Api, Resource, fields

RSU = None

flask_app = Flask(__name__)
app = Api(app = flask_app, version='1.0', title='One Tenth Scale RSU',
          description='This is the API for the one tenth scale road side unit controller')

name_space = app.namespace('RSU', description='One Tenth Scale RSU operations')

RSUVehicleCheckinResponse = app.model('RSUVehicleCheckinResponse', {
                                'pause': fields.Boolean(required=True,
                                                        description="True if Simulation is paused.",
                                                        example=True,
                                                        help="Pause cannot be blank."),
                                't_v': fields.Float(required=True,
                                                        description="Target velocity.",
                                                        example=1),
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
                                                      description="TFL cotnrol values for points in route.",
                                                      example=1.01),
                                'timestep': fields.Float(required=True,
                                                        description="Current timestep of RSU",
                                                        example=1.01)
})

RSUVehicleRegisterResponse = app.model('RSUVehicleRegisterResponse', {
                                'pause': fields.Boolean(required=True,
                                                        description="True if Simulation is paused.",
                                                        example=True,
                                                        help="Pause cannot be blank."),
                                'tfl_state': fields.List(fields.Integer,
                                                        required=True,
                                                        description="State of traffic lighs.",
                                                        example=1),
                                'veh_locations_x': fields.List(fields.Float,
                                                         required=True,
                                                         description="Location of vehicles x.",
                                                         example=0.0),
                                'veh_locations_y': fields.List(fields.Float,
                                                         required=True,
                                                         description="Location of vehicles y.",
                                                         example=-2.56),
                                'veh_locations_yaw': fields.List(fields.Float,
                                                         required=True,
                                                         description="Location of vehicles yaw.",
                                                         example=1.23),
                                'timestep': fields.Float(required=True,
                                                        description="Current timestep of RSU",
                                                        example=1.01)
})

@name_space.route("/checkin")
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})
    @app.doc(description="This method can be called while a RSU instance is running to resgister a cav and get the coordinate transpose and route for said vehicle.")
    @app.response(200, 'Success', RSUVehicleCheckinResponse)


    def json_example(self):
        request_data = request.get_json()
        try:
            if request_data:
                key = request_data['key']
                vehicle_id = request_data['vehicle_id']
                timestamp = request_data['timestamp']
                x = request_data['x']
                y = request_data['y']
                z = request_data['z']
                roll = request_data['roll']
                pitch = request_data['pitch']
                yaw = request_data['yaw']

                returnObject = RSU.register(key, vehicle_id, timestamp, x, y, z, roll, pitch, yaw)

                return jsonify(
                    returnObject
                )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")

@name_space.route("/checkin")
class MainClass(Resource):

    @app.doc(responses={200: 'OK', 401: 'RSU Not Running.', 500: 'Unknown Error'},
             params={})
    @app.doc(description="This method can be called after a CAV is registered to update the position of the vehicle, log detections, and get the state of traffic lighs.")
    @app.response(200, 'Success', RSUVehicleCheckinResponse)

    def json_example(self):
        request_data = request.get_json()
        try:
            if request_data:
                key = request_data['key']
                vehicle_id = request_data['vehicle_id']
                timestamp = request_data['timestamp']
                x = request_data['x']
                y = request_data['y']
                z = request_data['z']
                roll = request_data['roll']
                pitch = request_data['pitch']
                yaw = request_data['yaw']
                detections = request_data['detections']

                returnObject = RSU.checkin(key, vehicle_id, timestamp, x, y, z, roll, pitch, yaw, detections)

                return jsonify(
                    returnObject
                )
        except Exception as e:
            name_space.abort(500, e.__doc__, status="Could not retrieve information due to unknown internal error.", statusCode="500")