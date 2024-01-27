# General python imports
import math
import sys
import numpy as np
from simple_pid import PID
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

# fmt: off
# Project specific imports for this project
# TODO(eandert): This is a hacky way to import from the shared library, fix this
sys.path.append("../../../")
from shared_library import physics, sensor, shared_math, vehicle_settings
# fmt: on

''' This class contains the parameters of a RC car platform like wheelbase, etc. as
well as a rudimentary physics estimation based on the ackerman steering model. We can feed the class
a route which consists of a series of x,y coordinates and group speed target integers. The planner
class can then be classed with the current position from the localizer and it will use
pure pursuit to select a target point and decide a PPM value for the steering servo. There is also
a built in PID controller that will select the PID control for the motor based on the target velocity
of the target point velocity group. This velocity group accounts for traffic lights. '''


class Planner:
    def __init__(self):
        # Vehicle parameters
        self.vehicle_params = vehicle_settings.VehicleParams(
            vehicle_settings.VehicleType.CAV)

        # Physics simulator for tracking position in simulation mode
        self.physics = physics.Physics(
            physics.PhysicsModel.BICYCLE, self.vehicle_params)

        # Pure pursuit control settings
        self.k = 1.0  # Look forward gain
        self.look_forward_coefficient = 0.5  # Look-ahead distance
        # Current waypoint index that pure pursuit is targeting
        self.pursuit_target_index = 0
        # Previous waypoint index that pure pursuit was targeting
        self.previous_pursuit_target_index = None

        # PID control settings
        self.velocity_pid_controller = None
        self.distance_pid_controller = None
        self.last_target_within_traffic_light = 0
        self.distance_pid_control_en = False
        self.distance_pid_control_overide = False
        self.follow_distance = self.look_forward_coefficient
        self.follow_distance_gain = .5
        self.target_follow_distance = 1
        self.tfl_mode = 0
        self.av_intersection_permission = 0
        self.tind = 0

        # Position offsets for the LIDAR localizer
        self.target_velocity_general = 0
        self.localizer_position_offset_x = 0
        self.localizer_position_offset_y = 0
        self.localizer_theta_offset = 0

        # Vehicle coordinates and velocity targets
        self.x_coordinates = []
        self.y_coordinates = []
        self.velocity_coordinates = []

        # Updatable parameters
        self.previous_pursuit_target_index = 0
        self.target_index_x = 0
        self.target_index_y = 0
        self.target_velocity = 0
        self.steering_acceleration = 0
        self.motor_acceleration = 0

        # Vehicle position and orientation
        self.global_position_x = 0
        self.global_position_y = 0
        self.global_position_x_actual = 0
        self.global_position_y_actual = 0
        self.theta = 0  # Vehicle orientation
        self.velocity = 0  # Vehicle velocity

        # Miscellaneous parameters
        self.id = None
        self.simulated_vehicle = True
        self.key = None
        self.coordinate_group_velocities = [0, 0, 0, 0, 0]
        # Buffer to be added around vehicles to do follow distance math
        self.arbitrary_buffer = 0.1

        # Sensor detections
        self.camera_detections = []
        self.lidar_detections = []
        self.fusion_detections = []
        self.rawLidarDetections = []
        self.groundTruth = []
        self.localization_covariance = np.array(
            [[1.0, 0.0], [0.0, 1.0]]).tolist()

        # Raw LIDAR for GUI debugging
        self.localization_error = None
        self.lidar_detections_raw = []

        # Initialize sensors with standard error models
        self.localization = sensor.Localization(
            math.sqrt(0.0782), math.sqrt(0.0428), math.sqrt(0.0841), math.sqrt(0.0241))
        self.lidarSensor = sensor.Sensor("M1M1", 0.0, 360, 15.0, math.sqrt(
            0.0097), math.sqrt(0.0361), math.sqrt(0.0165), math.sqrt(0.0607))
        self.cameraSensor = sensor.Sensor("IMX160", 0.0, 160, 10.0, math.sqrt(
            0.0117), math.sqrt(0.023), math.sqrt(0.0517), math.sqrt(0.0126))

    def initialVehicleAtPosition(self, x_init, y_init, theta_init, x_coordinates, y_coordinates, velocity_coordinates, id, simulated_vehicle):
        """
        Initializes the vehicle at a given position and sets up the necessary parameters for control.

        Args:
            x_init (float): The initial x-coordinate of the vehicle.
            y_init (float): The initial y-coordinate of the vehicle.
            theta_init (float): The initial orientation angle of the vehicle.
            x_coordinates (list): List of x-coordinates for the global path.
            y_coordinates (list): List of y-coordinates for the global path.
            velocity_coordinates (list): List of velocity targets for the global path.
            id (int): The ID of the vehicle.
            simulated_vehicle (bool): Flag indicating whether the vehicle is simulated or not.
        """
        self.target_velocity_general = 0
        self.id = id
        self.simulated_vehicle = simulated_vehicle

        # Store the constant offset for the LIDAR (which assumes we are starting at 0,0) to world coordinates here
        self.global_position_x = x_init
        self.global_position_y = y_init
        self.theta = theta_init
        self.velocity = 0

        # Force update our position in the pysicic simulator to match the init position
        self.physics.force_position_update(x_init, y_init, 0, theta_init)

        # Store our starting position as an offset
        self.localizer_position_offset_x = x_init
        self.localizer_position_offset_y = y_init
        self.localizer_theta_offset = theta_init

        # Store our global path coordinates and velocity targets
        self.x_coordinates = x_coordinates
        self.y_coordinates = y_coordinates
        self.velocity_coordinates = velocity_coordinates

        # Initialize the various PID controllers
        if self.simulated_vehicle:
            self.velocity_pid_controller = PID(
                3.0, 0.00, 0.0, setpoint=self.target_velocity)
            self.distance_pid_controller = PID(
                2, 0.00, 0.0, setpoint=self.look_forward_coefficient)
        else:
            self.velocity_pid_controller = PID(
                2, 0.00, 0.0, setpoint=self.target_velocity)
            self.distance_pid_controller = PID(
                2, 0.00, 0.0, setpoint=self.look_forward_coefficient)

    def update_position(self, timestep):
        """
        Update the position of the vehicle based on the motor and steering acceleration.

        Args:
            timestep (float): The time step for the simulation.

        Returns:
            None
        """
        self.physics.update_position(
            self.motor_acceleration, self.steering_acceleration, timestep)

    def update_localization(self, use_incoming_localization, localization=None):
        """
        Update the localization of the vehicle.

        Args:
            use_incoming_localization (bool): Flag indicating whether to use incoming localization data.
            localization (tuple, optional): Localization data consisting of (x, y, theta) coordinates.

        Returns:
            None
        """
        if not use_incoming_localization:
            # Update the localization from the physics model
            self.global_position_x = self.physics.center_x
            self.global_position_y = self.physics.center_y
            self.velocity = self.physics.velocity
            self.theta = self.physics.theta
        else:
            if localization is None:
                raise Exception("Localization is None")
            # Update the localization from real data
            # Calculate velocity before we update, the localization positions are from last frame
            # self.axle_from_center is to adjust for lidar position vs rear axle
            self.velocity = self.calculate_velocity(localization[0] + self.localizer_position_offset_x, localization[1] +
                                                    self.localizer_position_offset_y, self.global_position_x, self.global_position_y, localization[2])

            # Update the localization position taking into account that the M1M1 LIDAR thinks we are at (0,0) when it initializes
            self.global_position_x = localization[0] + \
                self.localizer_position_offset_x
            self.global_position_y = localization[1] + \
                self.localizer_position_offset_y
            self.theta = localization[2] + \
                self.localizer_theta_offset
            # Force update our position in the pysicic simulator
            self.physics.force_position_update(
                self.global_position_x, self.global_position_y, self.velocity, self.theta)

    def calculate_velocity(self, x1, y1, x2, y2, theta):
        """
        Calculates the velocity based on the given coordinates and angle.

        Args:
            x1 (float): X-coordinate of the starting point.
            y1 (float): Y-coordinate of the starting point.
            x2 (float): X-coordinate of the ending point.
            y2 (float): Y-coordinate of the ending point.
            theta (float): Angle of the vehicle.

        Returns:
            float: Calculated velocity.

        """
        velocity = math.hypot(x2 - x1, y2 - y1) * (1/8)
        expected_theta = math.atan2(y2 - y1, x2 - x1)
        if not (theta < (expected_theta + math.radians(45)) and theta > (expected_theta - math.radians(45))):
            # We are traveling backwards according to the LIDAR so adjust the velocity accordingly
            velocity = -velocity
        return velocity

    def pure_pursuit_control(self):
        """
        Performs pure pursuit control to navigate the vehicle towards a target point.

        This method calculates the steering angle and target velocity based on the current position
        and the coordinates of the target point. It also takes into account traffic light conditions
        and autonomous intersection mode.

        Returns:
            None
        """
        index = self.search_target_index()

        tx = self.x_coordinates[index]
        ty = self.y_coordinates[index]
        self.tind = index

        alpha = math.atan2(ty - self.physics.rear_axle_y,
                           tx - self.physics.rear_axle_x) - self.theta

        look_forward = self.k * self.velocity + \
            self.look_forward_coefficient + self.vehicle_params.axle_from_center

        delta = math.atan2(2.0 * self.vehicle_params.wheelbase_length *
                           math.sin(alpha) / look_forward, 1.0)

        if delta > self.vehicle_params.steering_angle_max:
            delta = self.vehicle_params.steering_angle_max
        elif delta < -self.vehicle_params.steering_angle_max:
            delta = -self.vehicle_params.steering_angle_max

        # Account for the fact that in reverse we should be turning the other way
        # TODO(eandert): Do we need this?
        # if self.velocity < 0:
        #    delta = -delta
        self.steering_acceleration = delta

        self.target_index_x = tx
        self.target_index_y = ty
        self.distance_pid_control_overide = False
        if self.velocity_coordinates[index] == 0:
            self.target_velocity = self.target_velocity_general
            self.last_target_within_traffic_light = 0
        else:
            # Traffic light calculation
            if self.tfl_mode == 0:
                if self.coordinate_group_velocities[self.velocity_coordinates[index]] == 1:
                    if self.last_target_within_traffic_light == 1:
                        if self.target_velocity == 0:
                            # We are already stopping so keep stopping
                            self.target_velocity = 0
                            self.distance_pid_control_overide = True
                        else:
                            # We have already entered the light so keep going
                            self.target_velocity = self.target_velocity_general
                    else:
                        # This is the first point we have seen of this TFL, should have enought time to stop
                        self.target_velocity = 0
                        self.distance_pid_control_overide = True
                    self.last_target_within_traffic_light = 1
                elif self.coordinate_group_velocities[self.velocity_coordinates[index]] == 2:
                    self.target_velocity = self.target_velocity_general
                    self.last_target_within_traffic_light = 1
                else:
                    self.target_velocity = 0
                    self.last_target_within_traffic_light = 1
                    self.distance_pid_control_overide = True
            # Autonomous intersection mode calculation
            else:
                if self.av_intersection_permission:
                    self.target_velocity = self.target_velocity_general
                else:
                    self.target_velocity = 0
                    self.distance_pid_control_overide = True

    def update_pid(self):
        """
        Update the PID controllers based on the current state and control the motor acceleration.

        If distance PID control is enabled and not overridden, the distance-based PID control is used
        to maintain a target follow distance with respect to the velocity.
        Otherwise, the velocity PID control is used to maintain a target velocity.

        If the target velocity is set to 0 or if reverse is disabled in the case of a physical car,
        the motor acceleration is set to 0.

        Returns:
            None
        """
        if self.distance_pid_control_en and not self.distance_pid_control_overide:
            # Distance based PID control because there is an obstruction or vehicles ahead
            self.distance_pid_controller.setpoint = self.target_follow_distance + \
                self.follow_distance_gain * self.velocity
            self.motor_acceleration = self.distance_pid_controller(
                self.follow_distance)
        else:
            # Use velocity PID control because there is no obstruction ahead
            self.velocity_pid_controller.setpoint = self.target_velocity
            self.motor_acceleration = self.velocity_pid_controller(
                self.velocity)

        # Set our target velocity to 0 either if the global velocity is set to 0 or if we have reverse disabled in the case of a physical car
        if self.target_velocity_general == 0.0 or (self.target_velocity <= 0.0 and not self.simulated_vehicle):
            self.motor_acceleration = 0.0

    def calculate_distance_to_point(self, point_x, point_y):
        """
        Calculates the distance between the rear axle position and a given point.

        Args:
            point_x (float): The x-coordinate of the point.
            point_y (float): The y-coordinate of the point.

        Returns:
            float: The distance between the rear axle position and the given point.
        """
        dx = self.physics.rear_axle_x - point_x
        dy = self.physics.rear_axle_y - point_y
        return math.hypot(dx, dy)

    def recieve_coordinate_group_commands(self, commands):
        """
        Receives a list of coordinate group commands and updates the
        coordinate group velocities.

        Args:
            commands (list): A list of coordinate group commands.

        Returns:
            None
        """
        self.coordinate_group_velocities = commands

    def get_location(self):
        """
        Returns the location information of the vehicle.

        Returns:
            list: A list containing the global position x, global position y, theta, target velocity,
                  vehicle width, vehicle length, and vehicle ID.
        """
        return [self.global_position_x, self.global_position_y, self.theta, self.target_velocity, self.vehicle_params.width, self.vehicle_params.length, self.id]

    def check_if_point_in_rectangle(self, x1, y1, x2, y2, x, y):
        """
        Check if a point (x, y) is inside a rectangle defined by two points (x1, y1) and (x2, y2).

        Args:
            x1 (float): x-coordinate of the top left point of the rectangle.
            y1 (float): y-coordinate of the top left point of the rectangle.
            x2 (float): x-coordinate of the bottom right point of the rectangle.
            y2 (float): y-coordinate of the bottom right point of the rectangle.
            x (float): x-coordinate of the point to be checked.
            y (float): y-coordinate of the point to be checked.

        Returns:
            bool: True if the point is inside the rectangle, False otherwise.
        """
        if x1 > x2:
            xtemp = x2
            x2 = x1
            x1 = xtemp
        if y1 > y2:
            ytemp = y2
            y2 = y1
            y1 = ytemp
        if (x > x1) and (x < x2):
            if (y > y1) and (y < y2):
                return True
        return False

    def check_in_range_and_fov(self, target_angle, distance, center_angle, horizontal_fov, max_range):
        """
        Check if the target is within the specified range and field of view.

        Args:
            target_angle (float): The angle of the target.
            distance (float): The distance to the target.
            center_angle (float): The angle of the center.
            horizontal_fov (float): The horizontal field of view.
            max_range (float): The maximum range.

        Returns:
            bool: True if the target is within the range and field of view, False otherwise.
        """
        angle_diff = ((center_angle - target_angle + math.pi +
                      (2*math.pi)) % (2*math.pi)) - math.pi
        if abs(angle_diff) <= (horizontal_fov/2.0) and (distance <= max_range):
            return True
        return False

    def check_positions_of_other_vehicles_adjust_velocity(self, positions):
        """
        Checks the positions of other vehicles and adjusts the velocity accordingly.

        Args:
            positions (list): List of positions of other vehicles.

        Returns:
            None
        """
        self.follow_distance = 99
        self.distance_pid_control_en = False
        point = Point(self.target_index_x, self.target_index_y)
        for idx, each in enumerate(positions):
            # Create a bounding box for each vehicle that is length + 2*buffer long and width + 2*buffer wide
            x1 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2]+math.radians(
                90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2]-math.radians(180))))
            y1 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2]+math.radians(
                90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2]-math.radians(180))))
            x2 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2]-math.radians(
                90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2]-math.radians(180))))
            y2 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2]-math.radians(
                90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2]-math.radians(180))))
            x3 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2] -
                            math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2])))
            y3 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2] -
                            math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2])))
            x4 = each[0] + ((each[4]/2.0+self.arbitrary_buffer)*math.cos(each[2] +
                            math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2])))
            y4 = each[1] + ((each[4]/2.0+self.arbitrary_buffer)*math.sin(each[2] +
                            math.radians(90)) + ((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2])))

            polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
            if polygon.contains(point):
                # Within the safety buffer, adjust speed to be that of the front vehicle using distance control
                target_follow_distance = self.look_forward_coefficient + \
                    self.follow_distance_gain * self.velocity
                # follow_distance = math.hypot(each[0]+((each[5]/2.0+self.arbitrary_buffer)*math.cos(each[2]))-self.physics.rear_axle_x, each[1]+((each[5]/2.0+self.arbitrary_buffer)*math.sin(each[2]))-self.physics.rear_axle_y)
                follow_distance = math.hypot(
                    self.global_position_x - (x3+x4)/2.0, self.global_position_y - (y3+y4)/2.0)
                if self.follow_distance > follow_distance:
                    self.follow_distance = follow_distance
                    self.target_follow_distance = target_follow_distance
                    self.distance_pid_control_en = True

    def check_steering_angle_possible(self, x, y):
        """
        Check if it is possible to reach the current point based on geometry.

        Args:
            x (float): The x-coordinate of the target point.
            y (float): The y-coordinate of the target point.

        Returns:
            bool: True if it is possible to reach the target point, False otherwise.
        """
        # This equation is a quick check for if it is possible to get to the current point based on geometry
        # Essentually 2 circles that we cant go to
        dx = self.physics.rear_axle_x + self.vehicle_params.max_turning_radius * \
            math.cos(shared_math.angleDifference(
                self.theta + math.radians(90), x))
        dy = self.physics.rear_axle_y + self.vehicle_params.max_turning_radius * \
            math.cos(shared_math.angleDifference(
                self.theta + math.radians(90), y))
        d = math.hypot(dx, dy)

        dx2 = self.physics.rear_axle_x + self.vehicle_params.max_turning_radius * \
            math.cos(shared_math.angleDifference(
                self.theta - math.radians(90), x))
        dy2 = self.physics.rear_axle_y + self.vehicle_params.max_turning_radius * \
            math.cos(shared_math.angleDifference(
                self.theta - math.radians(90), y))
        d2 = math.hypot(dx2, dy2)

        a = shared_math.angleDifference(math.atan2(
            self.physics.rear_axle_y - y, self.physics.rear_axle_x - x), self.theta)

        # Check if the target point is within either no go circle
        if d < self.vehicle_params.max_turning_radius or d2 < self.vehicle_params.max_turning_radius and a > math.radians(-90) and a < math.radians(90):
            return False
        else:
            return True

    def search_target_index(self):
        """
        Searches for the target index in the list of coordinates based on the current position and velocity.

        Returns:
            int: The index of the target point in the list of coordinates.
        """
        look_forward = self.k * self.velocity + self.look_forward_coefficient

        if self.previous_pursuit_target_index is None:
            # Search for the initial point, not reverse
            dx = [self.physics.rear_axle_x - icx for icx in self.x_coordinates]
            dy = [self.physics.rear_axle_y - icy for icy in self.y_coordinates]
            d = np.hypot(dx, dy)
            for index in range(len(dx)):
                if not self.check_steering_angle_possible(dx[index], dy[index]):
                    d[index] = 1000.0
            index = np.argmin(d)
            self.previous_pursuit_target_index = index
        else:
            index = self.previous_pursuit_target_index
            distance_this_index = self.calculate_distance_to_point(
                self.x_coordinates[index], self.y_coordinates[index])
            while True:
                check_index = index + 1
                if check_index >= len(self.x_coordinates):
                    check_index = 0
                distance_next_index = self.calculate_distance_to_point(
                    self.x_coordinates[check_index], self.y_coordinates[check_index])
                if distance_next_index > distance_this_index:
                    break
                distance_this_index = distance_next_index
                index = check_index
            self.previous_pursuit_target_index = index

        look_ahead = self.calculate_distance_to_point(
            self.x_coordinates[index], self.y_coordinates[index])

        # search look ahead target point index
        while look_forward > look_ahead:
            look_ahead = self.calculate_distance_to_point(
                self.x_coordinates[index], self.y_coordinates[index])
            index += 1
            if index >= len(self.x_coordinates):
                index = 0

        return index

    def return_command_package(self):
        """
        Returns the command package for controlling the vehicle.

        The command package consists of the steering acceleration and motor acceleration.
        The steering acceleration is negated to reverse the steering direction.

        Returns:
            tuple: A tuple containing the steering acceleration and motor acceleration.
        """
        return -self.steering_acceleration, self.motor_acceleration
