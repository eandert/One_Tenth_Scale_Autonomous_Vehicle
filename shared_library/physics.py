import math
from enum import Enum


class PhysicsModel(Enum):
    """Represents the different physics models for the RC car."""
    BICYCLE = 0


class Physics:
    """
    This class represents the physics model for a vehicle. It includes methods for converting
    coordinates between the center of the car and the rear axle, and for forcing an update of
    the vehicle's position, velocity, and orientation.

    Attributes:
        vehicle_params (VehicleParams): The parameters of the vehicle, including the distance
                                        from the center of the car to the rear axle.

    Methods:
        convert_from_rear_axle_to_center(x_rear_axle, y_rear_axle, theta): Converts the
            coordinates from the rear axle of the car to the center of the car.
        convert_from_center_to_rear_axle(center_x, center_y, theta): Converts the coordinates
            from the center of the car to the rear axle.
        force_position_update(center_x, center_y, velocity, theta): Forces the physics model
            to move the position, velocity, and theta to a certain position.
    """

    def __init__(self, model, vehicle_params):
        """
        Initializes a Physics object with the specified model.

        Args:
            model (PhysicsModel): The model of the physics.
            vehicle_params (dict): A shared_library.vehicle_settings containing the parameters of the vehicle.

        Raises:
            Exception: If the specified model is not supported.
        """
        if model not in PhysicsModel:
            raise Exception("Physics class does not contain the model ", model)
        self.model = model
        self.center_x = 0
        self.center_y = 0
        self.rear_axle_x = 0
        self.rear_axle_y = 0
        self.theta = 0
        self.velocity = 0
        self.vehicle_params = vehicle_params

    def bicycle_model_position_update(self, motor_acceleration, steering_acceleration, timestep):
        """
        Updates the position of the bicycle model based on the given inputs.

        Parameters:
        - motor_acceleration (float): The acceleration of the motor.
        - steering_acceleration (float): The acceleration of the steering.
        - timestep (float): The time step for the update.

        Returns:
        - x (float): The x-coordinate of the updated position.
        - y (float): The y-coordinate of the updated position.
        - theta (float): The updated angle of the vehicle.
        - velocity (float): The updated velocity of the vehicle.
        """
        self.rear_axle_x += self.velocity * math.cos(self.theta) * timestep
        self.rear_axle_y += self.velocity * math.sin(self.theta) * timestep
        self.theta += (self.velocity / self.vehicle_params.wheelbase_length) * \
            math.tan(steering_acceleration) * timestep
        self.velocity += motor_acceleration * timestep

        self.center_x, self.center_y = self.convert_from_rear_axle_to_center(
            self.rear_axle_x, self.rear_axle_y, self.theta)

    def convert_from_rear_axle_to_center(self, x_rear_axle, y_rear_axle, theta):
        """
        Converts the coordinates from the rear axle to the center of the car.

        Args:
            x_rear_axle (float): The x-coordinate of the rear axle.
            y_rear_axle (float): The y-coordinate of the rear axle.
            theta (float): The angle of the car.

        Returns:
            tuple: The converted coordinates (x, y) at the center of the car.
        """
        center_x = x_rear_axle + \
            (self.vehicle_params.axle_from_center * math.cos(theta))
        center_y = y_rear_axle + \
            (self.vehicle_params.axle_from_center * math.sin(theta))
        return center_x, center_y

    def convert_from_center_to_rear_axle(self, center_x, center_y, theta):
        """
        Converts the coordinates from the center of the car to the rear axle.

        Args:
            center_x (float): X-coordinate of the center of the car.
            center_y (float): Y-coordinate of the center of the car.
            theta (float): Orientation angle of the car in radians.

        Returns:
            tuple: A tuple containing the X and Y coordinates of the rear axle.
        """
        reverse_theta = theta - math.radians(180)
        x_rear_axle = center_x - \
            (self.vehicle_params.axle_from_center * math.cos(reverse_theta))
        y_rear_axle = center_y - \
            (self.vehicle_params.axle_from_center * math.sin(reverse_theta))
        return x_rear_axle, y_rear_axle

    def force_position_update(self, center_x, center_y, velocity, theta):
        """
        Force the physics model to move the position, velocity, and theta to a certain position.
        This is typically used after initialization or when we are in the real world.

        Parameters:
            x (float): The x position of the rear axle in meters w.r.t. global 0.
            y (float): The y position of the rear axle in meters w.r.t. global 0.
            velocity (float): The velocity of travel of the rear axle in meters per second.
            theta (float): The direction of travel of the rear axle in radians.

        Returns:
            None
        """
        self.rear_axle_x, self.rear_axle_y = self.convert_from_center_to_rear_axle(
            center_x, center_y, theta)
        self.theta = theta
        self.velocity = velocity

    def update_position(self, motor_acceleration, steering_acceleration, timestep):
        """
        Updates the position of the vehicle based on the given motor acceleration, steering acceleration,
        wheelbase length, and timestep.

        Args:
            motor_acceleration (float): The acceleration of the motor.
            steering_acceleration (float): The acceleration of the steering.
            wheelbase_length (float): The length of the wheelbase.
            timestep (float): The time step for the update.

        Returns:
            The updated position of the vehicle.

        Raises:
            Exception: If the physics model is not supported.
        """
        if self.model == PhysicsModel.BICYCLE:
            self.bicycle_model_position_update(
                motor_acceleration, steering_acceleration, timestep)
        else:
            raise Exception(
                "Physics class does not contain the model ", self.model)
