import math

class VehicleType:
    CAV = "CAV"
    CIS = "CIS"

class VehicleParams:
    """
    Class representing the parameters of a vehicle.
    """

    def __init__(self, type):
        if type not in [VehicleType.CAV, VehicleType.CIS]:
            raise Exception("Vehicle type not supported: ", type)
        
        if type == VehicleType.CAV:
            self.width = 0.3
            self.length = 0.57
            self.wheelbase_length = 0.35
            self.wheelbase_width = 0.245
            self.axle_from_center = self.wheelbase_length / 2.0
            self.steering_angle_max = math.radians(30.0)
            self.velocity_max = 1.0
            self.max_turning_radius = self.wheelbase_length / math.tan(self.steering_angle_max)
        elif type == VehicleType.CIS:
            # TODO: What should these be set to for a CIS?
            self.width = .1
            self.length = .1
            self.wheelbase_length = .1
            self.wheelbase_width = .1
            self.axle_from_center = self.wheelbase_length/2.0
            self.steering_angle_max = math.radians(30.0)
            self.velocity_max = 0.0
            self.max_turning_radius = self.wheelbase_length / math.tan(self.steering_angle_max)
