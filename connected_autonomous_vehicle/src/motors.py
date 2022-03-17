from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import math


''' This class communicates with the PCA9685 servo controller board in order to control the 
steering servo and main motor PWM triggered motor driver.'''
class Motors:
    def __init__(self):
        self.steering_angle_max = 30.0
        self.servo_center = 90.0
        self.servoPIN = 0
        self.motorPIN = 1
        self.motorMax = 0.1

        print ( " Initializing motors " )

        # Startup the i2c interface
        i2c = busio.I2C(SCL, SDA)

        # Create a simple PCA9685 class instance
        self.pca = PCA9685(i2c)
        self.pca.frequency = 256

        # Start the PWM for the motor at 0
        self.motor = self.pca.channels[self.motorPIN]
        self.motor.duty_cycle = 0

        # Setup the steering servo and set to center
        self.steering = servo.Servo(self.pca.channels[self.servoPIN])
        self.steering.angle = self.servo_center

        print ( " Motors initialized " )

    def setControlMotors(self, steeringAcceleration, motorAcceleration):
        if steeringAcceleration == 0.0:
            servo = servo_center
            motor = 0.0
        else:
            # Adjust to our 180 degree servo, 90 = 30, -90 = -30
            servo = self.servo_center + (math.degrees(steeringAcceleration) * 3.0)
        
        if motorAcceleration > 0.0:
            motor = self.motorMax * motorAcceleration
            if motor > self.motorMax:
                motor = self.motorMax
        else:
            motor = 0.0

        self.motor.duty_cycle = int(65535*motor)
        #self.motor.duty_cycle = 0
        self.steering.angle = servo

        print ( "motor target: " , motorAcceleration, " actual " , self.motor.duty_cycle)
        print ( "Steering PID:" , servo, " Motor PID:", motor )

    def emergencyStop(self):
        # emergency stop set the motor to 0
        self.motor.duty_cycle = 0

    # Calling destructor 
    def __del__(self):
        self.pca.deinit()
        



