#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time
from core_lib import pca9685
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from std_srvs.srv import Trigger
import RPi.GPIO as GPIO # Only necessary if running on RPi.
from simple_pid import PID

# Create the main class for entry
class Thrusters(Node):
    def __init__(self):
        super().__init__('thrusters')

        # Supress GPIO Output
        GPIO.setwarnings(False)

        self.log = self.get_logger()

        # Attempt to connect to PCA9685
        try: self.pca = pca9685.PCA9685(bus=1)
        except IOError as e:
            self.log.warn("Cannot connect to PCA9685. Ignore this if PWM converter is unplugged")
            #exit()
        else:
            
            # Sets the frequency of the signal to 100hz
            self.pca.set_pwm_frequency(100)
            self.pca.output_enable()

            # Enables all thrusters
            self.pca.channels_set_duty_all(0.15)
            time.sleep(1) # Sleep is necessary to give thrusters time to initialize

        # Creates the subscriber and logger
        self.thruster_sub = self.create_subscription(Twist, 'cmd_vel', self.thruster_callback, 10)
        self.orientation_sub = self.create_subscription(Vector3, 'orientation_sensor', self.orientation_callback, 10)
        self.log = self.get_logger()

        # Define slider parameters
        slider_bounds = FloatingPointRange()
        slider_bounds.from_value = 0.0
        slider_bounds.to_value = 0.02
        slider_bounds.step = 0.0005
        slider_descriptor = ParameterDescriptor(floating_point_range = [slider_bounds])

        # Define PID Values
        self.yaw_control_enabled = False
        self.yaw_effort_value = 0
        self.yp = 0.0055
        self.yi = 0.0
        self.yd = 0.0
        self.error = 0.0

        self.yaw_target = 0

        # Declare parameters
        self.declare_parameter('yaw_control', False)
        self.declare_parameter('yaw_p', self.yp, slider_descriptor)
        self.declare_parameter('yaw_i', self.yi, slider_descriptor)
        self.declare_parameter('yaw_d', self.yd, slider_descriptor)

        self.add_on_set_parameters_callback(self.parameters_callback)


        # Last thruster values to prevent ESC reset
        self.last_thrusters = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15]

        # Max delta of thrusters
        self.max_delta = 0.004

        # Set up PID control stuff
        self.yaw_pid = PID(self.yp, self.yi, self.yd, setpoint = 0)
        #self.yaw_pid.sample_time = 0.00714 # Time between value recalculations (2x cmd_vel hz)
        self.yaw_pid.output_limits = (-1.0, 1.0) # Restrict output between -1 and 1

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'yaw_p': self.yp = param.value
            if param.name == 'yaw_i': self.yi = param.value
            if param.name == 'yaw_d': self.yd = param.value
            if param.name == 'yaw_control': self.yaw_control_enabled = param.value

        self.yaw_pid.tunings = (self.yp, self.yi, self.yd)

        if self.yaw_control_enabled:
            self.yaw_pid.set_auto_mode(True, last_output = self.yaw)
        if not self.yaw_control_enabled:
            self.yaw_pid.auto_mode = False

        return SetParametersResult(successful=True)

    def orientation_callback(self, msg):
        self.yaw = msg.x

        # Calculate a new PID effort value based on the newly updated orientation
        if self.yaw_control_enabled: 
            self.error = self.calculate_error(self.yaw)
        else:
            self.yaw_target = self.yaw

    # We provide this function to the pid controller so that it understands that 360 wraps around to 0
    def angle_wrap(self, angle):
        if angle > 0:
            if angle > 180:
                return angle - 360
        else:
            if angle < -180:
                return angle + 360
        return angle

    # Takes in actual angle of the ROV and calculates the error relative to setpoint
    def calculate_error(self, angle):
        error = self.yaw_target - angle
        error = self.angle_wrap(error)
        return error


    # Runs whenever /cmd_vel topic recieves a new twist msg
    # Twist msg reference: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    def thruster_callback(self, msg):
            
            # Implement separate logic for yaw control
            if self.yaw_control_enabled:

                # This callback runs about 75x / second.
                # We want top speed to rotate the ROV 360 degrees in 1 second.

                self.yaw_target += (msg.angular.z * 360 / 75)
                self.yaw_target = self.yaw_target % 360
                self.yaw_pid.setpoint = 0 #self.yaw_target
                self.yaw_effort_value = self.yaw_pid(self.error)
                self.log.info("effort: {} \
                        target: {} \
                        error: {}".format(self.yaw_effort_value, self.yaw_target, self.error))

                angularZ = self.yaw_effort_value

            
            linearX = msg.linear.x
            linearY = msg.linear.y
            linearZ = msg.linear.z
            angularX = msg.angular.x
            angularZ = msg.angular.z / 1.4142136 # Division because thrusters do not have to vector against each other for angular motion

            
            # Decompose the vectors into thruster values
            # linearX references moving along X-axis, or forward
            # angularZ referneces rotation around vertical axis, or Z-axis.
            # For more reference directions, see https://www.canva.com/design/DAFyPqmH8LY/2oMLLaP8HHGi2e07Ms8fug/view
        
            scale_value = 0.5 * (2 ** -0.5)

            msglist = [(linearX - linearY - angularZ) * scale_value, 
                       (linearX + linearY + angularZ) * scale_value,
                       (-linearX - linearY + angularZ) * scale_value,
                       (-linearX + linearY - angularZ) * scale_value,
                       -linearZ - angularX,
                       -linearZ + angularX]

            # function to limit a value between -1 and 1
            # min(value, 1) takes the returns lesser of the two values. So if value is greater than 1, it returns 1.
            def limit_value(value):
                return max(-0.95, min(value, 0.95))

            # Use map() to apply the limit_value function to each element of msglist
            msglist = list(map(limit_value, msglist))

            dutylist = [ round(0.15 - msglist[i] / 25, 5) for i in range(6) ]
            for i in range(6):
                if abs(dutylist[i] - self.last_thrusters[i]) > self.max_delta:
                    if dutylist[i] > self.last_thrusters[i]:
                        dutylist[i] = self.last_thrusters[i] + self.max_delta
                    else:
                        dutylist[i] = self.last_thrusters[i]- self.max_delta

                self.last_thrusters[i] = dutylist[i]
                #self.pca.channel_set_duty(i, dutylist[i])
    
            #self.log.info(str(dutylist))
            
# Runs the node
def main(args=None):
    rclpy.init(args=args)

    thrusters = Thrusters()

    rclpy.spin(thrusters)
    
    thrusters.destroy_node
    rclpy.shutdown

if __name__ == '__main__':
    main()
