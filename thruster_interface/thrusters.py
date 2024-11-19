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

        # Get the logger
        self.log = self.get_logger()

        # Attempt to connect to PCA9685
        try: self.pca = pca9685.PCA9685(bus=1)
        except IOError as e:
            self.log.warn("Cannot connect to PCA9685. Ignore this if PWM converter is unplugged")
            exit()
        else:
            
            # Sets the frequency of the signal to 100hz
            self.pca.set_pwm_frequency(100)
            self.pca.output_enable()

            # Enables all thrusters
            self.pca.channels_set_duty_all(0.15)
            time.sleep(1) # Sleep is necessary to give thrusters time to initialize

        # Creates the subscriber
        self.thruster_sub = self.create_subscription(Twist, 'cmd_vel', self.thruster_callback, 10)
        self.orientation_sub = self.create_subscription(Vector3, 'orientation_sensor', self.orientation_callback, 10)
        self.depth_sub = self.create_subscription(Vector3, 'depth_sensor', self.depth_callback, 10)

        # Define slider parameters
        slider_bounds = FloatingPointRange()
        slider_bounds.from_value = 0.0
        slider_bounds.to_value = 1.0
        slider_bounds.step = 0.025
        slider_descriptor = ParameterDescriptor(floating_point_range = [slider_bounds])

        # Define YAW Values
        self.yaw_control_enabled = False
        self.yaw_effort_value = 0
        self.yp = 0.0055
        self.yi = 0.0
        self.yd = 0.0
        self.yaw_error = 0.0

        self.yaw_target = 0

        # Variable to create a deadzone for angularZ for yaw lock
        self.yaw_deadzone = 0.05
        deadzone_bounds = FloatingPointRange()
        deadzone_bounds.from_value = 0
        deadzone_bounds.to_value = 1
        deadzone_bounds.step = 0.01
        deadzone_descriptor = ParameterDescriptor(floating_nt_range = [deadzone_bounds])

        # Define Depth Hold Values
        self.depth_hold_enabled = False
        self.depth_effort_value = 0
        self.depth_p = 0.275
        self.depth_i = 0.0 
        self.depth_d = 0.0 
        self.depth_error = 0.0

        self.depth_target = 0
        
        # Variable to create a deadzone for linearZ for depth hold
        self.depth_deadzone = 0.1
        deadzone_bounds = FloatingPointRange()
        deadzone_bounds.from_value = 0
        deadzone_bounds.to_value = 1
        deadzone_bounds.step = 0.01
        deadzone_descriptor = ParameterDescriptor(floating_nt_range = [deadzone_bounds])


        # Declare parameters
        self.declare_parameter('yaw_control', False)
        self.declare_parameter('yaw_p', self.yp, slider_descriptor)
        self.declare_parameter('yaw_i', self.yi, slider_descriptor)
        self.declare_parameter('yaw_d', self.yd, slider_descriptor)
        self.declare_parameter('yaw_deadzone', self.yaw_deadzone, deadzone_descriptor)

        self.declare_parameter('depth_hold', False)
        self.declare_parameter('depth_p', self.depth_p, slider_descriptor)
        self.declare_parameter('depth_i', self.depth_i, slider_descriptor)
        self.declare_parameter('depth_d', self.depth_d, slider_descriptor)
        self.declare_parameter('depth_deadzone', self.depth_deadzone, deadzone_descriptor)
        
        self.add_on_set_parameters_callback(self.parameters_callback)


        # Last thruster values to prevent ESC reset
        self.last_thrusters = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15]

        # Max delta of thrusters
        self.max_delta = 0.004

        # Sets up PIDs for yaw and depth as well as limits on those PIDs so that they don't overwhelm the thrusters
        self.yaw_pid = PID(self.yp, self.yi, self.yd, setpoint = 0)
        self.yaw_pid.output_limits = (-1.0, 1.0) 

        self.depth_pid = PID(self.depth_p, self.depth_i, self.depth_d, setpoint=0)
        self.depth_pid.output_limits = (-1.0, 1.0)

        #self.yaw_pid.sample_time = 0.00714 # Time between value recalculations (2x cmd_vel hz)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'yaw_p': self.yp = param.value
            if param.name == 'yaw_i': self.yi = param.value
            if param.name == 'yaw_d': self.yd = param.value
            if param.name == 'yaw_deadzone' : self.yaw_deadzone = param.value
            if param.name == 'yaw_control': self.yaw_control_enabled = param.value

            if param.name == 'depth_p': self.depth_p = param.value
            if param.name == 'depth_i': self.depth_i = param.value
            if param.name == 'depth_d': self.depth_d = param.value
            if param.name == 'depth_deadzone': self.depth_deadzone = param.value
            if param.name == 'depth_hold': self.depth_hold_enabled = param.value


        self.yaw_pid.tunings = (self.yp, self.yi, self.yd)
        self.depth_pid_tunings = (self.depth_p, self.depth_i, self.depth_d)

        if self.yaw_control_enabled:
            self.yaw_pid.set_auto_mode(True, last_output = self.yaw)
        if not self.yaw_control_enabled:
            self.yaw_pid.auto_mode = False

        if self.depth_hold_enabled:
            self.depth_pid.set_auto_mode(True, last_output = self.depth)
        if not self.depth_hold_enabled:
            self.depth_pid.auto_mode = False

        return SetParametersResult(successful=True)

    def orientation_callback(self, msg):
        self.yaw = msg.x
        # Calculate a new PID error value based on the newly updated orientation
        self.yaw_error = self.calculate_orientation_error(self.yaw)

    # Takes in actual angle of the ROV and calculates the error relative to setpoint
    def calculate_orientation_error(self, angle):
        error = self.yaw_target - angle
        # Forces angle between -180 and 180 
        if error > 180:
            return error - 360
        elif error < -180:
            return error + 360
        return error

    def depth_callback(self, msg):
        self.depth = msg.data
        # Calculate a new PID error value based on the new depth
        self.depth_error = self.depth_target - self.depth

    # Runs whenever /cmd_vel topic recieves a new twist msg
    # Twist msg reference: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    def thruster_callback(self, msg):    

        linearX = msg.linear.x
        linearY = msg.linear.y 
        angularX = msg.angular.x

        # Implement separate logic for yaw control
        # Use yaw-lock pid value unless pilot is moving on the axis
        # Here, we only use the joystick angularZ value if the pilot has rotated at least self.yaw_deadzone degrees
        self.yaw_error = self.calculate_orientation_error()
        if self.yaw_control_enabled and (abs(msg.angular.z) < self.yaw_deadzone):

            self.yaw_pid.setpoint = 0 #self.yaw_target
            self.yaw_effort_value = self.yaw_pid(self.yaw_error)
            self.log.info("yaw_effort: {} \
                    yaw: {} \
                    yaw_target: {} \
                    yaw_error: {}".format(self.yaw_effort_value, self.yaw, self.yaw_target, self.yaw_error))
            angularZ = self.yaw_effort_value
        else:
            self.yaw_target = self.yaw 
            angularZ = msg.angular.z



        # Depth Hold Logic 
        if self.depth_hold_enabled and (abs(msg.linear.z) < self.depth_deadzone):

            self.depth_pid.setpoint = 0
            self.depth_effort_value = self.depth_pid(self.depth_error)
            self.log.info("depth_effort: {} \
                    self.depth: {} \
                    depth_target: {} \
                    depth_error: {}".format(self.depth_effort_value, self.depth, self.depth_target, self.depth_error))

            linearZ = self.depth_effort_value
        else:
            self.depth_target = self.depth
            linearZ = msg.linear.z

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

        # Loop to prevent ESC reset
        # We 
        for i in range(6):
            if abs(dutylist[i] - self.last_thrusters[i]) > self.max_delta:
                if dutylist[i] > self.last_thrusters[i]:
                    dutylist[i] = self.last_thrusters[i] + self.max_delta
                else:
                    dutylist[i] = self.last_thrusters[i]- self.max_delta

            self.last_thrusters[i] = dutylist[i]
            self.pca.channel_set_duty(i, dutylist[i])

        self.log.info(str(dutylist))
        
# Runs the node
def main(args=None):
    rclpy.init(args=args)

    thrusters = Thrusters()

    rclpy.spin(thrusters)
    
    thrusters.destroy_node
    rclpy.shutdown

if __name__ == '__main__':
    main()
