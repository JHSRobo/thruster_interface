#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
import time
from core_lib import pca9685
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from std_srvs.srv import Trigger

# Create the main class for entry
class Thrusters(Node):
    def __init__(self):
        super().__init__('thrusters')


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
        self.thruster_status_sub = self.create_subscription(Bool, 'thruster_status', self.thruster_status_callback, 10)

        # Define slider parameters
        slider_bounds = FloatingPointRange()
        slider_bounds.from_value = 0.0
        slider_bounds.to_value = 1.0
        slider_bounds.step = 0.025
        slider_descriptor = ParameterDescriptor(floating_point_range = [slider_bounds])

        # Tells the thrusters whether they're allowed to spin or not
        self.thrusters_enabled = False

        # Last thruster values to prevent ESC reset
        self.last_thrusters = [0.15] * 8

        # Max delta of thrusters
        self.max_delta = 0.004

    # Runs whenever /cmd_vel topic recieves a new twist msg
    # Twist msg reference: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    def thruster_status_callback(self, msg):
        self.thrusters_enabled = msg.data

    def thruster_callback(self, msg):    

        linearX = msg.linear.x
        linearY = msg.linear.y 
        linearZ = msg.linear.z
        angularX = msg.angular.x
        angularY = msg.angular.y
        angularZ = msg.angular.z

        # Scalar to limit each thruster to 1800 Hz
        scalar = 0.75

        # Decompose the vectors into thruster values
        # linearX references moving along X-axis, or forward
        # angularZ referneces rotation around vertical axis, or Z-axis.

        # For more reference directions, see https://www.canva.com/design/DAFyPqmH8LY/2oMLLaP8HHGi2e07Ms8fug/view
        # Note: Need to update graphic. Note that pitching forward is being considered positive (ie tilting the MEH down)

        # 12 (HH)
        # 56 (VV)
        # 78 (VV)
        # 34 (HH)

        msglist = [(linearX - linearY - angularZ)  * scalar,     # Front-Left Horizontal
                   (linearX + linearY + angularZ)  * scalar,     # Front-Right Horizontal
                   (-linearX - linearY + angularZ) * scalar,     # Back-Left Horizontal
                   (-linearX + linearY - angularZ) * scalar,     # Back-Right Horizontal
                   (-linearZ + angularY - angularX) * scalar,    # Front-Left Vertical
                   (-linearZ + angularY + angularX) * scalar,    # Front-Right Vertical
                   (-linearZ - angularY - angularX) * scalar,    # Back-Left Vertical
                   (-linearZ - angularY + angularX) * scalar]    # Back-Right Vertical

        # function to limit a value between -0.95 and 0.95
        def limit_value(value):
            return max(-0.95, min(value, 0.95))

        msglist = list(map(limit_value, msglist))

        dutylist = [ round(0.15 - msglist[i] / 25, 5) for i in range(8) ]

        # Loop to prevent ESC reset
        # We make sure that the thrusters' speed can only change by a given amount each interval so as to not overwhelm them.
        if self.thrusters_enabled:
            for i in range(8):
                if abs(dutylist[i] - self.last_thrusters[i]) > self.max_delta:
                    if dutylist[i] > self.last_thrusters[i]:
                        dutylist[i] = self.last_thrusters[i] + self.max_delta
                    else:
                        dutylist[i] = self.last_thrusters[i] - self.max_delta
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
