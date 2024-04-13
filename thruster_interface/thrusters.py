#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from core_lib import pca9685
import RPi.GPIO as GPIO # Only necessary if running on RPi.

# Create the main class for entry
class Thrusters(Node):
    def __init__(self):
        super().__init__('thrusters')

        # Supress GPIO Output
        GPIO.setwarnings(False)

        self.logger = self.get_logger()

        # Attempt to connect to PCA9685
        try: self.pca = pca9685.PCA9685(bus=1)
        except IOError as e: self.logger.warn("Cannot connect to PCA9685. Ignore this if PWM converter is unplugged")
        else:
            
            # Sets the frequency of the signal to 100hz
            self.pca.set_pwm_frequency(100)
            self.pca.output_enable()

            # Enables all thrusters
            self.pca.channels_set_duty_all(0.15)
            time.sleep(1) # Sleep is necessary to give thrusters time to initialize

        # Creates the subscriber and logger
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.thruster_callback, 10)
        self.auto_subscriber = self.create_subscription(Twist, 'autovector', self.auto_callback, 10)
        self.logger = self.get_logger()

    
    # Runs whenever /cmd_vel topic recieves a new twist msg
    # Twist msg reference: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    def thruster_callback(self, msg):
            
            linearX = msg.linear.x
            linearY = msg.linear.y
            linearZ = msg.linear.z
            angularX = msg.angular.x
            angularZ = msg.angular.z

            # Decompose the vectors into thruster values
            msglist = [linearX - linearY - angularZ, 
                       linearX + linearY + angularZ,
                       -linearX - linearY + angularZ,
                       -linearX + linearY - angularZ,
                       -linearZ - angularX,
                       -linearZ + angularX]
            
            # Goes through each thruster and changes the list values into a duty_cycle
            for i in range(0, 6):
                duty_cycle = 0.15 - msglist[i] / 25

                # Writes the duty cycles
                self.pca.channel_set_duty(i, duty_cycle)
    def auto_callback(self, msg): # same exact callback but for autonomous task
            linearX = msg.linear.x
            linearY = msg.linear.y
            linearZ = msg.linear.z
            angularX = msg.angular.x
            angularZ = msg.angular.z

            # Decompose the vectors into thruster values
            msglist = [linearX - linearY - angularZ, 
                       linearX + linearY + angularZ,
                       -linearX - linearY + angularZ,
                       -linearX + linearY - angularZ,
                       -linearZ - angularX,
                       -linearZ + angularX]
            
            # Goes through each thruster and changes the list values into a duty_cycle
            for i in range(0, 6):
                duty_cycle = 0.15 - msglist[i] / 25

                # Writes the duty cycles
                self.pca.channel_set_duty(i, duty_cycle)
       
# Runs the node
def main(args=None):
    rclpy.init(args=args)

    thrusters = Thrusters()

    rclpy.spin(thrusters)
    
    thrusters.destroy_node
    rclpy.shutdown

if __name__ == '__main__':
    main()
