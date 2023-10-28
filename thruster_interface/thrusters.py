#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from core_lib import pca9685
import RPi.GPIO as GPIO

# Create the main class for entry
class Thrusters(Node):
    def __init__(self):
        super().__init__('thrusters')

        # Supress GPIO Output
        GPIO.setwarnings(False)


        # Made so if the pca is disconnected the program does not error out
        try: self.pca = pca9685.PCA9685(bus=1)
        except: self.logger.warn("Cannot connect to PCA9685. Ignore this if thrusters are unplugged")
        else:
            # Sets the frequency of the signal to 400hz
            self.pca.set_pwm_frequency(400)
            self.pca.output_enable()

            # Enables all thrusters
            self.pca.channels_set_duty_all(0.6)
            time.sleep(1)

        # Creates the subscriber and logger
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.thruster_callback, 10)
        self.subscription
        self.logger = self.get_logger()

    def thruster_callback(self, msg):
            
            # Converts the Twist vector into interpreted
            linearX = msg.linear.x
            linearY = msg.linear.y
            linearZ = msg.linear.z
            angularX = msg.angular.x
            angularZ = msg.angular.z

            # Creates a list of those vector values to control each thruster separately
            msglist = [linearX - linearY - angularZ, 
                       linearX + linearY + angularZ,
                       -linearX - linearY + angularZ,
                       -linearX + linearY - angularZ,
                       -linearZ - angularX,
                       -linearZ + angularX]
            
            # Goes through each thruster and changes the list values into a duty_cycle
            for i in range(0, 6):
                duty_cycle = 0.6 - msglist[i] / 3.125

                self.pca.channel_set_duty(i, duty_cycle)
       
# Runs the subscirber
def main(args=None):
    rclpy.init(args=args)

    thrusters = Thrusters()

    rclpy.spin(thrusters)
    
    thrusters.destroy_node
    rclpy.shutdown

if __name__ == '__main__':
    main()
