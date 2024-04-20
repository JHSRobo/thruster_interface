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
        self.logger = self.get_logger()

        # Last thruster values to prevent ESC reset
        self.last_thrusters = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15]

        # Max delta of thrusters
        self.max_delta = 0.004
    # Runs whenever /cmd_vel topic recieves a new twist msg
    # Twist msg reference: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    def thruster_callback(self, msg):
            
            linearX = msg.linear.x
            linearY = msg.linear.y
            linearZ = msg.linear.z
            angularX = msg.angular.x
            angularZ = msg.angular.z

            
            # Decompose the vectors into thruster values
            # linearX references moving along X-axis, or forward
            # angularZ referneces rotation around vertical axis, or Z-axis.
            # For more reference directions, see https://www.canva.com/design/DAFyPqmH8LY/2oMLLaP8HHGi2e07Ms8fug/view
        
            msglist = [linearX - linearY - angularZ, 
                       linearX + linearY + angularZ,
                       -linearX - linearY + angularZ,
                       -linearX + linearY - angularZ,
                       -linearZ - angularX,
                       -linearZ + angularX]
            
            # function to limit a value between -1 and 1
            # min(value, 1) takes the returns lesser of the two values. So if value is greater than 1, it returns 1.
            def limit_value(value):
                return max(-1, min(value, 1))

            # Use map() to apply the limit_value function to each element of msglist
            msglist = list(map(limit_value, msglist))
    
            
            # Goes through each thruster and changes the list values into a duty_cycle
            for i in range(0, 6):
                duty_cycle = 0.15 - msglist[i] / 25

                # Finds the last value of the thruster
                last_cycle = self.last_thrusters[i]
                # On switched direction, first value resets to 0.15
                if duty_cycle < 0.15 < last_cycle or last_cycle < 0.15 < duty_cycle:
                    duty_cycle = 0.15
                # Restricts change in the duty cycle to a delta value
                elif abs(duty_cycle - last_cycle) > self.max_delta:
                    if duty_cycle > last_cycle:
                        duty_cycle = last_cycle + self.max_delta
                    else:
                        duty_cycle = last_cycle - self.max_delta

                # Remembers this thruster for next input
                self.last_thrusters[i] = duty_cycle
                
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
