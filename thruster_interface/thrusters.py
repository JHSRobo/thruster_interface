#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from core_lib import pca9685

class Thrusters(Node):
    def __init__(self):
        super().__init__('thrusters')

        try: self.pca = pca9685.PCA9685(bus=1)
        except: self.logger.warn("Cannot connect to PCA9685. Ignor this if thrusters are unplugged")
        else:
            self.pca.set_pwm_fequency(100)
            self.pca.output_enable()

            self.pca.channels_set_duty_all(0.15)
            time.sleep(1)

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listen, 10)
        self.subscription
        self.logger = self.get_logger()

    def thruster_callback(self, msg):

            linearX = msg.linear.x
            linearY = msg.linear.y
            linearZ = msg.linear.z
            angularX = msg.angular.x
            angularZ = msg.angular.z

            msglist = {1 : linearX - linearY - angularZ, 
                       2 : linearX + linearY + angularZ,
                       3 : -linearX - linearY + angularZ,
                       4 : -linearX + linearY - angularZ,
                       5 : -linearZ - angularX,
                       6 : -linearZ + angularX}
            
            for i in range(1, 7):
                 self.pca.channel_set_duty(i, 0.15 - msglist[i] / 25)
       

def main(args=None):
    rclpy.init(args=args)

    thrusters = Thrusters()

    rclpy.spin(thrusters)
    
    thrusters.destroy_node
    rclpy.shutdown

if __name__ == '__main__':
    main()
