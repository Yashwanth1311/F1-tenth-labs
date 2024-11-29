#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed=0.0
        # TODO: create ROS subscribers and publishers.
        self.odom_subsciption=self.create_subscription(Odometry,"ego_racecar/odom",self.odom_callback,10)
        self.scan_subsciption=self.create_subscription(LaserScan,"scan",self.scan_callback,10)


        self.publisher = self.create_publisher(AckermannDriveStamped,"drive",10)



    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x


    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        ranges=np.array(scan_msg.ranges)
        ranges[ranges<scan_msg.range_min]=scan_msg.range_min
        ranges[ranges>scan_msg.range_max]=scan_msg.range_max
        angle_radians=np.arange(scan_msg.angle_min,scan_msg.angle_max,scan_msg.angle_increment)
        range_rates=np.cos(angle_radians)*self.speed+1e-6

        TTCs=np.array([ranges/range_rates])
        TTCs[TTCs<0]=1e+6

 
        # # TODO: publish command to brake
        
        if np.min(TTCs)<1.2:
            new_msg=AckermannDriveStamped()
            new_msg.drive.speed=0.0
            # new_msg.drive.steering_angle=0.0
            self.publisher.publish(new_msg)        

            print("BREAK!!!!")
        # elif np.min(TTCs)<4:
        
        #     new_msg=AckermannDriveStamped()


        #     new_msg.drive.speed=0.1
        #     new_msg.drive.steering_angle=0.0

        #     self.publisher.publish(new_msg)

        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()