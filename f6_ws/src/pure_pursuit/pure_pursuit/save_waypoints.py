#!/usr/bin/env python3
from time import gmtime, strftime
from rclpy.node import Node
import rclpy
import tf2_geometry_msgs 
import numpy as np
from numpy import linalg as LA
# import tf
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
# TODO CHECK: include needed ROS msg type headers and libraries
file = open(strftime('./way_points/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
   
        self.odom_subsciption=self.create_subscription(Odometry,"ego_racecar/odom",self.save_waypoint,10)
       

    def save_waypoint(self, data):


        quaternion = np.array([data.pose.pose.orientation.x, 
                                data.pose.pose.orientation.y, 
                                data.pose.pose.orientation.z, 
                                data.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)

        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                data.twist.twist.linear.y, 
                                data.twist.twist.linear.z]),2)
        if data.twist.twist.linear.x>0.:
            print(data.twist.twist.linear.x)

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y,
                                        euler[2],
                                        speed))


def main(args=None):
    rclpy.init(args=args)
    print("save points Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()