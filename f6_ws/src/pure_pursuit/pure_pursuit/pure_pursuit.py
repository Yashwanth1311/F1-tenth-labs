#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from tf_transformations import euler_from_quaternion
# TODO CHECK: include needed ROS msg type headers and libraries

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
   
        self.scan_subsciption=self.create_subscription(Odometry,"ego_racecar/odom",self.pose_callback,5)
        self.publisher = self.create_publisher(AckermannDriveStamped,drive_topic,5)

        self.way_points=np.genfromtxt(r"./way_points/data.csv", delimiter=',')
        self.way_points[:,-1]=1.0
        self.L=1

        

    def pose_callback(self, pose_msg):
        # print(self.way_points)
        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
                                pose_msg.pose.pose.orientation.y, 
                                pose_msg.pose.pose.orientation.z, 
                                pose_msg.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)

        yaw=euler[2]


        Roation_matrix= [[np.cos(yaw),-np.sin(yaw)],
                        [np.sin(yaw),np.cos(yaw)]]
                        
        displcement=[pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y]
        
        Hom_transformation = np.eye(3)  
        Hom_transformation[:2, :2] = Roation_matrix  
        Hom_transformation[:2, 2] = displcement


        waypoints_car_frame=np.dot(np.linalg.inv(Hom_transformation),self.way_points.T)[:2,:]
        filtered_waypoints = waypoints_car_frame[:, waypoints_car_frame[0, :] > 0]


        disatances=np.linalg.norm(filtered_waypoints, axis=0)

        closest_point=np.argmin(np.absolute(disatances-self.L))

        goal=filtered_waypoints[1,closest_point]


        control_angle = 2 * goal / self.L ** 2


        if 0 <= math.degrees(abs(control_angle)) <= 10:
            velocity = 3.0
        elif 10 < math.degrees(abs(control_angle)) <= 20:
            velocity =2.0
        else:
            velocity = 1.0


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()  
        drive_msg.drive.steering_angle=control_angle
        drive_msg.drive.speed=velocity
        # TODO: fill in drive message and publish
        self.publisher.publish(drive_msg)    

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()