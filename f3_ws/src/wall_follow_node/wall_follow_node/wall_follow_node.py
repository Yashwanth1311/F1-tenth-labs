import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from time import sleep
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
   
        self.scan_subsciption=self.create_subscription(LaserScan,lidarscan_topic,self.scan_callback,5)
        self.publisher = self.create_publisher(AckermannDriveStamped,drive_topic,5)

        # TODO: set PID gains
        self.kp = 0.8
        self.kd = 0.05
        self.ki =0.01

        # TODO: store history
        self.integral = 0
        self.prev_error = 0
        self.error =0 

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        angle=np.pi/4
        range_data=range_data[::-1]
        right_angle_index=int((self.msg.angle_max-np.pi/2)/self.msg.angle_increment)

        thetha_angle_index=int((self.msg.angle_max-np.pi/4)/self.msg.angle_increment)


    

        b=range_data[right_angle_index]

        a=range_data[thetha_angle_index]


        alpha=np.arctan((a*np.cos(angle)-b)/(a*np.sin(angle)))
        
        distance=b*np.cos(alpha)

        L=1.5
        print("alpha",alpha)
        projected_distance=distance+L*np.sin(alpha)

        print(distance,projected_distance)
        return projected_distance

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement

        angle=np.pi/4


        projected_distance=self.get_range(range_data,angle)


       


        error=dist-projected_distance

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0



        self.integral=self.integral+error*0.01

        de_dt=(error-self.prev_error)/0.01



        # TODO: Use kp, ki & kd to implement a PID controller
        control_angle=-(self.kp*error+self.ki*self.integral+self.kd*de_dt)

        self.prev_error=error


        print(error,control_angle*180/np.pi)


        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 1.5
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 1
        else:
            velocity = 0.5


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()  
        drive_msg.drive.steering_angle=control_angle
        drive_msg.drive.speed=velocity
        # TODO: fill in drive message and publish
        self.publisher.publish(drive_msg)    

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.msg=msg
        ranges=np.array(msg.ranges)
        ranges[ranges<msg.range_min]=msg.range_min
        ranges[ranges>msg.range_max]=msg.range_max

        
        error = self.get_error(ranges,1.2) # TODO: replace with error calculated by get_error()
        
        velocity = 1 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()