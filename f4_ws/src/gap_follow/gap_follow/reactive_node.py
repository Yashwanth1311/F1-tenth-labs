import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from numpy.core.fromnumeric import amax, size
class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
        self.scan_subsciption=self.create_subscription(LaserScan,lidarscan_topic,self.lidar_callback,5)
        self.publisher = self.create_publisher(AckermannDriveStamped,drive_topic,5)


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = list(ranges)
        window_size = 10
        range_index = 0
        while range_index < len(ranges):
            window_sample = proc_ranges[range_index:range_index+window_size]
            window_mean = np.mean(window_sample)
            proc_ranges[range_index:range_index+window_size] = [window_mean] * window_size
            range_index += window_size
        proc_ranges = [5 if i > 5 else i for i in proc_ranges]
        proc_ranges = [0 if i < 0.8 else i for i in proc_ranges]
        return proc_ranges
    
    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        index_list = [0 ,0]
        max_gap_value = 0
        start = True

        # Retrieve all start indeces and end indeces for the gap, the gap or free spaces refers to non-zero points
        for i in range(size(free_space_ranges)):
            if free_space_ranges[i] != 0 and start == True:
                index_list.append(i)
                start = False
            elif free_space_ranges[i] == 0 and start == False:
                index_list.append(i-1)
                start = True

        # Add in the last index if end index have not append into the list above (in case the last element is not zero)
        if len(index_list) % 2 == 1:
            index_list.append(size(free_space_ranges) - 1)

        # Compute the max gap start and end indeces
        max_gap_index = [index_list[0], index_list[1]]
        for i in range(int(len(index_list) / 2)):
                gap_value = index_list[2 * i + 1] - index_list[2 * i]
                if abs(gap_value) >= max_gap_value:
                    max_gap_value = gap_value
                    max_gap_index = [index_list[2 * i], index_list[2 * i +1]]
                
        return max_gap_index

    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        print("----",len(ranges))
        # max_gap_range=ranges[start_i:end_i]
        # index=np.argmax(max_gap_range)
        return start_i+ranges[start_i:end_i+1].index(max(ranges[start_i:end_i+1]))

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        angle_rads = np.arange(data.angle_min, data.angle_max, data.angle_increment)

        ranges = np.array(ranges)

        indexes = np.where((angle_rads < np.pi/2) & (angle_rads > -np.pi/2))

        proc_ranges = ranges[indexes]  # Selecting the ranges based on the condition
        angle_rads=angle_rads[indexes]



        proc_ranges = self.preprocess_lidar(proc_ranges)
        proc_ranges=np.array(proc_ranges)
        
        # TODO:
        #Find closest point to LiDAR

        close_point=np.argmin(proc_ranges)

        bubble_radius=10
        #Eliminate all points inside 'bubble' (set them to zero)
        for i in range(abs(close_point-bubble_radius),close_point+bubble_radius):
            proc_ranges[i] = 0.0

        #Find max length gap 
        start,end=self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        # best_point=self.find_best_point(start,end,list(proc_ranges))
        best_point=int((start+end)/2)
        print(best_point,proc_ranges[best_point])

        angle=angle_rads[best_point]
        # angle=-angle
        print(angle)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()  
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.steering_angle_velocity = 8.0
        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 0.4
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 0.3
        else:
            velocity = 0.2
        drive_msg.drive.speed = velocity
        # TODO: fill in drive message and publish
        self.publisher.publish(drive_msg)    


def main(args=None):
    rclpy.init(args=args)
    print("Reactive node     Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()