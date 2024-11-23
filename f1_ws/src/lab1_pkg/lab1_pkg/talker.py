import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped 

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.declare_parameter('v', 0.0)  # speed parameter
        self.declare_parameter('d', 0.0)  # steering angle parameter


        self.publisher=self.create_publisher(AckermannDriveStamped,"drive",10)

        self.timer=self.create_timer(0.2,self.publish_message)

    def publish_message(self):
        v=self.get_parameter("v").value
        d=self.get_parameter("d").value

        msg=AckermannDriveStamped()
        msg.drive.speed=v
        msg.drive.steering_angle=d


        self.publisher.publish(msg)

        self.get_logger().info(f"speed={v}, steering_angle={d}")

def main(args=None):
    rclpy.init(args=args)

    talker=Talker()

    rclpy.spin(talker)

    rclpy.destroy_node()
    rclpy.shut_down()
if __name__=="__main__":
    main()