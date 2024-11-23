import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')

        # Create a subscriber for the 'drive' topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10
        )

        # Create a publisher for the modified AckermannDriveStamped message
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def listener_callback(self, msg):
        # Get the speed and steering angle from the received message
        v = msg.drive.speed
        d = msg.drive.steering_angle

        # Multiply both values by 3
        new_v = v * 3
        new_d = d * 3

        # Create a new AckermannDriveStamped message with modified values
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = new_v
        new_msg.drive.steering_angle = new_d

        # Publish the modified message to 'drive_relay' topic
        self.publisher.publish(new_msg)
        self.get_logger().info(f"Relaying: speed={new_v}, steering_angle={new_d}")

def main(args=None):
    rclpy.init(args=args)
    relay = Relay()

    rclpy.spin(relay)

    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
