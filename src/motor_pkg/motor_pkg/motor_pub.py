import rclpy
import random
from rclpy.node import Node

from std_msgs.msg import Int32


class SpeedPublisher(Node):
   
    def __init__(self):
        super().__init__('speed_publisher')
        self.publisher_ = self.create_publisher(Int32, 'motor_speed', 10)
        timer_period = 5.0  # publish every 5 seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()

        # On first run, test the boundaries
        if self.i == 0:
            speed = 500
        
        else:
            # Generate numbers between -100 and 100
            speed = random.randint(-100, 100)

        msg.data = speed

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing speed: %i' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    speed_publisher = SpeedPublisher()

    rclpy.spin(speed_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()