import rclpy
import random
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

class LineFollower(Node):
   
    def __init__(self):
        super().__init__('line_follower')
        self.motor_publisher_ = self.create_publisher(Float32MultiArray, 'motor_topic', 10)
        self.ir_subscriber_ = self.create_subscription(Int32MultiArray, 'ir_topic', self.listener_callback, 10)

        # Default parameters
        self.declare_parameter('speed', 0.0)
        self.declare_parameter('direction', True)

    def listener_callback(self, msg):

        motor_speed = self.get_parameter('speed').value
        direction = self.get_parameter('direction').value

        ir_states = msg.data
        left_ir = ir_states[0]
        right_ir = ir_states[1]

        # Forward
        if left_ir == 0 and right_ir == 0 and direction:
            motor_behavior = [motor_speed, 1.0]
        
        # Reverse
        elif left_ir == 0 and right_ir == 0 and not direction:
            motor_behavior = [motor_speed, 2.0]

        # Left turn
        elif left_ir == 0 and right_ir == 1:
            motor_behavior = [motor_speed, 3.0]
        
        # Right turn
        elif left_ir == 1 and right_ir == 0:
            motor_behavior = [motor_speed, 4.0]
        
        else:
            motor_behavior = [motor_speed, 0.0]

        motor_msg = Float32MultiArray()
        motor_msg.data = motor_behavior

        self.motor_publisher_.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    rclpy.shutdown()


if __name__ == '__main__':
    main()