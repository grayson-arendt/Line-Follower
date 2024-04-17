import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

sys.path.append('/home/rpi/line_follower/src/include')

from PiHatLib import *

class IRPublisher(Node):

    def __init__(self):
        # Create publisher
        super().__init__('ir_publisher')
        self.ir_publisher_ = self.create_publisher(Int32MultiArray, 'ir_topic', 10)
        
        # Create timer
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):

        # Detect IR values
        left_IR = get_ir_state(IR2_INPUT_PIN)
        right_IR = get_ir_state(IR1_INPUT_PIN)

        # Create dictionary for values
        dict_IR = {DARK: 1, LIGHT: 0}

        # Dictionary for ANSI colors (RED, GREEN)
        dict_colors = {DARK: "\033[031m", LIGHT: "\033[032m"}
        dict_string = {DARK: "DARK", LIGHT: "LIGHT"}

        left_state = dict_IR[left_IR]
        right_state = dict_IR[right_IR]

        # Publish IR states
        ir_msg = Int32MultiArray()
        ir_msg.data = [left_state, right_state]

        self.ir_publisher_.publish(ir_msg)

        # This really isn't necessary but I always like color coding terminal output
        left_string = dict_colors[left_IR] + dict_string[left_IR] + "\033[0m"
        right_string = dict_colors[right_IR] + dict_string[right_IR] + "\033[0m"

        self.get_logger().info("LEFT IR: {} | RIGHT IR: {}".format(left_string, right_string))

        

def main(args=None):
    # Start ROS2
    rclpy.init(args=args)

    # Keep running node
    rclpy.spin(IRPublisher())

    # Shutdown ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
