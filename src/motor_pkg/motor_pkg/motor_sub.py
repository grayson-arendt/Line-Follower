import rclpy
import sys
import time
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
sys.path.append('/home/rpi/line_follower/src/include')

from PiHatLib import *

class MotorSubscriber(Node):

    def __init__(self):
        # Create subscriber
        super().__init__('motor_subscriber')
        self.motor_subscription = self.create_subscription(Float32MultiArray, 'motor_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        leftMotor = Motor('MOTOR1', 1)
        rightMotor = Motor('MOTOR3', 1)

        motor_speed = msg.data[0]
        # Turning 10% faster
        turn_speed = motor_speed + 10
        motor_behavior = int(msg.data[1])

        # ANSI escape codes: RED, GREEN, YELLOW, BLUE, MAGENTA
        colors_dict = {0: "\033[031m", 1: "\033[032m", 2: "\033[033m", 3: "\033[034m", 4: "\033[035m"}

        # Python 3.10 has switch cases now
        match motor_behavior:
            case 0:
                motor_string = "STOP"

                leftMotor.stop()
                rightMotor.stop()
                time.sleep(0.1)

            case 1:
                motor_string = "DRIVING FORWARD"

                leftMotor.forward(motor_speed)
                rightMotor.forward(motor_speed)
                time.sleep(0.1)

            case 2:
                motor_string = "REVERSING"

                leftMotor.reverse(motor_speed)
                rightMotor.reverse(motor_speed)
                time.sleep(0.1)

            case 3:
                motor_string = "TURNING LEFT"

                leftMotor.stop()
                rightMotor.forward(turn_speed)
                time.sleep(0.1)

            case 4:
                motor_string = "TURNING RIGHT"

                leftMotor.forward(turn_speed)
                rightMotor.stop()
                time.sleep(0.1)

        # Not necessary, but I like coloring terminal output
        logging_string = colors_dict[motor_behavior] + motor_string + "\033[0m"
        self.get_logger().info("SPEED: {} | MOTOR BEHAVIOR: {}".format(motor_speed, logging_string))



def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = MotorSubscriber()

    rclpy.spin(motor_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()