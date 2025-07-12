#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Robot parameters
WHEELBASE = 0.5  # Distance between wheels (adjust to your robot's value)

# Initial wheel angular velocity limits
wheel_velocity_max = 1.0  # Maximum angular velocity of each wheel
wheel_velocity_min = 0.55 # Minimum angular velocity of each wheel

# Key bindings for robot motion
MOVE_BINDINGS = {
    'w': (1, 0),  # Forward
    's': (-1, 0), # Backward
    'a': (0, 1),  # Left turn
    'd': (0, -1), # Right turn
}

def get_key():
    """
    Capture keyboard input in a non-blocking way.
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def calculate_linear_angular(wheel_vel_min, wheel_vel_max, wheelbase):
    """
    Calculate linear and angular velocity bounds based on wheel angular velocity limits.
    """
    linear_max = (wheel_vel_max + wheel_vel_min) / 2.0
    linear_min = (wheel_vel_min + wheel_vel_min) / 2.0
    angular_max = (wheel_vel_max - wheel_vel_min) / wheelbase
    angular_min = (wheel_vel_min - wheel_vel_max) / wheelbase
    return linear_min, linear_max, angular_min, angular_max

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')

        # Create a publisher for the /diff_cont/cmd_vel_unstamped topic
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        # Initialize velocities
        self.linear = 0.0
        self.angular = 0.0

        # Display instructions
        self.display_instructions()

    def display_instructions(self):
        print("Control Your Robot!")
        print("Use 'WASD' to move, 'CTRL+C' to quit.")
        print("Adjust velocity limits:")
        print("'z': Increase max wheel velocity")
        print("'x': Decrease max wheel velocity")
        print("'c': Increase min wheel velocity")
        print("'v': Decrease min wheel velocity")

    def run(self):
        global wheel_velocity_min, wheel_velocity_max

        try:
            while rclpy.ok():
                key = get_key()
                if key in MOVE_BINDINGS:
                    linear_input, angular_input = MOVE_BINDINGS[key]

                    # Recalculate velocity limits based on the current settings
                    linear_min, linear_max, angular_min, angular_max = calculate_linear_angular(
                        wheel_velocity_min, wheel_velocity_max, WHEELBASE)

                    # Scale input commands to match calculated limits
                    self.linear = linear_input * (linear_max if linear_input > 0 else linear_min)
                    self.angular = angular_input * (angular_max if angular_input > 0 else angular_min)
                elif key in ['z', 'x', 'c', 'v']:  # Adjust wheel velocity limits
                    self.adjust_limits(key)
                    print(f"Updated wheel velocity limits: Min = {wheel_velocity_min}, Max = {wheel_velocity_max}")
                elif key == '\x03':  # CTRL+C
                    break
                else:
                    self.linear = 0.0
                    self.angular = 0.0

                # Publish the velocity command
                twist = Twist()
                twist.linear.x = self.linear
                twist.angular.z = self.angular
                self.publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # Stop the robot on exit
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

    def adjust_limits(self, key, step=0.05):
        """
        Adjust the maximum and minimum angular velocities of the wheels.
        """
        global wheel_velocity_min, wheel_velocity_max
        if key == 'z':  # Increase max velocity
            wheel_velocity_max += step
        elif key == 'x':  # Decrease max velocity
            wheel_velocity_max = max(0.55, wheel_velocity_max - step)
        elif key == 'c':  # Increase min velocity
            wheel_velocity_min = min(wheel_velocity_min + step, wheel_velocity_max)
        elif key == 'v':  # Decrease min velocity
            wheel_velocity_min = max(0.55, wheel_velocity_min - step)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.run()

    # Shutdown
    teleop_node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
