import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # Initialize publisher for encoder values
        self.encoder_pub = self.create_publisher(String, 'encoder_data', 10)

        # Initialize subscriber for motor commands
        self.motor_cmd_sub = self.create_subscription(
            String, 'motor_command', self.motor_command_callback, 10)

        # Serial port configuration
        self.serial_port = serial.Serial('/dev/arduino', 115200, timeout=0.1)

        # Timer for periodic tasks
        self.timer = self.create_timer(0.033, self.read_from_arduino)

    def motor_command_callback(self, msg):
        """Send motor commands to the Arduino."""
        try:
            # Parse JSON from the received message
            command = json.loads(msg.data)
            json_cmd = json.dumps(command)
            self.serial_port.write((json_cmd + '\n').encode())
            self.get_logger().info(f'Sent command: {json_cmd}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding command: {e}')
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')

    def read_from_arduino(self):
        """Read encoder data from Arduino and publish it."""
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode().strip()
                # Parse CSV data
                parts = line.split(',')
                if len(parts) == 4:  # Expecting four values
                    encoder_data = {
                        'encoder1_count': int(parts[0]),
                        'encoder2_count': int(parts[1]),
                        'angular_velocity1': float(parts[2]),
                        'angular_velocity2': float(parts[3])
                    }
                    msg = String()
                    msg.data = json.dumps(encoder_data)  # Convert to JSON string
                    self.encoder_pub.publish(msg)
                    self.get_logger().info(f'Received encoder data: {msg.data}')
                else:
                    self.get_logger().error(f'Unexpected data format: {line}')
        except ValueError as e:
            self.get_logger().error(f'Error parsing encoder data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error reading from Arduino: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
