import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Int16
from std_msgs.msg import String
import requests
import os

class RobotControlSubscriber(Node):

    def __init__(self):
        super().__init__('robot_control_subscriber')
        robot_name = self.declare_parameter('robot_name', '1').get_parameter_value().string_value
        self.api_key = os.environ["CLOUDGRIPPER_API_KEY"]
        self.base_url = f"https://cloudgripper.zahidmhd.com/{robot_name}/api/v1.1/robot"
        self.subscription_xy = self.create_subscription(
            Point,
            '/xy',
            self.xy_callback,
            10)
        self.subscription_z = self.create_subscription(
            Float32,
            '/z',
            self.z_callback,
            10)
        self.subscription_gripper = self.create_subscription(
            Float32,
            '/gripper_value',
            self.gripper_callback,
            10)
        self.subscription_rotation = self.create_subscription(
            Int16,
            '/rotation_angle',
            self.rotation_callback,
            10)
        self.subscription_step_movements = self.create_subscription(
            String,
            '/step_command',
            self.step_command_callback,
            10)

    def xy_callback(self, msg):
        # Handle x and y values
        self.get_logger().info(f'X: {msg.x}, Y: {msg.y}')
        payload = {}
        headers = {'apiKey': self.api_key}
        url = self.base_url + f'/gcode/{msg.x}/{msg.y}'
        try:
            response = requests.get(url, headers=headers, data={}, timeout=0.5)
            if response.status_code == 200:
                self.get_logger().info(f'Successfully sent GCode with x: {msg.x}, y: {msg.y}')
            else:
                self.get_logger().error(f'Failed to send GCode: HTTP {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Exception during request: {str(e)}')

    def z_callback(self, msg):
        # Handle z value
        self.get_logger().info(f'Z: {msg.data}')
        payload = {}
        headers = {'apiKey': self.api_key}
        url = self.base_url + f'/up_down/{msg.data}'
        try:
            response = requests.get(url, headers=headers, data={}, timeout=0.5)
            if response.status_code == 200:
                self.get_logger().info(f'Successfully sent Z-movement with z: {msg.data}')
            else:
                self.get_logger().error(f'Failed to send Z-movement: HTTP {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Exception during request: {str(e)}')

    def gripper_callback(self, msg):
        # Handle gripper value
        self.get_logger().info(f'Gripper Value: {msg.data}')
        payload = {}
        headers = {'apiKey': self.api_key}
        url = self.base_url + f'/grip/{msg.data}'
        try:
            response = requests.get(url, headers=headers, data={}, timeout=0.5)
            if response.status_code == 200:
                self.get_logger().info(f'Successfully sent Grip position with val: {msg.data}')
            else:
                self.get_logger().error(f'Failed to send Grip position: HTTP {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Exception during request: {str(e)}')

    def rotation_callback(self, msg):
        # Handle rotation angle
        self.get_logger().info(f'Rotation Angle: {msg.data}')
        payload = {}
        headers = {'apiKey': self.api_key}
        url = self.base_url + f'/rotate/{msg.data}'
        try:
            response = requests.get(url, headers=headers, data={}, timeout=0.5)
            if response.status_code == 200:
                self.get_logger().info(f'Successfully sent Rotation with z: {msg.data}')
            else:
                self.get_logger().error(f'Failed to send Rotation: HTTP {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Exception during request: {str(e)}')

    def step_command_callback(self, msg):
        # Handle step commands
        self.get_logger().info(f'Step Direction: {msg.data}')
        payload = {}
        headers = {'apiKey': self.api_key}
        url = self.get_url_for_step_command(msg.data)
        if url:
            try:
                response = requests.get(url, headers=headers, data={}, timeout=0.5)
                if response.status_code == 200:
                    self.get_logger().info(f'Successfully sent Step Command: {msg.data}')
                else:
                    self.get_logger().error(f'Failed to send Step Command: HTTP {response.status_code}')
            except Exception as e:
                self.get_logger().error(f'Exception during request: {str(e)}')
    
    def get_url_for_step_command(self, command):
        command_to_endpoint = {
            "right": "/moveRight",
            "left": "/moveLeft",
            "forward": "/moveUp",
            "backward": "/moveDown",
        }

        endpoint = command_to_endpoint.get(command.lower())
        if endpoint:
            return f"{self.base_url}{endpoint}"
        else:
            self.get_logger().error(f'Invalid command: {command}')
            return None


def main(args=None):
    rclpy.init(args=args)
    robot_control_subscriber = RobotControlSubscriber()
    rclpy.spin(robot_control_subscriber)
    robot_control_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
