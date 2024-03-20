import rclpy
from rclpy.node import Node
import requests
from cloudgripper_msgs.msg import RobotState
import os

class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')
        robot_name = self.declare_parameter('robot_name', '1').get_parameter_value().string_value

        self.publisher_state = self.create_publisher(RobotState, '/robot_state', 10)
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.api_key = os.environ["CLOUDGRIPPER_API_KEY"]
        self.base_url = f"https://cloudgripper.zahidmhd.com/{robot_name}/api/v1.1/robot"

    def timer_callback(self):
        payload = {}
        headers = {'apiKey': self.api_key}
        url = self.base_url + "/getState"
        try:
            response = requests.get(url, headers=headers, data={}, timeout=0.5)
            if response.status_code == 200:
                data = response.json()['state']
                msg = RobotState()
                msg.x_norm = data["x_norm"]
                msg.y_norm = data["y_norm"]
                msg.z_norm = data["z_norm"]
                msg.rotation = data["rotation"]
                msg.claw_norm = data["claw_norm"]
                msg.z_current = float(data["z_current"])
                msg.rotation_current = float(data["rotation_current"])
                msg.claw_current = float(data["claw_current"])
                msg.timestamp = response.json()['timestamp']
                self.publisher_state.publish(msg)
                self.get_logger().info('Robot state published')
            else:
                self.get_logger().error('Failed to fetch robot state')
        except Exception as e:
            self.get_logger().error(f'Exception during request: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    robot_state_publisher = RobotStatePublisher()
    rclpy.spin(robot_state_publisher)
    robot_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
