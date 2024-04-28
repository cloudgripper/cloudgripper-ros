import rclpy
from rclpy.node import Node
import requests
from cloudgripper_msgs.srv import GetRobotState
from cloudgripper_msgs.msg import RobotState
import os

class RobotStateService(Node):

    def __init__(self):
        super().__init__('robot_state_service')
        robot_name = self.declare_parameter('robot_name', '1').get_parameter_value().string_value

        self.service = self.create_service(GetRobotState, 'get_robot_state', self.handle_get_robot_state)
        self.api_key = os.environ["CLOUDGRIPPER_API_KEY"]
        self.base_url = f"https://cloudgripper.zahidmhd.com/{robot_name}/api/v1.1/robot"

    def handle_get_robot_state(self, request, response):
        payload = {}
        headers = {'apiKey': self.api_key}
        url = self.base_url + "/getState"
        try:
            resp = requests.get(url, headers=headers, timeout=0.5)
            if resp.status_code == 200:
                data = resp.json()['state']
                response.robot_state.x_norm = data["x_norm"]
                response.robot_state.y_norm = data["y_norm"]
                response.robot_state.z_norm = data["z_norm"]
                response.robot_state.rotation = data["rotation"]
                response.robot_state.claw_norm = data["claw_norm"]
                response.robot_state.z_current = float(data["z_current"])
                response.robot_state.rotation_current = float(data["rotation_current"])
                response.robot_state.claw_current = float(data["claw_current"])
                response.robot_state.timestamp = resp.json()['timestamp']

                response.success = True
                self.get_logger().info('Robot state retrieved and sent')
            else:
                self.get_logger().error('Failed to fetch robot state')
                response.success = False
        except Exception as e:
            self.get_logger().error(f'Exception during request: {str(e)}')
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    robot_state_service = RobotStateService()
    rclpy.spin(robot_state_service)
    robot_state_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
