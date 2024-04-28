import rclpy
from rclpy.node import Node
import requests
import cv2
import numpy as np
import base64
import os

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from cloudgripper_msgs.srv import GetCameraImage  # Replace with your actual package name

class ImageService(Node):
    def __init__(self):
        super().__init__('image_service')
        robot_name = self.declare_parameter('robot_name', '1').get_parameter_value().string_value
        self.service = self.create_service(GetCameraImage, 'get_camera_image', self.handle_image_request)
        self.base_url = f"https://cloudgripper.zahidmhd.com/{robot_name}/api/v1.1/robot"
        self.api_key = os.environ["CLOUDGRIPPER_API_KEY"]
        self.bridge = CvBridge()

    def handle_image_request(self, request, response):
        camera_type = request.camera_type
        endpoint = '/getImageBase' if camera_type == 'base' else '/getImageTop'
        headers = {'apiKey': self.api_key}

        try:
            api_response = requests.get(self.base_url + endpoint, headers=headers, timeout=3)
            if api_response.status_code == 200:
                data = api_response.json()['data']
                img_data = base64.b64decode(data.encode('latin1'))
                np_img = np.frombuffer(img_data, dtype=np.uint8)
                cv_image = cv2.imdecode(np_img, 1)
                
                if cv_image is not None:
                    ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                    ros_image.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_frame")
                    response.image = ros_image
                    response.success = True
                    response.message = "Image fetched successfully"
                else:
                    response.success = False
                    response.message = "Image decode failed"
            else:
                response.success = False
                response.message = f"API call failed with status: {api_response.status_code}"
        except Exception as e:
            response.success = False
            response.message = f"Exception occurred: {str(e)}"

        return response

def main(args=None):
    rclpy.init(args=args)
    image_service = ImageService()
    rclpy.spin(image_service)
    image_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
