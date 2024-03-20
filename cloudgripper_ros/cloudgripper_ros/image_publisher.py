import rclpy
from rclpy.node import Node
import requests
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import io
import os

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        robot_name = self.declare_parameter('robot_name', '1').get_parameter_value().string_value
        self.publisher_base = self.create_publisher(Image, '/base_camera/image', 10)
        self.publisher_top = self.create_publisher(Image, '/top_camera/image', 10)

        self.base_url = f"https://cloudgripper.zahidmhd.com/{robot_name}/api/v1.1/robot"
        self.timer_period = 0.5  # seconds (fetch and publish every 5 seconds)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.api_key = os.environ["CLOUDGRIPPER_API_KEY"]

    def timer_callback(self):
        # Fetch and publish the base camera image
        self.fetch_and_publish_image(endpoint='/getImageBase', publisher=self.publisher_base)
        # Fetch and publish the top camera image
        self.fetch_and_publish_image(endpoint='/getImageTop', publisher=self.publisher_top)

    def fetch_and_publish_image(self, endpoint, publisher):
        payload = {}
        headers = {'apiKey': self.api_key}  
        try:
            response = requests.get(self.base_api + endpoint, headers=headers, data={}, timeout=0.5)
            if response.status_code == 200:
                response = response.json()
                getimage = response['data']
                time_stamp = response['time']
                encode_img = getimage.encode('latin1')
                img = base64.b64decode(encode_img)
                npimg = np.frombuffer(img, dtype=np.uint8)  
                source = cv2.imdecode(npimg, 1) 

                if source is not None:
                    ros_image = self.bridge.cv2_to_imgmsg(source, encoding="bgr8")
                    ros_image.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_frame")
                    publisher.publish(ros_image)
                    self.get_logger().info(f'Image published at ROS time {ros_image.header.stamp} on {publisher.topic_name}')
                else:
                    self.get_logger().error('Could not decode image from ' + endpoint)
            else:
                self.get_logger().error(f'Failed to fetch image: HTTP {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Exception fetching or decoding image from {endpoint}: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
