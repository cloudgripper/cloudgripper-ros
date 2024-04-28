import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cloudgripper_msgs.srv import GetCameraImage  # Adjust this to your actual service definition

class DualCameraPublisher(Node):
    def __init__(self):
        super().__init__('dual_camera_publisher')
        # Client to request images
        self.client = self.create_client(GetCameraImage, '/get_camera_image')
        self.bridge = CvBridge()
        # Publishers for each camera image
        self.base_publisher = self.create_publisher(Image, '/base_camera/image', 10)
        self.top_publisher = self.create_publisher(Image, '/top_camera/image', 10)
        
        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Service not available, waiting again...')

    def request_and_publish_image(self, camera_type):
        req = GetCameraImage.Request()
        req.camera_type = camera_type
        future = self.client.call_async(req)
        future.add_done_callback(lambda future: self.handle_response(future, camera_type))

    def handle_response(self, future, camera_type):
        try:
            response = future.result()
            if response.success:
                # Convert the received ROS image message to an OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(response.image, desired_encoding='bgr8')
                # Convert the OpenCV image back to a ROS image message for publishing
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                # Publish the image on the appropriate topic
                if camera_type == 'base':
                    self.base_publisher.publish(ros_image)
                    self.get_logger().info('Base camera image published.')
                elif camera_type == 'top':
                    self.top_publisher.publish(ros_image)
                    self.get_logger().info('Top camera image published.')
            else:
                self.get_logger().info(f'Failed to receive image from {camera_type} camera: ' + response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraPublisher()
    node.request_and_publish_image('base')
    node.request_and_publish_image('top')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
