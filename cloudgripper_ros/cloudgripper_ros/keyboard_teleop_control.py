import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int16
from geometry_msgs.msg import Twist

class CloudgripperKeyboardControl(Node):
    def __init__(self):
        super().__init__('cloudgripper_keyboard_controller')
        # Publishers for controls
        self.pub_step_command = self.create_publisher(String, '/step_command', 10)
        self.pub_z_movement = self.create_publisher(Float32, '/z', 10)
        self.pub_gripper = self.create_publisher(Float32, '/gripper_value', 10)
        self.pub_rotation = self.create_publisher(Int16, '/rotation_angle', 10)

        # Current state variables
        self.current_z = 0.0
        self.current_rotation = 180
        self.current_gripper = 0.0

        # Subscribers to update current states
        self.create_subscription(Float32, '/z', self.z_callback, 10)
        self.create_subscription(Float32, '/gripper_value', self.gripper_callback, 10)
        self.create_subscription(Int16, '/rotation_angle', self.rotation_callback, 10)

        # Subscriber to listen to /cmd_vel topic
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def z_callback(self, msg):
        self.current_z = msg.data

    def rotation_callback(self, msg):
        self.current_rotation = msg.data

    def gripper_callback(self, msg):
        self.current_gripper = msg.data

    def cmd_vel_callback(self, msg):
        # Step command based on linear.x velocity
        if msg.linear.x > 0 and msg.angular.z == 0:
            self.publish_step_command('forward')  # 'i' pressed
        elif msg.linear.x == 0 and msg.angular.z == 0 and msg.linear.z == 0:
            self.publish_step_command('backward')  # 'k'pressed
        elif msg.angular.z > 0 and msg.linear.x == 0:
            self.publish_step_command('left')  # 'j' pressed
        elif msg.angular.z < 0 and msg.linear.x == 0:
            self.publish_step_command('right')  # 'l' pressed

        # Rotation adjustment
        if msg.linear.x > 0 and msg.angular.z < 0:
            self.adjust_rotation(5) # 'o' pressed
        elif msg.linear.x > 0 and msg.angular.z > 0:
            self.adjust_rotation(-5) # 'u' pressed
 

        # Gripper adjustment 
        if msg.linear.x < 0 and msg.angular.z < 0:
            self.adjust_gripper(-1.0) # 'm' pressed
        elif msg.linear.x < 0 and msg.angular.z == 0:
            self.adjust_gripper(1.0)  # ',' pressed

        # Z movement
        if msg.linear.z > 0:
            self.adjust_z_movement(0.1)  # 't' pressed
        elif msg.linear.z < 0:
            self.adjust_z_movement(-0.1)  # 'b' pressed


    def publish_step_command(self, direction):
        msg = String()
        msg.data = direction
        self.pub_step_command.publish(msg)
        self.get_logger().info(f'Published step command: {direction}')

    def adjust_z_movement(self, delta):
        msg = Float32()
        self.current_z += delta
        self.current_z = max(0.0, min(1.0, self.current_z)) 
        msg.data = self.current_z
        self.pub_z_movement.publish(msg)

    def adjust_rotation(self, angle_change):
        msg = Int16()
        self.current_rotation += angle_change
        self.current_rotation = max(0, min(180, self.current_rotation)) 
        msg.data = self.current_rotation
        self.get_logger().info('Published rotation')
        self.pub_rotation.publish(msg)

    def adjust_gripper(self, change):
        msg = Float32()
        self.current_gripper += change
        self.current_gripper = max(0.0, min(1.0, self.current_gripper))  # Clamp gripper value within bounds
        msg.data = self.current_gripper
        self.pub_gripper.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CloudgripperKeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
