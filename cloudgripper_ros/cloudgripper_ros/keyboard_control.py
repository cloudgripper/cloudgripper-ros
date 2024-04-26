#!/usr/bin/env python3

from pynput.keyboard import Key, Listener
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int16

class CloudgripperKeyboardController(Node):
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

        # Start the keyboard listener
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()
        self.get_logger().info('CloudGripper Keyboard Controller Node Started')

    def z_callback(self, msg):
        self.current_z = msg.data

    def rotation_callback(self, msg):
        self.current_rotation = msg.data

    def gripper_callback(self, msg):
        self.current_gripper = msg.data

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                self.handle_char_keys(key.char)
            elif key in (Key.up, Key.down, Key.left, Key.right):
                self.handle_special_keys(key)
        except Exception as e:
            self.get_logger().error(f'Error processing key press: {str(e)}')

    def handle_char_keys(self, char):
            msg = String() 
            if char == 'w':
                msg.data = 'forward'
                self.pub_step_command.publish(msg)
            elif char == 's':
                msg.data = 'backward'
                self.pub_step_command.publish(msg)
            elif char == 'a':
                msg.data = 'left'
                self.pub_step_command.publish(msg)
            elif char == 'd':
                msg.data = 'right'
                self.pub_step_command.publish(msg)
            elif char == 'z':
                self.adjust_gripper(-1.0)
            elif char == 'x':
                self.adjust_gripper(1.0)

    def handle_special_keys(self, key):
        if key == Key.up:
            self.adjust_z_movement(0.1)  # Increase Z position slightly
        elif key == Key.down:
            self.adjust_z_movement(-0.1) # Decrease Z position slightly
        elif key == Key.left:
            self.adjust_rotation(-5)      # Rotate left by 5 degrees
        elif key == Key.right:
            self.adjust_rotation(5)
    
    def adjust_z_movement(self, delta):
        msg = Float32()
        self.current_z += delta
        if self.current_z >=1.0 : self.current_z=1.0
        if self.current_z <=0.0 : self.current_z=0.0
        msg.data = self.current_z
        self.pub_z_movement.publish(msg)

    def adjust_rotation(self, angle_change):
        msg = Int16()
        self.current_rotation += angle_change
        if self.current_rotation >=180 : self.current_rotation=180
        if self.current_rotation <=0 : self.current_rotation=0
        msg.data = self.current_rotation
        self.get_logger().info('send rot')
        self.pub_rotation.publish(msg)

    def adjust_gripper(self, change):
        msg = Float32()
        self.current_gripper += change
        if self.current_gripper >=1.0 : self.current_gripper=1.0
        if self.current_gripper <=0.0: self.current_gripper=0.0
        msg.data = self.current_gripper
        self.pub_gripper.publish(msg)


    def on_shutdown(self):
        self.listener.stop()
        self.get_logger().info('Shutting down Keyboard Controller Node')

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = CloudgripperKeyboardController()
    
    try:
        rclpy.spin(keyboard_controller)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_controller.on_shutdown()
        keyboard_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
