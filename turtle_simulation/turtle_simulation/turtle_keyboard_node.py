import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import threading
import msvcrt  # For Windows keyboard input

class TurtleKeyboardController(Node):
    def __init__(self):
        super().__init__('turtle_keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Default values
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        self.get_logger().info('Turtle Keyboard Controller Node started. Use keyboard to control:')
        self.get_logger().info('W: Forward, S: Stop, X: Backward')
        self.get_logger().info('A: Turn Left, D: Turn Right')
        self.get_logger().info('Q: Exit')

    def timer_callback(self):
        # Create Twist message
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_speed
        
        # Publish the message
        self.publisher_.publish(msg)
        
    def set_movement(self, linear, angular):
        self.linear_speed = linear
        self.angular_speed = angular
        self.get_logger().info(f'Setting speed: linear={linear}, angular={angular}')


def main(args=None):
    rclpy.init(args=args)
    turtle_keyboard_controller = TurtleKeyboardController()
    
    # Simple keyboard control
    def keyboard_input():
        while rclpy.ok():
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                
                if key == 'w':  # Forward
                    turtle_keyboard_controller.set_movement(2.0, 0.0)
                elif key == 's':  # Stop
                    turtle_keyboard_controller.set_movement(0.0, 0.0)
                elif key == 'x':  # Backward
                    turtle_keyboard_controller.set_movement(-2.0, 0.0)
                elif key == 'a':  # Left turn
                    turtle_keyboard_controller.set_movement(0.0, 2.0)
                elif key == 'd':  # Right turn
                    turtle_keyboard_controller.set_movement(0.0, -2.0)
                elif key == 'q':  # Quit
                    break
    
    # Start keyboard input thread
    keyboard_thread = threading.Thread(target=keyboard_input)
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    try:
        rclpy.spin(turtle_keyboard_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        turtle_keyboard_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()