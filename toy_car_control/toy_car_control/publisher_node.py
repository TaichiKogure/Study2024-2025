import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class ToyCarPublisher(Node):
    def __init__(self):
        super().__init__('toy_car_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Default values
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        self.get_logger().info('Toy Car Publisher Node started. Use keyboard to control:')
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
    toy_car_publisher = ToyCarPublisher()
    
    # Simple keyboard control
    import threading
    import msvcrt  # For Windows
    
    def keyboard_input():
        while rclpy.ok():
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                
                if key == 'w':  # Forward
                    toy_car_publisher.set_movement(0.5, 0.0)
                elif key == 's':  # Stop
                    toy_car_publisher.set_movement(0.0, 0.0)
                elif key == 'x':  # Backward
                    toy_car_publisher.set_movement(-0.5, 0.0)
                elif key == 'a':  # Left turn
                    toy_car_publisher.set_movement(0.0, 0.5)
                elif key == 'd':  # Right turn
                    toy_car_publisher.set_movement(0.0, -0.5)
                elif key == 'q':  # Quit
                    break
    
    # Start keyboard input thread
    keyboard_thread = threading.Thread(target=keyboard_input)
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    try:
        rclpy.spin(toy_car_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        toy_car_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()