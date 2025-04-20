import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Subscribe to pose topic to get turtle position
        self.pose_subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        
        # Current pose of the turtle
        self.current_pose = Pose()
        
        self.get_logger().info('Turtle Controller Node started')
    
    def pose_callback(self, msg):
        # Store the current pose
        self.current_pose = msg
        
        # Log position occasionally (not every update to avoid flooding)
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter % 50 == 0:  # Log every 50 updates
                self.get_logger().info(f'Turtle position: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')
        else:
            self.log_counter = 0
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Log received command
        self.get_logger().info(f'Received cmd_vel: linear.x={linear_x}, angular.z={angular_z}')
        
        # Here you would implement any additional control logic
        # For turtlesim, the cmd_vel is directly used by the turtle
        # so we don't need to do anything else
        
        # Check if turtle is approaching the boundary
        self.check_boundary()
    
    def check_boundary(self):
        # turtlesim has a 11x11 world with (0,0) at the bottom left
        # and (11,11) at the top right
        margin = 0.5  # margin from the edge
        
        if self.current_pose.x < margin or self.current_pose.x > 11.0 - margin or \
           self.current_pose.y < margin or self.current_pose.y > 11.0 - margin:
            self.get_logger().warn('Turtle is approaching the boundary!')


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        turtle_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()