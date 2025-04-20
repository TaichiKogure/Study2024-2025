import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
import tf_transformations

class GazeboTurtleNode(Node):
    def __init__(self):
        super().__init__('gazebo_turtle_node')
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Subscribe to Gazebo model odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            'turtle/odom',
            self.odom_callback,
            10)
        
        # Publisher for Gazebo model velocity
        self.gazebo_vel_publisher = self.create_publisher(
            Twist,
            'turtle/cmd_vel',
            10)
        
        # Current pose of the turtle
        self.current_pose = Pose()
        
        self.get_logger().info('Gazebo Turtle Node started')
    
    def cmd_vel_callback(self, msg):
        # Forward the command velocity to Gazebo
        self.gazebo_vel_publisher.publish(msg)
        
        # Log received command
        self.get_logger().info(f'Forwarding cmd_vel to Gazebo: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
    
    def odom_callback(self, msg):
        # Extract position and orientation from odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        
        # Log position occasionally (not every update to avoid flooding)
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
            if self.log_counter % 50 == 0:  # Log every 50 updates
                self.get_logger().info(f'Turtle position: x={position.x:.2f}, y={position.y:.2f}, theta={yaw:.2f}')
        else:
            self.log_counter = 0


def main(args=None):
    rclpy.init(args=args)
    gazebo_turtle_node = GazeboTurtleNode()
    
    try:
        rclpy.spin(gazebo_turtle_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        gazebo_turtle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()