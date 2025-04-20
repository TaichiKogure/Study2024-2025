import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Import RPi.GPIO if running on Raspberry Pi
try:
    import RPi.GPIO as GPIO
    ON_PI = True
except ImportError:
    ON_PI = False
    print("RPi.GPIO not available. Running in simulation mode.")

# Motor control pins (adjust as needed for your hardware setup)
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
RIGHT_MOTOR_FORWARD = 22
RIGHT_MOTOR_BACKWARD = 23
PWM_FREQUENCY = 100  # Hz

class ToyCarController(Node):
    def __init__(self):
        super().__init__('toy_car_controller')
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('Toy Car Controller Node started')
        
        # Initialize GPIO if on Raspberry Pi
        if ON_PI:
            self.setup_gpio()
        else:
            self.get_logger().info('Running in simulation mode (not on Raspberry Pi)')
    
    def setup_gpio(self):
        # Set GPIO mode
        GPIO.setmode(GPIO.BCM)
        
        # Setup motor pins as output
        GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
        GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
        GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
        GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)
        
        # Setup PWM for speed control
        self.left_forward_pwm = GPIO.PWM(LEFT_MOTOR_FORWARD, PWM_FREQUENCY)
        self.left_backward_pwm = GPIO.PWM(LEFT_MOTOR_BACKWARD, PWM_FREQUENCY)
        self.right_forward_pwm = GPIO.PWM(RIGHT_MOTOR_FORWARD, PWM_FREQUENCY)
        self.right_backward_pwm = GPIO.PWM(RIGHT_MOTOR_BACKWARD, PWM_FREQUENCY)
        
        # Start PWM with 0% duty cycle (stopped)
        self.left_forward_pwm.start(0)
        self.left_backward_pwm.start(0)
        self.right_forward_pwm.start(0)
        self.right_backward_pwm.start(0)
        
        self.get_logger().info('GPIO initialized')
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Log received command
        self.get_logger().info(f'Received cmd_vel: linear.x={linear_x}, angular.z={angular_z}')
        
        # Convert to motor speeds
        left_speed, right_speed = self.calculate_motor_speeds(linear_x, angular_z)
        
        # Apply motor speeds
        self.set_motor_speeds(left_speed, right_speed)
    
    def calculate_motor_speeds(self, linear_x, angular_z):
        # Simple differential drive calculation
        # Positive linear_x means forward, negative means backward
        # Positive angular_z means turn left, negative means turn right
        
        # Scale values to motor speed range (-100 to 100)
        max_linear = 0.5  # m/s
        max_angular = 0.5  # rad/s
        
        # Normalize to range -1.0 to 1.0
        linear_factor = max(-1.0, min(1.0, linear_x / max_linear))
        angular_factor = max(-1.0, min(1.0, angular_z / max_angular))
        
        # Calculate left and right wheel speeds
        left_speed = 100.0 * (linear_factor - angular_factor)
        right_speed = 100.0 * (linear_factor + angular_factor)
        
        # Ensure speeds are within -100 to 100 range
        left_speed = max(-100.0, min(100.0, left_speed))
        right_speed = max(-100.0, min(100.0, right_speed))
        
        return left_speed, right_speed
    
    def set_motor_speeds(self, left_speed, right_speed):
        if not ON_PI:
            # Simulation mode - just log the speeds
            self.get_logger().info(f'Setting motor speeds: left={left_speed}, right={right_speed}')
            return
        
        # Set left motor
        if left_speed > 0:  # Forward
            self.left_forward_pwm.ChangeDutyCycle(left_speed)
            self.left_backward_pwm.ChangeDutyCycle(0)
        else:  # Backward or stop
            self.left_forward_pwm.ChangeDutyCycle(0)
            self.left_backward_pwm.ChangeDutyCycle(abs(left_speed))
        
        # Set right motor
        if right_speed > 0:  # Forward
            self.right_forward_pwm.ChangeDutyCycle(right_speed)
            self.right_backward_pwm.ChangeDutyCycle(0)
        else:  # Backward or stop
            self.right_forward_pwm.ChangeDutyCycle(0)
            self.right_backward_pwm.ChangeDutyCycle(abs(right_speed))
    
    def cleanup(self):
        if ON_PI:
            # Stop motors
            self.left_forward_pwm.ChangeDutyCycle(0)
            self.left_backward_pwm.ChangeDutyCycle(0)
            self.right_forward_pwm.ChangeDutyCycle(0)
            self.right_backward_pwm.ChangeDutyCycle(0)
            
            # Cleanup GPIO
            self.left_forward_pwm.stop()
            self.left_backward_pwm.stop()
            self.right_forward_pwm.stop()
            self.right_backward_pwm.stop()
            GPIO.cleanup()
            
            self.get_logger().info('GPIO cleaned up')


def main(args=None):
    rclpy.init(args=args)
    toy_car_controller = ToyCarController()
    
    try:
        rclpy.spin(toy_car_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        toy_car_controller.cleanup()
        toy_car_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()