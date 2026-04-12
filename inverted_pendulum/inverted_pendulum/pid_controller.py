import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # 1. Listen to the Physics Engine to get the current pole angle
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10)
            
        # 2. Command the Physics Engine by sending pushing forces
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- The Magic Numbers (PID Gains) ---
        # These determine how aggressively the robot tries to balance.
        # You will likely need to tune these!
        self.kp = 150.0   # Proportional: Push harder the further it falls
        self.ki = 0.0     # Integral: Push to fix long-term leaning
        self.kd = 20.0    # Derivative: Push against the *speed* of the fall to dampen it
        
        self.target_angle = 0.0  # 0 radians = perfectly upright
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def state_callback(self, msg):
        # Find the pole's angle from the incoming message
        try:
            pole_idx = msg.name.index('pole_joint')
            current_angle = msg.position[pole_idx]
        except ValueError:
            return

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            return

        # Calculate how far off we are from straight up
        error = self.target_angle - current_angle

        # --- The PID Math ---
        p_term = self.kp * error
        
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        d_term = self.kd * (error - self.prev_error) / dt

        # Calculate total force (Negative because we have to push *under* the falling pole)
        force = -(p_term + i_term + d_term)

        # Save values for the next loop
        self.prev_error = error
        self.last_time = current_time

        # Send the force command to the physics engine
        cmd = Twist()
        cmd.linear.x = force
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()