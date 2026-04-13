import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import math

class PhysicsEngine(Node):
    def __init__(self):
        super().__init__('physics_engine')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.force_sub = self.create_subscription(Twist, '/cmd_vel', self.force_callback, 10)
        self.dist_sub = self.create_subscription(Twist, '/disturbance', self.disturbance_callback, 10)
        self.reset_sub = self.create_subscription(Empty, '/reset', self.reset_callback, 10)
        
        self.gravity = 9.81
        self.mass_cart = 1.0
        self.mass_pole = 0.1
        self.total_mass = self.mass_cart + self.mass_pole
        self.length = 0.5 
        self.pole_mass_length = self.mass_pole * self.length
        
        # The URDF track is 2.0 meters long. It goes from -1.0 to 1.0.
        # We set the limit to 0.9 so the cart stays fully on the track.
        self.track_limit = 0.9 
        
        self.reset_state()
        
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.physics_loop)
        
    def reset_state(self):
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.1 
        self.theta_dot = 0.0
        self.applied_force = 0.0

    def force_callback(self, msg):
        self.applied_force = msg.linear.x

    def disturbance_callback(self, msg):
        self.theta_dot += msg.angular.z * 0.5

    def reset_callback(self, msg):
        self.get_logger().info('*** SIMULATION RESET ***')
        self.reset_state()

    def physics_loop(self):
        # --- NEW: Speed Limits (Prevent Math Explosions) ---
        # Cap the pole rotation at 50 radians/sec, and cart speed at 20 m/s
        self.theta_dot = max(min(self.theta_dot, 50.0), -50.0)
        self.x_dot = max(min(self.x_dot, 20.0), -20.0)
        
        temp = (self.applied_force + self.pole_mass_length * self.theta_dot**2 * math.sin(self.theta)) / self.total_mass
        theta_acc = (self.gravity * math.sin(self.theta) - math.cos(self.theta) * temp) / (self.length * (4.0/3.0 - self.mass_pole * math.cos(self.theta)**2 / self.total_mass))
        x_acc = temp - self.pole_mass_length * theta_acc * math.cos(self.theta) / self.total_mass
        
        self.x_dot += x_acc * self.dt
        self.x_dot *= 0.995 # Track Friction
        
        self.x += self.x_dot * self.dt
        
        # --- NEW: The Hard Stops (Boundary Collisions) ---
        if self.x > self.track_limit:
            self.x = self.track_limit
            # 1. Inertia Kick: The base stops, the pole whips forward!
            self.theta_dot += self.x_dot * 1.5 
            # 2. Bounce: The cart bounces off the wall at half speed
            self.x_dot = -self.x_dot * 0.5 
            self.get_logger().warn('!!! CRASHED INTO RIGHT WALL !!!')
            
        elif self.x < -self.track_limit:
            self.x = -self.track_limit
            # 1. Inertia Kick: The base stops, the pole whips forward!
            self.theta_dot += self.x_dot * 1.5 
            # 2. Bounce: The cart bounces off the wall at half speed
            self.x_dot = -self.x_dot * 0.5
            self.get_logger().warn('!!! CRASHED INTO LEFT WALL !!!')
            
        # -------------------------------------------------

        self.theta_dot += theta_acc * self.dt
        self.theta += self.theta_dot * self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['cart_joint', 'pole_joint']
        msg.position = [self.x, self.theta]
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PhysicsEngine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()