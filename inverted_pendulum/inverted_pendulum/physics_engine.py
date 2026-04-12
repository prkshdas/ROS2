import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time

class PhysicsEngine(Node):
    def __init__(self):
        super().__init__('physics_engine')
        
        # Publish the joint states to move the 3D model
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribe to /cmd_vel to listen for forces applied to the cart
        self.force_sub = self.create_subscription(Twist, '/cmd_vel', self.force_callback, 10)
        
        # Physical Constants
        self.gravity = 9.81
        self.mass_cart = 1.0
        self.mass_pole = 0.1
        self.total_mass = self.mass_cart + self.mass_pole
        self.length = 0.5 # Half of the pole's length
        self.pole_mass_length = self.mass_pole * self.length
        
        # State Variables (Start with the pole slightly leaning so it falls)
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.1  # 0.1 radians off-center
        self.theta_dot = 0.0
        
        self.applied_force = 0.0
        
        # Run the physics loop at 100Hz (0.01 seconds)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.physics_loop)
        
    def force_callback(self, msg):
        # We will use the linear.x value of the Twist message as our horizontal force
        self.applied_force = msg.linear.x

    def physics_loop(self):
        # --- The Math (Standard Cart-Pole Dynamics) ---
        temp = (self.applied_force + self.pole_mass_length * self.theta_dot**2 * math.sin(self.theta)) / self.total_mass
        
        # Calculate Angular Acceleration (Pole falling)
        theta_acc = (self.gravity * math.sin(self.theta) - math.cos(self.theta) * temp) / (self.length * (4.0/3.0 - self.mass_pole * math.cos(self.theta)**2 / self.total_mass))
        
        # Calculate Linear Acceleration (Cart moving)
        x_acc = temp - self.pole_mass_length * theta_acc * math.cos(self.theta) / self.total_mass
        
        # Euler Integration (Update velocity and position based on acceleration)
        self.x_dot += x_acc * self.dt
        self.x += self.x_dot * self.dt
        self.theta_dot += theta_acc * self.dt
        self.theta += self.theta_dot * self.dt

        # --- Publish the Data to RViz ---
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