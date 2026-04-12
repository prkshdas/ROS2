import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math

class PhysicsEngine(Node):
    def __init__(self):
        super().__init__('physics_engine')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.force_sub = self.create_subscription(Twist, '/cmd_vel', self.force_callback, 10)
        
        # --- UPDATED: Listen to the built-in ROS 2 slider tool ---
        self.dist_sub = self.create_subscription(Twist, '/disturbance', self.disturbance_callback, 10)
        
        self.gravity = 9.81
        self.mass_cart = 1.0
        self.mass_pole = 0.1
        self.total_mass = self.mass_cart + self.mass_pole
        self.length = 0.5 
        self.pole_mass_length = self.mass_pole * self.length
        
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.1 
        self.theta_dot = 0.0
        self.applied_force = 0.0
        
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.physics_loop)
        
    def force_callback(self, msg):
        self.applied_force = msg.linear.x

    # --- UPDATED: The Wind/Slider Function ---
    def disturbance_callback(self, msg):
        # The slider goes from -1 to 1. We multiply it to make the "wind" stronger.
        # This pushes the pole continuously while the slider is moved.
        self.theta_dot += msg.angular.z * 0.5

    def physics_loop(self):
        temp = (self.applied_force + self.pole_mass_length * self.theta_dot**2 * math.sin(self.theta)) / self.total_mass
        theta_acc = (self.gravity * math.sin(self.theta) - math.cos(self.theta) * temp) / (self.length * (4.0/3.0 - self.mass_pole * math.cos(self.theta)**2 / self.total_mass))
        x_acc = temp - self.pole_mass_length * theta_acc * math.cos(self.theta) / self.total_mass
        
        self.x_dot += x_acc * self.dt
        self.x_dot *= 0.995 # Track Friction
        
        self.x += self.x_dot * self.dt
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