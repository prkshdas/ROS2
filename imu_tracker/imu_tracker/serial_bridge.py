import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math

class IMUBridge(Node):
    def __init__(self):
        super().__init__('imu_serial_bridge')
        
        # Listening to the Pico
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                # Check if the line actually has a comma before trying to split it
                if ',' in line:
                    roll_deg, pitch_deg = map(float, line.split(','))
                    
                    pitch_rad = math.radians(pitch_deg)
                    pitch_rad = max(min(pitch_rad, 1.57), -1.57)

                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = ['elbow_joint'] 
                    msg.position = [pitch_rad]
                    
                    self.joint_pub.publish(msg)
                else:
                    self.get_logger().warn(f"Skipped weird serial data: {line}")
                    
            except Exception as e:
                # NEVER use 'pass' here! We need to see the errors.
                self.get_logger().error(f"Serial Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()