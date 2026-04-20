import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
import serial
import math

class IMUBridge(Node):
    def __init__(self):
        super().__init__('imu_serial_bridge')
        
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.calib_sub = self.create_subscription(Empty, '/calibrate', self.calibrate_callback, 10)
        
        self.timer = self.create_timer(0.02, self.read_serial)
        
        # We now track offsets and raw data for BOTH joints
        self.shoulder_offset = 0.0
        self.elbow_offset = 0.0
        self.current_raw_shoulder = 0.0
        self.current_raw_elbow = 0.0

    def calibrate_callback(self, msg):
        # Save current position of both sensors as the new "zero"
        self.shoulder_offset = self.current_raw_shoulder
        self.elbow_offset = self.current_raw_elbow
        self.get_logger().info(f'*** CALIBRATED! Shoulder: {math.degrees(self.shoulder_offset):.1f}°, Elbow: {math.degrees(self.elbow_offset):.1f}° ***')

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                if ',' in line:
                    # Unpack the two values from the Pico
                    shoulder_deg, elbow_deg = map(float, line.split(','))
                    
                    self.current_raw_shoulder = math.radians(shoulder_deg)
                    self.current_raw_elbow = math.radians(elbow_deg)
                    
                    # Apply offsets and limits
                    calib_shoulder = max(min(self.current_raw_shoulder - self.shoulder_offset, 1.57), -1.57)
                    calib_elbow = max(min(self.current_raw_elbow - self.elbow_offset, 1.57), -1.57)

                    # Pack BOTH joints into a single message array
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = ['shoulder_joint', 'elbow_joint'] 
                    msg.position = [calib_shoulder, calib_elbow]
                    
                    self.joint_pub.publish(msg)
                else:
                    self.get_logger().warn(f"Skipped weird serial data: {line}")
                    
            except Exception as e:
                self.get_logger().error(f"Serial Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()