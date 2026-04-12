import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # find the URDF file
    urdf_file = os.path.join(get_package_share_directory('inverted_pendulum'), 'urdf', 'pendulum.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([
        # Node 1 : Broadcaste the 3D model to the rest of the ROS
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description' : robot_desc}]
        ),
        
        # Node 2 : A GUI slider tool to manually test our joints
        Node(
            package='inverted_pendulum',
            executable='physics_engine'
        ),
        
        # Node 3 : Open RViz for 3D visualization
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])