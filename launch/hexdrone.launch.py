from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ubicoders_hexdrone',
            executable='vdist_pub',
            name='vdist_pub',
            output='screen', 
        ),
        Node(
            package='webcam_img_proc',
            executable='node_webcam',
            name='node_webcam',
            output='screen', 
        ),
        Node(
            package='aruco_ps', 
            executable='body_aruco_node', 
            name='body_aruco_node',  
            output='screen',             
        ),
        Node(
            package="ubicoders_hexdrone",
            executable="hexdrone_ips_pub",
            name="hexdrone_ips_pub",
            output="screen",
        )
    ])