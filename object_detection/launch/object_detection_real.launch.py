from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([Node(package="object_detection",executable="object_detection_real",output="screen"),Node(package="object_detection",executable="static_transform_publisher_real",output="screen"),Node(package="rviz2",executable="rviz2",arguments=["-d","/home/user/ros2_ws/src/manipulation_project/rviz_real_visualizer.rviz"])])