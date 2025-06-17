from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "name", package_name="my_moveit_config"
    ).to_moveit_configs()

    # Start perception node immediately
    perception_node = TimerAction(period=10.0,actions=[Node(
        name="perception",
        package="object_detection",
        executable="object_detection",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )])
    
    # Delay launching pick_and_place by 10 seconds
    delayed_pick_and_place = TimerAction(
        period=15.0,
        actions=[
            Node(
                name="pick_and_place",
                package="moveit2_scripts",
                executable="pick_and_place_perception",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': True},
                ],
            )
        ]
    )

    return LaunchDescription([
        perception_node,
        Node(
            package="object_detection",
            executable="static_transform_publisher",
            output="screen"
        ),
        delayed_pick_and_place
    ])
