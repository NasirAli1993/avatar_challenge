from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import OpaqueFunction

def wait_for_user_input(context):
    input(" RViz should be open now. Manually add the 'visualization_marker_array' topic, then press Enter to continue...\n")
    
    draw_shapes_node = Node(
        package="avatar_challenge",
        executable="draw_shapes",
        name="draw_shapes",
        output="screen"
    )
    return [draw_shapes_node]
    

def generate_launch_description():
    xarm_config = FindPackageShare("xarm_moveit_config")
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                xarm_config,
                "launch",
                "xarm7_moveit_fake.launch.py",
            ])
        )
    )

    wait_input_after_delay = TimerAction(
        period=8.0,
        actions=[OpaqueFunction(function=wait_for_user_input)]
    )

    

    return LaunchDescription([
        xarm_moveit_launch,
        wait_input_after_delay,
        
    ])
