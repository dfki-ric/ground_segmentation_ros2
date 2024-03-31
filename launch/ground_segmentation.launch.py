from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

def launch_setup(context, *args, **kwargs):

    parameters = PathJoinSubstitution(
        [FindPackageShare("ground_segmentation"), "config", "parameters.yaml"]
    )

    ground_segmentation_node = Node(
        package="ground_segmentation",
        executable="ground_segmentation_node",
        parameters=[parameters],
        #arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[('/ground_segmentation/input', LaunchConfiguration('remapped_input'))],
        output="screen",
    )

    nodes_to_start = [ground_segmentation_node]
    
    return nodes_to_start

def generate_launch_description():
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'remapped_input',
            default_value='/ground_segmentation/input',
            description='Topic name to be remapped'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])