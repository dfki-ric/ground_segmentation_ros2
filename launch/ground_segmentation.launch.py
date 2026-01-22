from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    parameters = PathJoinSubstitution(
        [FindPackageShare("ground_segmentation_ros2"), "config", "parameters.yaml"]
    )

    ground_segmentation_ros2_node = Node(
        package="ground_segmentation_ros2",
        executable="ground_segmentation_ros2_node",
        parameters=[parameters],
        #arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[('/ground_segmentation/input_pointcloud', LaunchConfiguration('pointcloud_topic')),
                    ('/ground_segmentation/input_imu', LaunchConfiguration('imu_topic'))],
        output="screen",
    )

    nodes_to_start = [ground_segmentation_ros2_node]
    
    return nodes_to_start

def generate_launch_description():
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/ground_segmentation/input_pointcloud',
            description='Topic name for the pointcloud'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/ground_segmentation/input_imu',
            description='Topic name for the imu'
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])