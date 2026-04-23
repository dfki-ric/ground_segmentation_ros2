from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def launch_setup(context, *args, **kwargs):

    # Get the params_file argument (user can override default)
    params_file_arg = LaunchConfiguration("params_file").perform(context)
    
    # If no custom params file provided, use default from package
    if params_file_arg == "default":
        parameters_file = PathJoinSubstitution(
            [FindPackageShare("ground_segmentation_ros2"), "config", "parameters.yaml"]
        )
    else:
        parameters_file = params_file_arg

    ground_segmentation_ros2_node = Node(
        package="ground_segmentation_ros2",
        executable="ground_segmentation_ros2_node",
        parameters=[
            parameters_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }
        ],
        remappings=[
            ("/ground_segmentation/input_pointcloud", LaunchConfiguration("pointcloud_topic")),
            ("/ground_segmentation/input_imu", LaunchConfiguration("imu_topic")),
        ],
        output="screen",
    )

    return [ground_segmentation_ros2_node]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/ground_segmentation/input_pointcloud",
            description="Topic name for the pointcloud",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/ground_segmentation/input_imu",
            description="Topic name for the imu",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (set true for Gazebo / bag playback)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "params_file",
            default_value="default",
            description="Full path to the ground segmentation config file. "
                        "Use 'default' for package default, or provide absolute path to custom config.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
