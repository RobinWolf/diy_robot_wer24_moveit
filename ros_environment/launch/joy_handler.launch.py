from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# call this launch file if you want to move your robot around with an xbox controller in the c- space

## !!! CURRENTLY NOT SUPPORTED !!!

# because servoing isnt implemented for the diy hardware

def generate_launch_description():
    joy_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_environment'), 'joy_node.launch.py'])),
    )
    joy_handler = Node(package="ros_environment",
                       executable="joy_handler",
                       output="screen")

    return LaunchDescription([joy_node_launch, joy_handler])
