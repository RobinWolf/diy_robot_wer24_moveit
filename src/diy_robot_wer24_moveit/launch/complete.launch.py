import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue



def load_yaml(package_name, file_path):
    # TODO make it work with parameter specified package name
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    


def generate_launch_description():
#declare launch arguments (can be passed in the command line while launching)
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for the links and joints in the robot cell",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_sub",
            default_value='"sub_"',
            description="Prefix for the subframe of the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_arm",
            default_value='"arm_"',
            description="Prefix for the robotarm inside the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_grip",
            default_value='"grip_"',
            description="Prefix for the gripper inside the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value='"true"',
            description="start the robot with fake (mock) hardware or real controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value='"192.168.212.203"',
            description="The IP-Adress with which the robot hardware joins the common network",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ssid",
            default_value='"DIY-Robotics"',
            description="The SSID from the common network (PC and ESP must be member of this network)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file, This Rviz includes motion planning",
    )
  )
    

    # Initialize Arguments
    tf_prefix = LaunchConfiguration("tf_prefix")
    tf_prefix_sub = LaunchConfiguration("tf_prefix_sub")
    tf_prefix_arm = LaunchConfiguration("tf_prefix_arm")
    tf_prefix_grip = LaunchConfiguration("tf_prefix_grip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_ssid = LaunchConfiguration("robot_ssid")
    moveit_rviz = LaunchConfiguration("moveit_rviz")


    #define used packages
    description_package = "diy_robot_full_cell_description"
    arm_driver_package = "diy_robotarm_wer24_driver"
    moveit_package = "diy_robot_wer24_moveit"


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "joint_limits.yaml"]
    )
    initial_positions_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "initial_positions.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "cell_model.urdf.xacro"]),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "initial_positions_params:=",
            initial_positions_params,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "tf_prefix_sub:=",
            tf_prefix_sub,
            " ",
            "tf_prefix_arm:=",
            tf_prefix_arm,
            " ",
            "tf_prefix_grip:=",
            tf_prefix_grip,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_ssid:=",
            robot_ssid,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
         ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_package), "srdf", "robot.srdf.xacro"]),
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "tf_prefix_sub:=",
                tf_prefix_sub,
                " ",
                "tf_prefix_arm:=",
                tf_prefix_arm,
                " ",
                "tf_prefix_grip:=",
                tf_prefix_grip,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "config", "kinematics.yaml"]
    )


    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml(moveit_package, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(moveit_package, "config/controllers.yaml")


    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_package), "rviz", "rviz_config.rviz"]) # define path to rviz-config file

    
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(moveit_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
        ],
    )


    #launch the driver with the controller.launch.py script from drivers package (as dependencie inside this container)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(arm_driver_package), 'launch']), "/controller.launch.py"]),
            launch_arguments={
                "tf_prefix": tf_prefix,
                "tf_prefix_sub": tf_prefix_sub,
                "tf_prefix_arm": tf_prefix_arm,
                "tf_prefix_grip": tf_prefix_grip,
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "robot_ssid": robot_ssid,   #the rviz argument is not passed, because the rviz from driver package (without moveit) should not be launched here in all cases (default = false)
            }.items(),
    )

    planning_group = {"planning_group": "wer24_robotarm"}
    moveit_wrapper = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics, planning_group],
    )



    nodes_to_start = [driver_launch, move_group_node, rviz_node, moveit_wrapper]


    return LaunchDescription(declared_arguments + nodes_to_start)