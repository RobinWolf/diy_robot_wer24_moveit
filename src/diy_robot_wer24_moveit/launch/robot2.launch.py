import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition


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
            "rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file, should be deactivated when launching moveit from this base image.",
    )
  )


    tf_prefix = LaunchConfiguration("tf_prefix")
    tf_prefix_sub = LaunchConfiguration("tf_prefix_sub")
    tf_prefix_arm = LaunchConfiguration("tf_prefix_arm")
    tf_prefix_grip = LaunchConfiguration("tf_prefix_grip")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_ssid = LaunchConfiguration("robot_ssid")

    rviz = LaunchConfiguration("rviz")



    ###################################################################
    ##                         URDF Configuration                    ##
    ###################################################################
    description_package = "diy_robot_full_cell_description"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "cell_model.urdf.xacro"]),
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


    ###################################################################
    ##                       MoveIt Configuration                    ##
    ###################################################################

    #configure the SRDF
    moveit_package = "diy_robot_wer24_moveit"
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_package), "srdf", "robot.srdf.xacro"]),
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

    #robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)} 

    #configure the kinematics solver
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "config", "kinematics.yaml"]
    )

    #configure the planning algorithms such as A-*, RRT, PRM
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(moveit_package, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    

    # Trajectory Execution Configuration of the trajectories controllers
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

    #define the move group node
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

    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_package), "rviz", "rviz_config.rviz"]) # define path to rviz-config file
    
    rviz_node = Node(
        package="rviz2",
        #condition=IfCondition(rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics],
    )


    #define the planning group node
    planning_group = {"planning_group": "wer24_robotarm"}
    moveit_wrapper = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics, planning_group],
    )


    #calls the launch file from the driver package
    arm_driver_package = "diy_robotarm_wer24_driver"

    base_rviz = "false"

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(arm_driver_package), 'launch']), "/controller.launch.py"]),
        launch_arguments={
            "tf_prefix": tf_prefix,
            "tf_prefix_sub": tf_prefix_sub,
            "tf_prefix_arm": tf_prefix_arm,
            "tf_prefix_grip": tf_prefix_grip,
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "robot_ssid": robot_ssid,
            "rviz": base_rviz,
        }.items(),
    )
    nodes_to_start = [base_launch, rviz_node, move_group_node, moveit_wrapper]

    return LaunchDescription(declared_arguments + nodes_to_start)




