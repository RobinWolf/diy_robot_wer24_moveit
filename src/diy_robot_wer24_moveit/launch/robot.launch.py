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


def load_yaml(package_name, file_path):
    # TODO make it work with parameter specified package name
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('diy_robotarm_wer24_driver'), 'launch']), "/controller.launch.py"]),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "tf_prefix": tf_prefix
        }.items(),
    )

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
            "use_fake_hardware:=",
            use_fake_hardware,
         ]
    )


    # robot_ip = LaunchConfiguration("robot_ip")
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 
    # robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diy_robot_wer24_moveit"), "srdf", "robot.srdf.xacro"]
            ),
        ]
    )
    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)} 


    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("diy_robot_wer24_moveit"), "config", "kinematics.yaml"]
    )

    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # ompl_planning_yaml = load_yaml("diy_robot_wer24_moveit", "config/ompl_planning.yaml")
    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    
    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("diy_robot_wer24_moveit", "config/controllers.yaml")
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

    # warehouse_ros_config = {
    #     "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #     "warehouse_host": warehouse_sqlite_path,
    # }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2_moveit",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         ompl_planning_pipeline_config,
    #         robot_description_kinematics,
    #         warehouse_ros_config,
    #     ],
    # )

    planning_group = {"planning_group": "wer24_robotarm"}
    moveit_wrapper = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, robot_description_kinematics, planning_group],
    )

    # servo_yaml = load_yaml("ur5_cell_description", "config/ur_servo.yaml")
    # servo_params = {"moveit_servo": servo_yaml}
    # kinematics_yaml = load_yaml("ur5_cell_description", "config/kinematics.yaml")
    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node_main",
    #     parameters=[
    #         servo_params,
    #         robot_description,
    #         robot_description_semantic,
    #         kinematics_yaml
    #     ],
    #     output="screen",
    # )

    # nodes_to_start = [base_launch, move_group_node, rviz_node, moveit_wrapper, servo_node]
    nodes_to_start = [base_launch, move_group_node, moveit_wrapper]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "robot_ip",
    #         default_value="192.168.1.102",
    #         description="IP address by which the robot can be reached.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="aöskflsfighposdf.",
        )
    )



    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
