#include <moveit_wrapper/moveit_wrapper.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;


namespace moveit_wrapper
{
    MoveitWrapper::MoveitWrapper(const rclcpp::NodeOptions &options) : Node("moveit_wrapper", options)
    {
        _i_move_group_initialized = false;
        this->get_parameter("planning_group", _planning_group);
        _pose_target_lin_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _pose_target_ptp_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _joint_position_target_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        _reset_planning_group_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        _move_to_pose_lin = this->create_service<moveit_wrapper::srv::MoveToPose>("move_to_pose_lin", std::bind(&MoveitWrapper::move_to_pose_lin, this, _1, _2), rmw_qos_profile_services_default, _pose_target_lin_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_lin service initialized.");

        _move_to_pose_ptp = this->create_service<moveit_wrapper::srv::MoveToPose>("move_to_pose_ptp", std::bind(&MoveitWrapper::move_to_pose_ptp, this, _1, _2), rmw_qos_profile_services_default, _pose_target_ptp_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_ptp service initialized.");

        _move_to_joint_position = this->create_service<moveit_wrapper::srv::MoveToJointPosition>("move_to_joint_position", std::bind(&MoveitWrapper::move_to_joint_position, this, _1, _2), rmw_qos_profile_services_default, _joint_position_target_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_joint_position service initialized.");

        _reset_planning_group = this->create_service<moveit_wrapper::srv::String>("reset_planning_group", std::bind(&MoveitWrapper::reset_planning_group, this, _1, _2), rmw_qos_profile_services_default, _reset_planning_group_group);
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "reset_planning_group service initialized.");
//        _move_to_pose = this->create_service<moveit_wrapper::srv::MoveToPose>("move_to_pose", std::bind(&MoveitWrapper::move_to_pose, this, _1, _2));
//        _move_to_joint_position = this->create_service<moveit_wrapper::srv::MoveToJointPosition>("move_to_joint_position", std::bind(&MoveitWrapper::move_to_joint_position, this, _1, _2));
        rclcpp::Rate loop_rate(1);
        loop_rate.sleep();

        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Initialized.");
    }


    void MoveitWrapper::init_move_group()
    {
        static const std::string PLANNING_GROUP = _planning_group;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), PLANNING_GROUP.c_str());
        _move_group.reset(new moveit::planning_interface::MoveGroupInterface(shared_from_this(), _planning_group));

        _i_move_group_initialized = true;
        rclcpp::Rate loop_rate(1000);
        loop_rate.sleep();
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "Ready to receive commands.");
    }


    void MoveitWrapper::reset_planning_group(const std::shared_ptr<moveit_wrapper::srv::String::Request> request,
                std::shared_ptr<moveit_wrapper::srv::String::Response> response)
    {
        _i_move_group_initialized = false;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), request->data.c_str());
        _planning_group = request->data.c_str();
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), _planning_group.c_str());
        _move_group->stop();
        _move_group->clearPoseTargets();
        init_move_group();
        response->success = true;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "reset_planning_group callback executed.");
    }


    void MoveitWrapper::move_to_pose_lin(const std::shared_ptr<moveit_wrapper::srv::MoveToPose::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToPose::Response> response)
    {
        bool success = false;
        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();

            // Set the maximum velocity scaling factor
            double scaling = request->velocityscaling
            _move_group->setMaxVelocityScalingFactor(request->velocityscaling);

            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(request->pose);


            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.001;                   
            double fraction = _move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            
            // Rescale the timestamps of the trajectory based on the velocity scaling factor
            rescaleTrajectory(trajectory, scaling);

            if(fraction > 0.0) {
                success = true;
                //_move_group->execute(trajectory);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                my_plan.trajectory_ = trajectory;
                _move_group->execute(my_plan);
            }
        }
        response->success = success;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_lin callback executed.");
    }


    void MoveitWrapper::move_to_pose_ptp(const std::shared_ptr<moveit_wrapper::srv::MoveToPose::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToPose::Response> response)
    {
        bool success = false;
        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();

            // Set the maximum velocity scaling factor
            _move_group->setMaxVelocityScalingFactor(request->velocityscaling);

            _move_group->setPoseTarget(request->pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            success = (_move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success) {
                _move_group->move();
            }
        }
        response->success = success;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_pose_ptp callback executed.");
    }


    void MoveitWrapper::move_to_joint_position(const std::shared_ptr<moveit_wrapper::srv::MoveToJointPosition::Request> request,
                std::shared_ptr<moveit_wrapper::srv::MoveToJointPosition::Response> response)
    {
        bool success = false;
        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();

            success = ptp_joint(request->joint_position);
        }
        response->success = success;
        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "move_to_joint_position callback executed.");
    }


    bool MoveitWrapper::ptp_joint(const std::vector<double> joint_position)
    {
        bool success = false;
        if(_i_move_group_initialized)
        {
            _move_group->stop();
            _move_group->clearPoseTargets();
            _move_group->setJointValueTarget(joint_position);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            success = (_move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success) {
                _move_group->move();
            }
        }
        return success;
    }

    void rescaleTrajectory(moveit_msgs::RobotTrajectory& trajectory, double velocity_scaling) {

        double time_scaling = 1.0 / velocity_scaling;

        // Loop through each point in the trajectory and rescale the timestamps
        for (auto& point : trajectory.joint_trajectory.points) {
            // Scale the time_from_start of each trajectory point
            point.time_from_start *= scaling_factor;
        }

        RCLCPP_INFO(rclcpp::get_logger("moveit_wrapper"), "timestamp rescaling executed.");
    }
}