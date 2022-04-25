#include "locobot_arms/locobot_arms_action_server.hpp"

namespace locobot_arms
{
    LocobotArmsActionServer::LocobotArmsActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : arm_planning_group(ARM_GROUP),
          gripper_planning_group(GRIPPER_GROUP)
    {
        node_ = rclcpp::Node::make_shared("locobot_arms_actions", options);
        using namespace std::placeholders;
        this->joint_server_ = rclcpp_action::create_server<ArmJointPlanner>(
            node_,
            "locobot_joint_action",
            std::bind(&LocobotArmsActionServer::handle_joint_goal, this, _1, _2),
            std::bind(&LocobotArmsActionServer::handle_joint_cancel, this, _1),
            std::bind(&LocobotArmsActionServer::handle_joint_accepted, this, _1));

        this->pose_server_ = rclcpp_action::create_server<ArmPosePlanner>(
            node_,
            "locobot_pose_action",
            std::bind(&LocobotArmsActionServer::handle_pose_goal, this, _1, _2),
            std::bind(&LocobotArmsActionServer::handle_pose_cancel, this, _1),
            std::bind(&LocobotArmsActionServer::handle_pose_accepted, this, _1));

        this->gripper_server_ = rclcpp_action::create_server<GripperPlanner>(
            node_,
            "locobot_gripper_action",
            std::bind(&LocobotArmsActionServer::handle_gripper_goal, this, _1, _2),
            std::bind(&LocobotArmsActionServer::handle_gripper_cancel, this, _1),
            std::bind(&LocobotArmsActionServer::handle_gripper_accepted, this, _1));
        
        this->name_server_ = rclcpp_action::create_server<ArmNamePosPlanner>(
            node_,
            "locobot_name_pos_action",
            std::bind(&LocobotArmsActionServer::handle_name_goal, this, _1, _2),
            std::bind(&LocobotArmsActionServer::handle_name_cancel, this, _1),
            std::bind(&LocobotArmsActionServer::handle_name_accepted, this, _1));

        joint_check_server_ = node_->create_service<raya_arms_msgs::srv::ArmJointPlannerCheck>("locobot_joint_check", std::bind(&LocobotArmsActionServer::joint_check, this, _1, _2));
        pose_check_server_ = node_->create_service<raya_arms_msgs::srv::ArmPosePlannerCheck>("locobot_pose_check", std::bind(&LocobotArmsActionServer::pose_check, this, _1, _2));
    }

    void LocobotArmsActionServer::init_moveit_groups()
    {
        move_group_arm = new moveit::planning_interface::MoveGroupInterface(node_, arm_planning_group);
        move_group_gripper = new moveit::planning_interface::MoveGroupInterface(node_, gripper_planning_group);
        arm_joint_model_group = move_group_arm->getCurrentState()->getJointModelGroup(arm_planning_group);
        gripper_joint_model_group = move_group_gripper->getCurrentState()->getJointModelGroup(gripper_planning_group);

        move_group_arm->setMaxVelocityScalingFactor(1);
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_arm->getPlanningFrame();

        // The id of the object is used to identify it.
        collision_object.id = "table_camera";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.005;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.16;

        // Define a pose for the box (specified relative to frame_id).
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.0025;

        shape_msgs::msg::SolidPrimitive primitive1;
        primitive1.type = primitive.BOX;
        primitive1.dimensions.resize(3);
        primitive1.dimensions[primitive.BOX_X] = 0.06;
        primitive1.dimensions[primitive.BOX_Y] = 0.10;
        primitive1.dimensions[primitive.BOX_Z] = 0.13;

        // Define a pose for the box (specified relative to frame_id).
        geometry_msgs::msg::Pose box_pose1;
        box_pose1.orientation.w = 1.0;
        box_pose1.position.x = -0.02;
        box_pose1.position.y = -0.115;
        box_pose1.position.z = 0.065;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_object.primitives.push_back(primitive1);
        collision_object.primitive_poses.push_back(box_pose1);

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        planning_scene_interface.addCollisionObjects(collision_objects);
    }

    rclcpp_action::GoalResponse LocobotArmsActionServer::handle_joint_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ArmJointPlanner::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received joint goal request for the arm : %s", goal->arm.data());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse LocobotArmsActionServer::handle_joint_cancel(
        const std::shared_ptr<GoalHandleArmJointPlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel joint goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LocobotArmsActionServer::handle_joint_accepted(const std::shared_ptr<GoalHandleArmJointPlanner> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly RCLCPP_INFO(node_->get_logger(), "Executing finished...");to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LocobotArmsActionServer::execute_joint, this, _1), goal_handle}.detach();
    }

    void LocobotArmsActionServer::execute_joint(const std::shared_ptr<GoalHandleArmJointPlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ArmJointPlanner::Feedback>();
        auto result = std::make_shared<ArmJointPlanner::Result>();
        const std::string PLANNING_GROUP = goal->arm;
        moveit::planning_interface::MoveGroupInterface *move_group;
        moveit::planning_interface::MoveItErrorCode success;
        bool correct_group = true;
        bool correct_number_joints = false;

        if (goal->arm.compare(arm_planning_group) == 0)
            move_group = move_group_arm;
        else if (goal->arm.compare(gripper_planning_group) == 0)
            move_group = move_group_gripper;
        else
            correct_group = false;

        if (correct_group)
        {
            if (move_group->setJointValueTarget(goal->position))
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                correct_number_joints = true;
                success = move_group->plan(my_plan);
                if (success == success.SUCCESS)
                {
                    auto initial_joints = move_group->getCurrentJointValues();
                    bool is_complete = false;
                    auto future = std::async([move_group, &my_plan, &is_complete]
                                             {auto success=move_group->execute(my_plan); is_complete=true; return success; });
                    while (!is_complete)
                    {
                        auto actual_joints = move_group->getCurrentJointValues();
                        feedback->arm = PLANNING_GROUP;
                        feedback->percentage_complete = get_progress(initial_joints, goal->position, actual_joints);
                        goal_handle->publish_feedback(feedback);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    success = future.get();
                }
            }
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            convert_result(success, &result->error, correct_group, correct_number_joints);
            goal_handle->succeed(result);
            RCLCPP_INFO(node_->get_logger(), "Goal joint succeeded");
        }
    }

    rclcpp_action::GoalResponse LocobotArmsActionServer::handle_name_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ArmNamePosPlanner::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received name goal request for the arm : %s", goal->arm.data());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse LocobotArmsActionServer::handle_name_cancel(
        const std::shared_ptr<GoalHandleArmNamePosPlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel name goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LocobotArmsActionServer::handle_name_accepted(const std::shared_ptr<GoalHandleArmNamePosPlanner> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly RCLCPP_INFO(node_->get_logger(), "Executing finished...");to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LocobotArmsActionServer::execute_name, this, _1), goal_handle}.detach();
    }

    void LocobotArmsActionServer::execute_name(const std::shared_ptr<GoalHandleArmNamePosPlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ArmNamePosPlanner::Feedback>();
        auto result = std::make_shared<ArmNamePosPlanner::Result>();
        const std::string PLANNING_GROUP = goal->arm;
        moveit::planning_interface::MoveGroupInterface *move_group;
        moveit::planning_interface::MoveItErrorCode success;
        bool correct_group = true;
        bool correct_number_joints = false;

        if (goal->arm.compare(arm_planning_group) == 0)
            move_group = move_group_arm;
        else if (goal->arm.compare(gripper_planning_group) == 0)
            move_group = move_group_gripper;
        else
            correct_group = false;

        if (correct_group)
        {
            if (move_group->setNamedTarget(goal->name))
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                correct_number_joints = true;
                success = move_group->plan(my_plan);
                if (success == success.SUCCESS)
                {
                    auto initial_joints = move_group->getCurrentJointValues();
                    bool is_complete = false;
                    auto future = std::async([move_group, &my_plan, &is_complete]
                                             {auto success=move_group->execute(my_plan); is_complete=true; return success; });
                    while (!is_complete)
                    {
                        auto actual_joints = move_group->getCurrentJointValues();
                        feedback->arm = PLANNING_GROUP;
                        std::vector<double> joint_values;
                        moveit::core::RobotStatePtr kinematic_state = move_group->getCurrentState(10);
                        auto kinematic_model = move_group->getJointValueTarget();
                        const auto joint_model_group = kinematic_model.getJointModelGroup(PLANNING_GROUP);
                        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
                        feedback->percentage_complete = get_progress(initial_joints,joint_values, actual_joints);
                        goal_handle->publish_feedback(feedback);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    success = future.get();
                }
            }
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            convert_result(success, &result->error, correct_group, correct_number_joints);
            goal_handle->succeed(result);
            RCLCPP_INFO(node_->get_logger(), "Goal joint succeeded");
        }
    }

    rclcpp_action::GoalResponse LocobotArmsActionServer::handle_gripper_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const GripperPlanner::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received gripper goal request for the gripper");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse LocobotArmsActionServer::handle_gripper_cancel(
        const std::shared_ptr<GoalHandleGripperPlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel gripper goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LocobotArmsActionServer::handle_gripper_accepted(const std::shared_ptr<GoalHandleGripperPlanner> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly RCLCPP_INFO(node_->get_logger(), "Executing finished...");to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LocobotArmsActionServer::execute_gripper, this, _1), goal_handle}.detach();
    }

    void LocobotArmsActionServer::execute_gripper(const std::shared_ptr<GoalHandleGripperPlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing gripper goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GripperPlanner::Feedback>();
        auto result = std::make_shared<GripperPlanner::Result>();
        moveit::planning_interface::MoveGroupInterface *move_group;
        moveit::planning_interface::MoveItErrorCode success;
        bool correct_group = true;
        bool correct_number_joints = true;

        move_group = move_group_gripper;
        double distance = (goal->width / 2) - 0.0125;
        if (distance < MIN_GRIPPER)
            distance = MIN_GRIPPER;
        if (distance > MAX_GRIPPER)
            distance = MAX_GRIPPER;

        std::vector<double> joints;
        joints.push_back(distance);
        joints.push_back(-distance);
        move_group->setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        success = move_group->plan(my_plan);
        if (success == success.SUCCESS)
        {
            auto initial_joints = move_group->getCurrentJointValues();
            bool is_complete = false;
            auto future = std::async([move_group, &my_plan, &is_complete]
                                     {auto success=move_group->execute(my_plan); is_complete=true; return success; });
            while (!is_complete)
            {
                auto actual_joints = move_group->getCurrentJointValues();
                feedback->arm = GRIPPER_GROUP;
                feedback->percentage_complete = get_progress(initial_joints, joints, actual_joints);
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            success = future.get();
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            convert_result(success, &result->error, correct_group, correct_number_joints);
            goal_handle->succeed(result);
            RCLCPP_INFO(node_->get_logger(), "Goal joint succeeded");
        }
    }

    rclcpp_action::GoalResponse LocobotArmsActionServer::handle_pose_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ArmPosePlanner::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received pose goal request for the arm : %s", goal->arm.data());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse LocobotArmsActionServer::handle_pose_cancel(
        const std::shared_ptr<GoalHandleArmPosePlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel pose goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LocobotArmsActionServer::handle_pose_accepted(const std::shared_ptr<GoalHandleArmPosePlanner> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly RCLCPP_INFO(node_->get_logger(), "Executing finished...");to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LocobotArmsActionServer::execute_pose, this, _1), goal_handle}.detach();
    }

    void LocobotArmsActionServer::execute_pose(const std::shared_ptr<GoalHandleArmPosePlanner> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ArmPosePlanner::Feedback>();
        auto result = std::make_shared<ArmPosePlanner::Result>();
        const std::string PLANNING_GROUP = goal->arm;
        moveit::planning_interface::MoveGroupInterface *move_group;
        moveit::planning_interface::MoveItErrorCode success;
        bool correct_group = true;

        if (goal->arm.compare(arm_planning_group) == 0)
            move_group = move_group_arm;
        else if (goal->arm.compare(gripper_planning_group) == 0)
            move_group = move_group_gripper;
        else
            correct_group = false;

        if (correct_group)
        {
            bool is_complete = false;
            if (!goal->cartesian_path)
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;

                move_group->setPoseTarget(goal->goal_pose);
                success = move_group->plan(my_plan);

                if (success == success.SUCCESS)
                {
                    auto initial_pose = move_group->getCurrentPose();

                    auto future = std::async([move_group, &my_plan, &is_complete]
                                             {auto success=move_group->execute(my_plan); is_complete=true; return success; });
                    while (!is_complete)
                    {
                        auto actual_pose = move_group->getCurrentPose();
                        feedback->arm = PLANNING_GROUP;
                        feedback->percentage_complete = get_progress(initial_pose, goal->goal_pose, actual_pose);
                        goal_handle->publish_feedback(feedback);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    success = future.get();
                }
            }
            else
            {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(goal->goal_pose);

                moveit_msgs::msg::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                if (fraction != -1)
                {
                    auto initial_pose = move_group->getCurrentPose();

                    auto future = std::async([move_group, &trajectory, &is_complete]
                                             {auto success=move_group->execute(trajectory); is_complete=true; return success; });
                    while (!is_complete)
                    {
                        auto actual_pose = move_group->getCurrentPose();
                        feedback->arm = PLANNING_GROUP;
                        feedback->percentage_complete = get_progress(initial_pose, goal->goal_pose, actual_pose);
                        goal_handle->publish_feedback(feedback);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    success = future.get();
                }
            }
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            convert_result(success, &result->error,
                           correct_group);
            goal_handle->succeed(result);
            RCLCPP_INFO(node_->get_logger(), "Goal pose succeeded");
        }
    }

    void LocobotArmsActionServer::convert_result(moveit::planning_interface::MoveItErrorCode success,
                                                 uint8_t *error,
                                                 bool correct_group)
    {
        if (correct_group)
        {
            switch (success.val)
            {
            case success.SUCCESS:
                *error = raya_arms_msgs::action::ArmPosePlanner_Result::SUCCESS;
                break;
            case success.FAILURE:
                *error = raya_arms_msgs::action::ArmPosePlanner_Result::FAILURE;
                break;
            case 0:
                *error = raya_arms_msgs::action::ArmPosePlanner_Result::FAILURE;
                break;
            default:
                *error = (uint8_t)(success.val * -1);
                break;
            }
        }
        else
        {
            *error = raya_arms_msgs::action::ArmPosePlanner_Result::INVALID_GROUP_NAME;
        }
    }

    void LocobotArmsActionServer::convert_result(moveit::planning_interface::MoveItErrorCode success,
                                                 uint8_t *error,
                                                 bool correct_group,
                                                 bool correct_number_joints)
    {
        if (!correct_number_joints)
        {
            *error = raya_arms_msgs::action::ArmJointPlanner_Result::INVALID_NUMBER_JOINTS;
        }
        else
        {
            convert_result(success, error, correct_group);
        }
    }

    double LocobotArmsActionServer::get_progress(std::vector<double> initial_joints,
                                                 std::vector<double> target_joints,
                                                 std::vector<double> actual_joints)
    {
        return get_distance(initial_joints, target_joints) < 0.001 ? 100.0 : 100.0 * abs(1.0 - get_distance(actual_joints, target_joints) / get_distance(initial_joints, target_joints));
    }

    double LocobotArmsActionServer::get_progress(geometry_msgs::msg::PoseStamped initial_pose,
                                                 geometry_msgs::msg::Pose target_pose,
                                                 geometry_msgs::msg::PoseStamped actual_pose)
    {
        std::vector<double> initial_pose_d(3);
        initial_pose_d.push_back(initial_pose.pose.position.x);
        initial_pose_d.push_back(initial_pose.pose.position.y);
        initial_pose_d.push_back(initial_pose.pose.position.z);
        std::vector<double> target_pose_d(3);
        target_pose_d.push_back(target_pose.position.x);
        target_pose_d.push_back(target_pose.position.y);
        target_pose_d.push_back(target_pose.position.z);
        std::vector<double> actual_pose_d(3);
        actual_pose_d.push_back(actual_pose.pose.position.x);
        actual_pose_d.push_back(actual_pose.pose.position.y);
        actual_pose_d.push_back(actual_pose.pose.position.z);
        tf2::Quaternion initial_quat(initial_pose.pose.orientation.x,
                                     initial_pose.pose.orientation.y,
                                     initial_pose.pose.orientation.z,
                                     initial_pose.pose.orientation.w);
        tf2::Quaternion target_quat(target_pose.orientation.x,
                                    target_pose.orientation.y,
                                    target_pose.orientation.z,
                                    target_pose.orientation.w);
        tf2::Quaternion actual_quat(actual_pose.pose.orientation.x,
                                    actual_pose.pose.orientation.y,
                                    actual_pose.pose.orientation.z,
                                    actual_pose.pose.orientation.w);

        double position_dis = get_distance(initial_pose_d, target_pose_d) < 0.001 ? 100.0 : 100.0 * (1.0 - get_distance(actual_pose_d, target_pose_d) / get_distance(initial_pose_d, target_pose_d));
        double orientation_dis = target_quat.angleShortestPath(initial_quat) < 0.001 ? 100.0 : 100.0 * (1.0 - target_quat.angleShortestPath(actual_quat) / target_quat.angleShortestPath(initial_quat));
        return (position_dis + orientation_dis) / 2;
    }

    double LocobotArmsActionServer::get_distance(std::vector<double> initial_point, std::vector<double> goal_point)
    {
        double distance = 0;
        for (size_t i = 0; i < initial_point.size(); i++)
        {
            distance += pow(initial_point[i] - goal_point[i], 2);
        }
        return sqrt(distance);
    }

    void LocobotArmsActionServer::joint_check(const std::shared_ptr<raya_arms_msgs::srv::ArmJointPlannerCheck::Request> request,
                                              std::shared_ptr<raya_arms_msgs::srv::ArmJointPlannerCheck::Response> response)
    {

        const std::string PLANNING_GROUP = request->arm;
        moveit::planning_interface::MoveGroupInterface *move_group;
        moveit::planning_interface::MoveItErrorCode success;
        bool correct_group = true;
        bool correct_number_joints = false;

        if (request->arm.compare(arm_planning_group) == 0)
            move_group = move_group_arm;
        else if (request->arm.compare(gripper_planning_group) == 0)
            move_group = move_group_gripper;
        else
            correct_group = false;

        if (correct_group)
        {
            if (move_group->setJointValueTarget(request->position))
            {
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                correct_number_joints = true;
                success = move_group->plan(my_plan);
            }
        }
        // Check if goal is done
        if (rclcpp::ok())
        {
            convert_result(success, &response->error, correct_group, correct_number_joints);
            RCLCPP_INFO(node_->get_logger(), "Goal joint succeeded");
        }
    }

    void LocobotArmsActionServer::pose_check(const std::shared_ptr<raya_arms_msgs::srv::ArmPosePlannerCheck::Request> request,
                                             std::shared_ptr<raya_arms_msgs::srv::ArmPosePlannerCheck::Response> response)
    {

        const std::string PLANNING_GROUP = request->arm;
        moveit::planning_interface::MoveGroupInterface *move_group;
        moveit::planning_interface::MoveItErrorCode success;
        bool correct_group = true;

        if (request->arm.compare(arm_planning_group) == 0)
            move_group = move_group_arm;
        else if (request->arm.compare(gripper_planning_group) == 0)
            move_group = move_group_gripper;
        else
            correct_group = false;

        if (correct_group)
        {
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;

            move_group->setPoseTarget(request->goal_pose);
            success = move_group->plan(my_plan);
        }
        if (rclcpp::ok())
        {
            convert_result(success, &response->error,
                           correct_group);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    rclcpp::executors::MultiThreadedExecutor executor;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto locobot_arms = std::make_shared<locobot_arms::LocobotArmsActionServer>(node_options);
    executor.add_node(locobot_arms->node_);
    std::thread([&executor]()
                {executor.spin(); rclcpp::shutdown(); })
        .detach();
    locobot_arms->init_moveit_groups();
    while (rclcpp::ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
