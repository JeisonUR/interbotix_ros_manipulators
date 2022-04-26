#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <future>
#include <vector>

#include "raya_arms_msgs/action/arm_joint_planner.hpp"
#include "raya_arms_msgs/action/arm_pose_planner.hpp"
#include "raya_arms_msgs/action/gripper_planner.hpp"
#include "raya_arms_msgs/action/arm_name_pos_planner.hpp"
#include "raya_arms_msgs/srv/arm_joint_planner_check.hpp"
#include "raya_arms_msgs/srv/arm_pose_planner_check.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_state/robot_state.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace locobot_arms
{
    constexpr auto ARM_GROUP = "interbotix_arm";
    constexpr auto GRIPPER_GROUP = "interbotix_gripper";
    constexpr float MIN_GRIPPER = 0.011;
    constexpr float MAX_GRIPPER = 0.038;
    constexpr auto ROBOT_DESCRIPTION = "robot_description";
    class LocobotArmsActionServer
    {
    public:
        using ArmJointPlanner = raya_arms_msgs::action::ArmJointPlanner;
        using GoalHandleArmJointPlanner = rclcpp_action::ServerGoalHandle<ArmJointPlanner>;
        using ArmNamePosPlanner = raya_arms_msgs::action::ArmNamePosPlanner;
        using GoalHandleArmNamePosPlanner = rclcpp_action::ServerGoalHandle<ArmNamePosPlanner>;
        using GripperPlanner = raya_arms_msgs::action::GripperPlanner;
        using GoalHandleGripperPlanner = rclcpp_action::ServerGoalHandle<GripperPlanner>;
        using ArmPosePlanner = raya_arms_msgs::action::ArmPosePlanner;
        using GoalHandleArmPosePlanner = rclcpp_action::ServerGoalHandle<ArmPosePlanner>;

        /**
         * @brief Construct a new Gary Arms Action Server object
         *
         * @param options
         */
        LocobotArmsActionServer(const rclcpp::NodeOptions &options);

        rclcpp::Node::SharedPtr node_;
        /**
         * @brief
         *
         */
        void init_moveit_groups();

    private:
        /**
         * @brief
         *
         * @return rclcpp_action::GoalResponse
         */
        rclcpp_action::GoalResponse handle_joint_goal(const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const ArmJointPlanner::Goal> goal);

        /**
         * @brief
         *
         * @param goal_handle
         * @return rclcpp_action::CancelResponse
         */
        rclcpp_action::CancelResponse handle_joint_cancel(const std::shared_ptr<GoalHandleArmJointPlanner> goal_handle);

        /**
         * @brief
         *
         * @param goal_handle
         */
        void handle_joint_accepted(const std::shared_ptr<GoalHandleArmJointPlanner> goal_handle);

        /**
         * @brief task that execute a joint target request
         *
         * @param goal_handle
         */
        void execute_joint(const std::shared_ptr<GoalHandleArmJointPlanner> goal_handle);

        /*
         * @brief
         *
         * @return rclcpp_action::GoalResponse
         */
        rclcpp_action::GoalResponse handle_name_goal(const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const ArmNamePosPlanner::Goal> goal);

        /**
         * @brief
         *
         * @param goal_handle
         * @return rclcpp_action::CancelResponse
         */
        rclcpp_action::CancelResponse handle_name_cancel(const std::shared_ptr<GoalHandleArmNamePosPlanner> goal_handle);

        /**
         * @brief
         *
         * @param goal_handle
         */
        void handle_name_accepted(const std::shared_ptr<GoalHandleArmNamePosPlanner> goal_handle);

        /**
         * @brief task that execute a name target request
         *
         * @param goal_handle
         */
        void execute_name(const std::shared_ptr<GoalHandleArmNamePosPlanner> goal_handle);

        /**
         * @brief
         *
         * @return rclcpp_action::GoalResponse
         */
        rclcpp_action::GoalResponse handle_gripper_goal(const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const GripperPlanner::Goal> goal);

        /**
         * @brief
         *
         * @param goal_handle
         * @return rclcpp_action::CancelResponse
         */
        rclcpp_action::CancelResponse handle_gripper_cancel(const std::shared_ptr<GoalHandleGripperPlanner> goal_handle);

        /**
         * @brief
         *
         * @param goal_handle
         */
        void handle_gripper_accepted(const std::shared_ptr<GoalHandleGripperPlanner> goal_handle);

        /**
         * @brief task that execute a joint target request
         *
         * @param goal_handle
         */
        void execute_gripper(const std::shared_ptr<GoalHandleGripperPlanner> goal_handle);

        /**
         * @brief
         *
         * @return rclcpp_action::GoalResponse
         */
        rclcpp_action::GoalResponse handle_pose_goal(const rclcpp_action::GoalUUID &uuid,
                                                     std::shared_ptr<const ArmPosePlanner::Goal> goal);

        /**
         * @brief
         *
         * @param goal_handle
         * @return rclcpp_action::CancelResponse
         */
        rclcpp_action::CancelResponse handle_pose_cancel(const std::shared_ptr<GoalHandleArmPosePlanner> goal_handle);

        /**
         * @brief
         *
         * @param goal_handle
         */
        void handle_pose_accepted(const std::shared_ptr<GoalHandleArmPosePlanner> goal_handle);
        

        /**
         * @brief task that execute a pose target request
         *
         * @param goal_handle
         */
        void execute_pose(const std::shared_ptr<GoalHandleArmPosePlanner> goal_handle);

        /**
         * @brief Convert the result of the trajectory execution or planning
         *
         * @param success the valor that was got it
         * @param error a pointer to the variable where to save the error
         * @param correct_group indicates if the group was available
         */
        void convert_result(moveit::planning_interface::MoveItErrorCode success,
                            uint8_t *error,
                            bool correct_group);

        /**
         * @brief Convert the result of the trajectory execution or planning
         *
         * @param success the valor that was got it
         * @param error a pointer to the variable where to save the error
         * @param correct_group indicates if the group was available
         * @param correct_number_joints indicates if the number of joints is valid
         */
        void convert_result(moveit::planning_interface::MoveItErrorCode success,
                            uint8_t *error,
                            bool correct_group,
                            bool correct_number_joints);

        /**
         * @brief Get the progress of the execution of the joint target
         *
         * @param initial_joints the initial_value of the joints when the movement start
         * @param target_joints the target value of the joints
         * @param actual_joints the actual value of the joints
         */
        double get_progress(std::vector<double> initial_joints,std::vector<double> target_joints, std::vector<double> actual_joints);

        /**
         * @brief Get the progress of the execution of the pose target
         *
         * @param initial_pose the initial pose when the movement start
         * @param target_pose the target pose
         * @param actual_pose the actual pose 
         */
        double get_progress(geometry_msgs::msg::PoseStamped initial_pose,geometry_msgs::msg::Pose target_pose, geometry_msgs::msg::PoseStamped actual_pose);
        
        /**
         * @brief Get the euclidean distance between two points 
         * 
         * @param initial_point 
         * @param goal_point 
         * @return double 
         */
        double get_distance(std::vector<double> initial_point, std::vector<double> goal_point);

        void joint_check(const std::shared_ptr<raya_arms_msgs::srv::ArmJointPlannerCheck::Request> request,
          std::shared_ptr<raya_arms_msgs::srv::ArmJointPlannerCheck::Response>      response);
        
        void pose_check(const std::shared_ptr<raya_arms_msgs::srv::ArmPosePlannerCheck::Request> request,
          std::shared_ptr<raya_arms_msgs::srv::ArmPosePlannerCheck::Response>      response);

        void onSceneMonitorReceivedUpdate(
            planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

        rclcpp_action::Server<ArmJointPlanner>::SharedPtr joint_server_;
        rclcpp_action::Server<ArmPosePlanner>::SharedPtr pose_server_;
        rclcpp_action::Server<GripperPlanner>::SharedPtr gripper_server_;
        rclcpp_action::Server<ArmNamePosPlanner>::SharedPtr name_server_;
        rclcpp::Service<raya_arms_msgs::srv::ArmJointPlannerCheck>::SharedPtr joint_check_server_;
        rclcpp::Service<raya_arms_msgs::srv::ArmPosePlannerCheck>::SharedPtr pose_check_server_;

        /* definitions of the name of the planning groups*/
        const std::string arm_planning_group;
        const std::string gripper_planning_group;
        

        /*Definitions of the arms's interfaces*/
        moveit::planning_interface::MoveGroupInterface *move_group_arm;
        moveit::planning_interface::MoveGroupInterface *move_group_gripper;
        // moveit::planning_interface::MoveGroupInterface move_group_panda;

        /*Definitions of the joint model groups*/
        const moveit::core::JointModelGroup *arm_joint_model_group;
        const moveit::core::JointModelGroup *gripper_joint_model_group;
        

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    };
}
