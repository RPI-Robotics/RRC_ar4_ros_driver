#include <chrono>
#include <vector>

#include "rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"


class arm_controller : public rclcpp::Node {

    public: 
    
    arm_controller() : Node("arm_controller"){
        
    }

    static void plan_and_execute(moveit::planning_interface::MoveGroupInterface& mgi,
                            rclcpp::Logger logger,
                            const std::string& label)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto plan_code = mgi.plan(plan);
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Planning failed: " + label);
        }

        auto exec_code = mgi.execute(plan);
        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Execution failed: " + label);
        }

    RCLCPP_INFO(logger, "Done: %s", label.c_str());
    }

    private:
    const std::vector<std::string> joint_names = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
    const std::vector<double> safe = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<double> pick = {0.0, 1.0647, 0.1047, 0.0, 0.4014, 0.0};
    const std::vector<double> place = {0.5, 1.0647, 0.1047, 0.0, 0.4014, 0.0};

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client;

};
