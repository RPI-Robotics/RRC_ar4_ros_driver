#include <vector>
#include <memory>
#include <stdexcept>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"


class arm_controller {

    public: 


    arm_controller(std::shared_ptr<rclcpp::Node> node, 
        double a_pt, unsigned int a_npa, double a_mvsf, double a_masf,
        double g_pt, unsigned int g_npa, double g_mvsf, double g_masf) : _node(std::move(node)),arm(_node,"ar_manipulator"), gripper(_node, "ar_gripper") {

        arm.setPlanningTime(a_pt);
        arm.setNumPlanningAttempts(a_npa);
        arm.setMaxVelocityScalingFactor(a_mvsf);
        arm.setMaxAccelerationScalingFactor(a_masf);

        gripper.setPlanningTime(g_pt);
        gripper.setNumPlanningAttempts(g_npa);
        gripper.setMaxVelocityScalingFactor(g_mvsf);
        gripper.setMaxAccelerationScalingFactor(g_masf);
    }

    //this and set_joint_target are basically taken straight from simple_pick_place_mgi.cpp in the example files
    void plan_and_execute(moveit::planning_interface::MoveGroupInterface& mgi, const std::string& label)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        moveit::core::MoveItErrorCode plan_code = mgi.plan(plan);
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Planning failed: " + label);
        }

        moveit::core::MoveItErrorCode exec_code = mgi.execute(plan);
        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Execution failed: " + label);
        }

        RCLCPP_INFO(_node->get_logger(), "Done: %s", label.c_str());
    }

    void set_joint_target(const std::vector<double>& joint_values, const std::string& label)
    {
        if (joint_names.size() != joint_values.size()) {
            throw std::runtime_error("Joint name/value size mismatch for " + label);
        }

        // Name->value map avoids ordering surprises.
        std::map<std::string, double> target;
        for (size_t i = 0; i < joint_names.size(); ++i) {
            target[joint_names[i]] = joint_values[i];
        }

        arm.setJointValueTarget(target);
    }

    void simple_pp_routine(){
        // Open gripper (MoveIt named state)
        RCLCPP_INFO(_node->get_logger(), "Gripper -> open");
        gripper.setNamedTarget("open");
        plan_and_execute(gripper, "gripper open");

        // Arm SAFE
        RCLCPP_INFO(_node->get_logger(), "Arm -> SAFE ");
        set_joint_target(safe, "SAFE");
        plan_and_execute(arm, "arm SAFE");

        // Arm PICK
        RCLCPP_INFO(_node->get_logger(), "Arm -> PICK ");
        set_joint_target(pick, "PICK");
        plan_and_execute(arm, "arm PICK");

        // Close gripper (MoveIt named state)
        RCLCPP_INFO(_node->get_logger(), "Gripper -> closed");
        gripper.setNamedTarget("closed");
        plan_and_execute(gripper, "gripper closed");

        // Return to SAFE
        RCLCPP_INFO(_node->get_logger(), "Arm -> SAFE (return)");
        set_joint_target(safe, "SAFE(return)");
        plan_and_execute(arm, "arm SAFE (return)");

        // Go to PLACE
        RCLCPP_INFO(_node->get_logger(), "Arm -> PLACE ");
        set_joint_target(place, "PLACE");
        plan_and_execute(arm, "arm PLACE");
        
        // Re-open gripper
        RCLCPP_INFO(_node->get_logger(), "Gripper -> open (end)");
        gripper.setNamedTarget("open");
        plan_and_execute(gripper, "gripper open (end)");
        
        // Return SAFE
        RCLCPP_INFO(_node->get_logger(), "Arm -> SAFE (return)");
        set_joint_target(safe, "SAFE(return)");
        plan_and_execute(arm, "arm SAFE (return)");

        RCLCPP_INFO(_node->get_logger(), "Demo complete.");
    }

    private:
    std::shared_ptr<rclcpp::Node> _node;

    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface gripper;

    const std::vector<std::string> joint_names = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
    const std::vector<double> safe = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<double> pick = {0.0, 1.0647, 0.1047, 0.0, 0.4014, 0.0};
    const std::vector<double> place = {0.5, 1.0647, 0.1047, 0.0, 0.4014, 0.0}; //not used as of now 
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv); 

    try {
        std::shared_ptr node = std::make_shared<rclcpp::Node>("ar4_simple_pick_and_place");
        arm_controller ac(node,5.0,10,0.3,0.3,2.0,5,1.0,1.0); //supposedly good demo values, from the simple_pick_place_mgi.cpp in the example files
        ac.simple_pp_routine();
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("arm_controller"),
            "ERROR: %s", e.what()
        );
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();

    return 0;
}