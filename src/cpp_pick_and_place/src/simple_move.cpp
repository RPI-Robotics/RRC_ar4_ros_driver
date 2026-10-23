#include <vector>
#include <memory>
#include <stdexcept>
#include <string>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "geometry_msgs/msg/pose.hpp"

class arm_controller {

    public: 
    
    arm_controller(std::shared_ptr<rclcpp::Node> node, 
        double a_pt, unsigned int a_npa, double a_mvsf, double a_masf,
        double g_pt, unsigned int g_npa, double g_mvsf, double g_masf) : _node(std::move(node)),arm(_node,"ar_manipulator"), gripper(_node, "ar_gripper") {
        //pose is: {x,y,z,qx,qy,qz,qw}
        //do note that its like x,y on the ground plane and z is the height
        //also the 0.707 in x and w in the quaternion was just read from the current pose when at SAFE and put here
        _node->declare_parameter("pick_pose",  std::vector<double>{0.0, -0.5, 0.0, 0.707, 0.0, 0.0, 0.707});
        _node->declare_parameter("place_pose", std::vector<double>{-0.2, -0.5, 0.0, 0.707, 0.0, 0.0, 0.707});

        std::vector<double> pick_val = _node->get_parameter("pick_pose").as_double_array();
        std::vector<double> place_val = _node->get_parameter("place_pose").as_double_array();

        if(pick_val.size() != 7 || place_val.size() != 7){
            throw std::runtime_error("pick and place values must be 7 long");
        }

        //might be a better way to upack these values, idk
        pick.position.x = pick_val[0];
        pick.position.y = pick_val[1];
        pick.position.z = pick_val[2];

        pick.orientation.x = pick_val[3];
        pick.orientation.y = pick_val[4];
        pick.orientation.z = pick_val[5];
        pick.orientation.w = pick_val[6];
        
        place.position.x = place_val[0];
        place.position.y = place_val[1];
        place.position.z = place_val[2];

        place.orientation.x = place_val[3];
        place.orientation.y = place_val[4];
        place.orientation.z = place_val[5];
        place.orientation.w = place_val[6];

        arm.setPlanningTime(a_pt);
        arm.setNumPlanningAttempts(a_npa);
        arm.setMaxVelocityScalingFactor(a_mvsf);
        arm.setMaxAccelerationScalingFactor(a_masf);

        gripper.setPlanningTime(g_pt);
        gripper.setNumPlanningAttempts(g_npa);
        gripper.setMaxVelocityScalingFactor(g_mvsf);
        gripper.setMaxAccelerationScalingFactor(g_masf);
    }

    //I think this is a good way to do over just guessing angles? there might be a better method
    void set_pose_target(const geometry_msgs::msg::Pose& pose, const std::string& label){
        if (!arm.setPoseTarget(pose)) {
            throw std::runtime_error("Could not set pose target: " + label);
        }
        
        //debugging
        RCLCPP_INFO(_node->get_logger(),"target: x=%.3f y=%.3f z=%.3f",pose.position.x,pose.position.y,pose.position.z);
        RCLCPP_INFO(_node->get_logger(),"orientation: x=%.3f y=%.3f z=%.3f w=%.3f",pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
        RCLCPP_INFO(_node->get_logger(),"reference frame: %s",arm.getPoseReferenceFrame().c_str());

        plan_and_execute(arm, label);
        arm.clearPoseTargets();
    }

    //this and set_joint_target are basically taken straight from simple_pick_place_mgi.cpp in the example files
    void plan_and_execute(moveit::planning_interface::MoveGroupInterface& mgi, const std::string& label){        
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        moveit::core::MoveItErrorCode plan_code = mgi.plan(plan);
        if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(_node->get_logger(), "planning failed at %s; MoveIt error: %d", label.c_str(),plan_code.val); //more verbose
            throw std::runtime_error("Planning failed: " + label);
        }

        moveit::core::MoveItErrorCode exec_code = mgi.execute(plan);
        if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Execution failed: " + label);
        }

        RCLCPP_INFO(_node->get_logger(), "Done: %s", label.c_str());
    }

    void set_joint_target(const std::vector<double>& joint_values, const std::string& label){
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
        set_pose_target(pick, "PICK");

        std::this_thread::sleep_for(std::chrono::seconds(1)); //arbitary delays just so that it doesn't close or open gripper and immediately move

        // Close gripper (MoveIt named state)
        RCLCPP_INFO(_node->get_logger(), "Gripper -> closed");
        gripper.setNamedTarget("closed");
        plan_and_execute(gripper, "gripper closed");

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Return to SAFE
        RCLCPP_INFO(_node->get_logger(), "Arm -> SAFE (return)");
        set_joint_target(safe, "SAFE(return)");
        plan_and_execute(arm, "arm SAFE (return)");

        // Go to PLACE
        RCLCPP_INFO(_node->get_logger(), "Arm -> PLACE ");
        set_pose_target(place, "PLACE");

        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Re-open gripper
        RCLCPP_INFO(_node->get_logger(), "Gripper -> open (end)");
        gripper.setNamedTarget("open");
        plan_and_execute(gripper, "gripper open (end)");

        std::this_thread::sleep_for(std::chrono::seconds(1));
        
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
    const std::vector<double> safe = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //this won't really ever change so it doesn't need a parameter
    geometry_msgs::msg::Pose pick;
    geometry_msgs::msg::Pose place;
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