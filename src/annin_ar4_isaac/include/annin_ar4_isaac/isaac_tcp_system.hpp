#pragma once

#include <atomic>
#include <hardware_interface/system_interface.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace annin_ar4_isaac {

// ros2_control system that exchanges joint commands and states with the
// ar4_isaac_bridge.py script running inside Isaac Sim over a TCP socket.
//
// Line protocol (one message per line, whitespace separated):
//   Isaac -> ROS: S <sim_time> <n> <name> <position> <velocity> ...
//   ROS -> Isaac: C <n> <name> <position> <velocity> ...   (velocity may be nan)
class IsaacTcpSystem : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacTcpSystem);

  ~IsaacTcpSystem() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void start_connection_thread();
  void stop_connection_thread();
  void connection_loop();
  int connect_to_isaac();
  void close_socket();
  void handle_line(const std::string& line);
  bool wait_for_state(std::chrono::milliseconds timeout);

  rclcpp::Logger logger_ = rclcpp::get_logger("isaac_tcp_system");
  rclcpp::Clock steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

  std::string host_;
  std::string port_;

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_index_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  std::vector<bool> has_position_command_;
  std::vector<bool> has_velocity_command_;

  // Mimic joints have no command interfaces; they are commanded as
  // multiplier * position command of the joint they mimic.
  struct MimicJoint {
    size_t joint;
    size_t mimicked_joint;
    double multiplier;
  };
  std::vector<MimicJoint> mimic_joints_;

  // Latest state received from Isaac Sim, written by the connection thread.
  std::mutex state_mutex_;
  std::vector<double> received_positions_;
  std::vector<double> received_velocities_;
  bool state_received_ = false;

  std::atomic<int> socket_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread connection_thread_;
  std::mutex send_mutex_;

  rclcpp::Node::SharedPtr clock_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
};

}  // namespace annin_ar4_isaac
