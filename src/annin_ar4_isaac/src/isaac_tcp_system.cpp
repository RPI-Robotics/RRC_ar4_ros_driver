#include "annin_ar4_isaac/isaac_tcp_system.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace annin_ar4_isaac {

namespace {
std::string get_param(const hardware_interface::HardwareInfo& info,
                      const std::string& name,
                      const std::string& default_value) {
  auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? default_value : it->second;
}
}  // namespace

IsaacTcpSystem::~IsaacTcpSystem() { stop_connection_thread(); }

hardware_interface::CallbackReturn IsaacTcpSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = rclcpp::get_logger("isaac_tcp_system." + info_.name);

  host_ = get_param(info_, "isaac_host", "host.docker.internal");
  port_ = get_param(info_, "isaac_port", "50555");

  const size_t num_joints = info_.joints.size();
  position_states_.assign(num_joints, 0.0);
  velocity_states_.assign(num_joints, 0.0);
  position_commands_.assign(num_joints, 0.0);
  velocity_commands_.assign(num_joints, 0.0);
  has_position_command_.assign(num_joints, false);
  has_velocity_command_.assign(num_joints, false);
  received_positions_.assign(num_joints, 0.0);
  received_velocities_.assign(num_joints, 0.0);

  for (size_t i = 0; i < num_joints; ++i) {
    const auto& joint = info_.joints[i];
    joint_names_.push_back(joint.name);
    joint_index_[joint.name] = i;
    for (const auto& interface : joint.command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        has_position_command_[i] = true;
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_command_[i] = true;
      } else {
        RCLCPP_ERROR(logger_, "Joint '%s' has unsupported command interface '%s'",
                     joint.name.c_str(), interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  for (size_t i = 0; i < num_joints; ++i) {
    const auto& params = info_.joints[i].parameters;
    auto mimic_it = params.find("mimic");
    if (mimic_it == params.end()) {
      continue;
    }
    auto mimicked = joint_index_.find(mimic_it->second);
    if (mimicked == joint_index_.end()) {
      RCLCPP_ERROR(logger_, "Joint '%s' mimics unknown joint '%s'",
                   joint_names_[i].c_str(), mimic_it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    auto multiplier_it = params.find("multiplier");
    double multiplier =
        multiplier_it == params.end() ? 1.0 : std::stod(multiplier_it->second);
    mimic_joints_.push_back({i, mimicked->second, multiplier});
  }

  std::string publish_clock = get_param(info_, "publish_clock", "false");
  if (publish_clock == "true" || publish_clock == "True") {
    clock_node_ = rclcpp::Node::make_shared("isaac_sim_clock");
    clock_publisher_ = clock_node_->create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::ClockQoS());
  }

  RCLCPP_INFO(logger_, "Isaac Sim bridge at %s:%s", host_.c_str(),
              port_.c_str());
  start_connection_thread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IsaacTcpSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_POSITION,
                                  &position_states_[i]);
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_VELOCITY,
                                  &velocity_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
IsaacTcpSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (has_position_command_[i]) {
      command_interfaces.emplace_back(joint_names_[i],
                                      hardware_interface::HW_IF_POSITION,
                                      &position_commands_[i]);
    }
    if (has_velocity_command_[i]) {
      command_interfaces.emplace_back(joint_names_[i],
                                      hardware_interface::HW_IF_VELOCITY,
                                      &velocity_commands_[i]);
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn IsaacTcpSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Start from the simulated pose so the robot does not jump on activation.
  if (wait_for_state(std::chrono::seconds(2))) {
    read(rclcpp::Time(), rclcpp::Duration(0, 0));
  } else {
    RCLCPP_WARN(logger_,
                "No joint states from Isaac Sim yet. Is the bridge script "
                "running and the simulation playing? Assuming zero pose.");
  }
  position_commands_ = position_states_;
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacTcpSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IsaacTcpSystem::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_received_) {
    position_states_ = received_positions_;
    velocity_states_ = received_velocities_;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacTcpSystem::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (socket_fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> positions(joint_names_.size(), nan);
  std::vector<double> velocities(joint_names_.size(), nan);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (has_position_command_[i]) positions[i] = position_commands_[i];
    if (has_velocity_command_[i]) velocities[i] = velocity_commands_[i];
  }
  for (const auto& mimic : mimic_joints_) {
    positions[mimic.joint] = mimic.multiplier * positions[mimic.mimicked_joint];
    velocities[mimic.joint] =
        mimic.multiplier * velocities[mimic.mimicked_joint];
  }

  std::ostringstream message;
  message << std::setprecision(9);
  size_t count = 0;
  std::ostringstream joints;
  joints << std::setprecision(9);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::isnan(positions[i]) && std::isnan(velocities[i])) continue;
    joints << ' ' << joint_names_[i] << ' ' << positions[i] << ' '
           << velocities[i];
    ++count;
  }
  if (count == 0) {
    return hardware_interface::return_type::OK;
  }
  message << "C " << count << joints.str() << '\n';

  const std::string data = message.str();
  std::lock_guard<std::mutex> lock(send_mutex_);
  int fd = socket_fd_;
  if (fd >= 0 && ::send(fd, data.data(), data.size(), MSG_NOSIGNAL) < 0) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 5000,
                         "Failed to send joint commands to Isaac Sim");
  }
  return hardware_interface::return_type::OK;
}

void IsaacTcpSystem::start_connection_thread() {
  running_ = true;
  connection_thread_ = std::thread(&IsaacTcpSystem::connection_loop, this);
}

void IsaacTcpSystem::stop_connection_thread() {
  running_ = false;
  close_socket();
  if (connection_thread_.joinable()) {
    connection_thread_.join();
  }
}

void IsaacTcpSystem::close_socket() {
  std::lock_guard<std::mutex> lock(send_mutex_);
  int fd = socket_fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
}

int IsaacTcpSystem::connect_to_isaac() {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* addresses = nullptr;
  if (::getaddrinfo(host_.c_str(), port_.c_str(), &hints, &addresses) != 0) {
    return -1;
  }
  int fd = -1;
  for (addrinfo* addr = addresses; addr != nullptr; addr = addr->ai_next) {
    fd = ::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if (fd < 0) continue;
    if (::connect(fd, addr->ai_addr, addr->ai_addrlen) == 0) break;
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(addresses);
  if (fd >= 0) {
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    // Lets the loop notice shutdown while Isaac Sim is paused and silent.
    timeval timeout{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }
  return fd;
}

void IsaacTcpSystem::connection_loop() {
  std::string buffer;
  char chunk[4096];
  while (running_) {
    if (socket_fd_ < 0) {
      int fd = connect_to_isaac();
      if (fd < 0) {
        RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 10000,
                             "Waiting for Isaac Sim bridge at %s:%s ...",
                             host_.c_str(), port_.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      if (!running_) {
        ::close(fd);
        break;
      }
      buffer.clear();
      socket_fd_ = fd;
      RCLCPP_INFO(logger_, "Connected to Isaac Sim bridge");
    }

    ssize_t received = ::recv(socket_fd_, chunk, sizeof(chunk), 0);
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }
    if (received <= 0) {
      if (running_) {
        RCLCPP_WARN(logger_, "Disconnected from Isaac Sim bridge");
      }
      close_socket();
      continue;
    }
    buffer.append(chunk, static_cast<size_t>(received));
    size_t newline;
    while ((newline = buffer.find('\n')) != std::string::npos) {
      handle_line(buffer.substr(0, newline));
      buffer.erase(0, newline + 1);
    }
  }
}

void IsaacTcpSystem::handle_line(const std::string& line) {
  std::istringstream stream(line);
  std::string type, time_str, count_str;
  if (!(stream >> type >> time_str >> count_str) || type != "S") {
    return;
  }
  try {
    const double sim_time = std::stod(time_str);
    const size_t count = std::stoul(count_str);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (size_t n = 0; n < count; ++n) {
        std::string name, position, velocity;
        if (!(stream >> name >> position >> velocity)) break;
        auto it = joint_index_.find(name);
        if (it == joint_index_.end()) continue;
        received_positions_[it->second] = std::stod(position);
        received_velocities_[it->second] = std::stod(velocity);
      }
      state_received_ = true;
    }
    if (clock_publisher_) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = rclcpp::Time(static_cast<int64_t>(sim_time * 1e9));
      clock_publisher_->publish(clock);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 5000,
                         "Bad message from Isaac Sim: %s", e.what());
  }
}

bool IsaacTcpSystem::wait_for_state(std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_received_) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

}  // namespace annin_ar4_isaac

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(annin_ar4_isaac::IsaacTcpSystem,
                       hardware_interface::SystemInterface)
