#include "demo_arm/servo_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <iostream>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace demo_arm {

hardware_interface::CallbackReturn ServoSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info) {

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.rot_base_name = info_.hardware_parameters["rot_base_name"];
  cfg_.arm1_name = info_.hardware_parameters["arm1_name"]; 
  cfg_.arm2_name = info_.hardware_parameters["arm2_name"];
  cfg_.arm3_name = info_.hardware_parameters["arm3_name"];
  cfg_.claw_name = info_.hardware_parameters["claw_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]); 
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timout_ms"]);
  // cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  rot_base_.setup(cfg_.rot_base_name);
  arm1_.setup(cfg_.arm1_name);
  arm2_.setup(cfg_.arm2_name);
  arm3_.setup(cfg_.arm3_name);
  claw_.setup(cfg_.claw_name);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ServoSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rot_base_.name, hardware_interface::HW_IF_POSITION, &rot_base_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rot_base_.name, hardware_interface::HW_IF_VELOCITY, &rot_base_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    arm1_.name, hardware_interface::HW_IF_POSITION, &arm1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    arm1_.name, hardware_interface::HW_IF_VELOCITY, &arm1_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    arm2_.name, hardware_interface::HW_IF_POSITION, &arm2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    arm2_.name, hardware_interface::HW_IF_VELOCITY, &arm2_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    arm3_.name, hardware_interface::HW_IF_POSITION, &arm3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    arm3_.name, hardware_interface::HW_IF_VELOCITY, &arm3_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    claw_.name, hardware_interface::HW_IF_POSITION, &claw_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    claw_.name, hardware_interface::HW_IF_VELOCITY, &claw_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ServoSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    rot_base_.name, hardware_interface::HW_IF_POSITION, &rot_base_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    arm1_.name, hardware_interface::HW_IF_POSITION, &arm1_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    arm2_.name, hardware_interface::HW_IF_POSITION, &arm2_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    arm3_.name, hardware_interface::HW_IF_POSITION, &arm3_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    claw_.name, hardware_interface::HW_IF_POSITION, &claw_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn ServoSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoSystemHardware"), "Activating ...please wait...");

  // connect to arduino with device, baud_rate and timeout_ms parameters
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("ServoSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoSystemHardware"), "Deactivating ...please wait...");

  comms_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("ServoSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ServoSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // get encorder values 
  comms_.read_encoder_values(rot_base_.enc, arm1_.enc, arm2_.enc, arm3_.enc, claw_.enc);

  double delta_sec = period.seconds();

  double pos_prev = rot_base_.pos;
  rot_base_.pos = rot_base_.calc_enc_angle();
  rot_base_.vel = (rot_base_.pos - pos_prev) / delta_sec;

  pos_prev = arm1_.pos;
  arm1_.pos = arm1_.calc_enc_angle();
  arm1_.vel = (arm1_.pos - pos_prev) / delta_sec;

  pos_prev = arm2_.pos;
  arm2_.pos = arm2_.calc_enc_angle();
  arm2_.vel = (arm2_.pos - pos_prev) / delta_sec;

  pos_prev = arm3_.pos;
  arm3_.pos = arm3_.calc_enc_angle();
  arm3_.vel = (arm3_.pos - pos_prev) / delta_sec;

  pos_prev = claw_.pos;
  claw_.pos = claw_.calc_enc_angle();
  claw_.vel = (claw_.pos - pos_prev) / delta_sec;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  int rot_base_cmd = rot_base_.cmd;
  int arm1_cmd = arm1_.cmd;
  int arm2_cmd = arm2_.cmd;
  int arm3_cmd = arm3_.cmd;
  int claw_cmd = claw_.cmd;

  comms_.set_motor_values(rot_base_cmd, arm1_cmd, arm2_cmd, arm3_cmd, claw_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace demo_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  demo_arm::ServoSystemHardware, hardware_interface::SystemInterface)