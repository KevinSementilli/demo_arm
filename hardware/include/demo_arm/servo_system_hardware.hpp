#ifndef DEMO_ARM_SERVO_SYSTEM_HARDWARE_HPP_
#define DEMO_ARM_SERVO_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "demo_arm/arduino_comms.hpp"
#include "demo_arm/Servo.hpp"

namespace demo_arm
{
  class ServoSystemHardware : public hardware_interface::SystemInterface {
  
  struct Config {
    
    std::string rot_base_name = "";
    std::string arm1_name = "";
    std::string arm2_name = "";
    std::string arm3_name = "";
    std::string claw_name = "";

    float loop_rate = 0;
    std::string device = "";
    int baud_rate = 0 ;
    int timeout_ms = 0;
    // int enc_counts_per_rev = 0;

  };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ServoSystemHardware);

    hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
  
    ArduinoComms comms_;
    Config cfg_;

    Servo rot_base_;
    Servo arm1_;
    Servo arm2_;
    Servo arm3_;
    Servo claw_;
    
  };

} // namespace demo_arm

#endif // DEMO_ARM_SERVO_SYSTEM_HARDWARE_HPP