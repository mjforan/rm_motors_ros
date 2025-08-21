#ifndef RM_MOTORS_HW__RM_MOTORS_HW_HPP_
#define RM_MOTORS_HW__RM_MOTORS_HW_HPP_

#include <vector>

#include <rm_motors_can.hpp>
#include <rclcpp/macros.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rm_motors_hw
{

class RmMotorsSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RmMotorsSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  const char* can_interface_;
  bool simulate_;
  rm_motors_can::RmMotorsCan *gmc_;
  std::vector<const char*> state_interface_types_ = {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT, "temperature"};
  std::vector<rm_motors_can::MotorType> motor_types_;
  std::vector<rm_motors_can::CmdMode> command_modes_;
  std::vector<double> hw_commands_;
  std::vector<std::vector<double>> hw_states_;
  std::vector<uint> motor_ids_;
  std::vector<double> position_offsets_;
};

}  // namespace rm_motors_hw

#endif  // RM_MOTORS_HW__RM_MOTORS_HW_HPP_
