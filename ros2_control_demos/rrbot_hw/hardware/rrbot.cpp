#include "rrbot_hw/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rrbot_hw
{
hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  hw_commands_.resize(command_interface_types_.size());
  for(std::vector<double>& v : hw_commands_)
    v.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_states_.resize(state_interface_types_.size());
  for(std::vector<double>& v : hw_states_)
    v.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints){

    try{
      uint hw_id = stoi(joint.parameters.at("gm6020_id"));
      RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "got motor ID %u", hw_id);
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"),
        "Joint %s missing parameter: gm6020_id", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.size() != hw_commands_.size()){
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemEffortOnlyHardware"),
        "Joint %s has %zu command interfaces. %zu expected.", joint.name.c_str(),
        joint.command_interfaces.size(), hw_commands_.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for(uint j=0; j<command_interface_types_.size(); j++){
      if (joint.command_interfaces[j].name != command_interface_types_[j]){
        RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemEffortOnlyHardware"),
          "Joint %s's #%u command interface is, %s. '%s' expected.", joint.name.c_str(),
          j, joint.command_interfaces[j].name.c_str(), command_interface_types_[j]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != hw_states_.size()){
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemEffortOnlyHardware"),
        "Joint %s has %zu state interfaces. %zu expected.", joint.name.c_str(),
        joint.state_interfaces.size(), hw_states_.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for(uint j=0; j<state_interface_types_.size(); j++){
      if (joint.state_interfaces[j].name != state_interface_types_[j]){
        RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemEffortOnlyHardware"),
          "Joint %s's #%u state interface is, %s. '%s' expected.", joint.name.c_str(),
          j, joint.state_interfaces[j].name.c_str(), state_interface_types_[j]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Configuring ...please wait...");

  // zero all states and commands
  for(std::vector<double>& v : hw_states_)
    for(double & x : v)
      x = 0.0;
  for(std::vector<double>& v : hw_commands_)
    for(double & x : v)
      x = 0.0;
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemEffortOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
    for (uint j = 0; j < hw_states_.size(); j++)
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, state_interface_types_[j], &hw_states_[j][i]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemEffortOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
    for (uint j = 0; j < hw_commands_.size(); j++)
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, command_interface_types_[j], &hw_commands_[j][i]));

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemEffortOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Reading...");

  for (uint i = 0; i < hw_states_[0].size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[3][i] = 27.0;                                                             // temperature
    hw_states_[2][i] = hw_commands_[1][i];                                               // effort
    hw_states_[1][i] = hw_states_[1][i] + (hw_commands_[1][i]*0.5 - hw_states_[1][i])/2; // velocity
    hw_states_[0][i] = hw_states_[0][i] + hw_states_[1][i]*0.5;                          // position
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemEffortOnlyHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  for (uint i = 0; i < hw_commands_[1].size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Got command %.5f for joint %d!",
      hw_commands_[1][i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace rrbot_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rrbot_hw::RRBotSystemEffortOnlyHardware, hardware_interface::SystemInterface)
