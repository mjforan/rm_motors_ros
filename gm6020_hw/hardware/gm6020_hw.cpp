#include "gm6020_hw/gm6020_hw.hpp"
#include <rclcpp/rclcpp.hpp>

namespace gm6020_hw
{
hardware_interface::CallbackReturn Gm6020SystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  try{
    can_interface_ = info_.hardware_parameters.at("can_interface").c_str();
    RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "got CAN interface %s", can_interface_);
  }
  catch (const std::out_of_range& e){
    RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
      "Missing parameter: can_interface");
    return hardware_interface::CallbackReturn::ERROR;
  }
  try{
    simulate_ = info_.hardware_parameters.at("simulate")=="true";
    RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "got simulate %s", simulate_?"true":"false");
  }
  catch (const std::out_of_range& e){
    RCLCPP_WARN(rclcpp::get_logger("Gm6020SystemHardware"),"Missing parameter: simulate. Assuming true");
    simulate_ = true;
  }
  
  hw_commands_.resize(command_interface_types_.size());
  for(std::vector<double>& v : hw_commands_)
    v.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_states_.resize(state_interface_types_.size());
  for(std::vector<double>& v : hw_states_)
    v.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints){

    try{
      uint hw_id = stoi(joint.parameters.at("gm6020_id"));
      RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "got motor ID %u", hw_id);
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s missing parameter: gm6020_id", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.size() != hw_commands_.size()){
      RCLCPP_FATAL(
        rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s has %zu command interfaces. %zu expected.", joint.name.c_str(),
        joint.command_interfaces.size(), hw_commands_.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for(uint j=0; j<command_interface_types_.size(); j++){
      if (joint.command_interfaces[j].name != command_interface_types_[j]){
        RCLCPP_FATAL(
          rclcpp::get_logger("Gm6020SystemHardware"),
          "Joint %s's #%u command interface is, %s. '%s' expected.", joint.name.c_str(),
          j, joint.command_interfaces[j].name.c_str(), command_interface_types_[j]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != hw_states_.size()){
      RCLCPP_FATAL(
        rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s has %zu state interfaces. %zu expected.", joint.name.c_str(),
        joint.state_interfaces.size(), hw_states_.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for(uint j=0; j<state_interface_types_.size(); j++){
      if (joint.state_interfaces[j].name != state_interface_types_[j]){
        RCLCPP_FATAL(
          rclcpp::get_logger("Gm6020SystemHardware"),
          "Joint %s's #%u state interface is, %s. '%s' expected.", joint.name.c_str(),
          j, joint.state_interfaces[j].name.c_str(), state_interface_types_[j]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("Gm6020SystemHardware"), "Configuring ...please wait...");

  // zero all states and commands
  for(std::vector<double>& v : hw_states_)
    for(double & x : v)
      x = 0.0;
  for(std::vector<double>& v : hw_commands_)
    for(double & x : v)
      x = 0.0;

  gmc_ = gm6020_can_init(can_interface_);
  RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  //TODO release socket
  RCLCPP_INFO(rclcpp::get_logger("Gm6020Hardware"), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Gm6020SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
    for (uint j = 0; j < hw_states_.size(); j++)
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, state_interface_types_[j], &hw_states_[j][i]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Gm6020SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
    for (uint j = 0; j < hw_commands_.size(); j++)
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, command_interface_types_[j], &hw_commands_[j][i]));

  return command_interfaces;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
//  gm6020_can_run(gmc_, 1000.0/100); // TODO store thread ID somehow
  RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO stop run() thread 
  RCLCPP_INFO(rclcpp::get_logger("Gm6020Hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Gm6020SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "Reading...");

  if (!simulate_)
    gm6020_can_run_once(gmc_);
  // Iterate through all joints
  for (uint i = 0; i < hw_states_[0].size(); i++)
  {
    if (simulate_){    
      hw_states_[3][i] = 27.0;                                                             // temperature
      hw_states_[2][i] = hw_commands_[1][i];                                               // effort
      hw_states_[1][i] = hw_states_[1][i] + (hw_commands_[1][i]*0.5 - hw_states_[1][i])/2; // velocity
      hw_states_[0][i] = hw_states_[0][i] + hw_states_[1][i]*0.5;                          // position
    }
    else{
      hw_states_[0][i] = gm6020_can_get(gmc_, motor_ids_[i], FbField::Position);
      hw_states_[1][i] = gm6020_can_get(gmc_, motor_ids_[i], FbField::Velocity);
      hw_states_[2][i] = gm6020_can_get(gmc_, motor_ids_[i], FbField::Current) * N_PER_A;
      hw_states_[3][i] = gm6020_can_get(gmc_, motor_ids_[i], FbField::Temperature);
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Gm6020SystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{



  for (uint i = 0; i < hw_commands_[1].size(); i++)
  {
    if (simulate_){
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("Gm6020SystemHardware"), "Got command %.5f for joint %d!",
      hw_commands_[1][i], i);
    }
    else {
      // TODO switch to cmd_multiple to send all at once
      // TODO only allow one command mode at a time
      gm6020_can_cmd_single(gmc_, CmdMode::Voltage, motor_ids_[i], hw_commands_[0][i]);
    }
  }
  if(!simulate_) // TODO should this be handled in a separate thread for higher update rates? Does it matter if we aren't going to read at 1kHz?
    gm6020_can_run_once(gmc_);
  RCLCPP_INFO(
    rclcpp::get_logger("Gm6020SystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace gm6020_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gm6020_hw::Gm6020SystemHardware, hardware_interface::SystemInterface)
