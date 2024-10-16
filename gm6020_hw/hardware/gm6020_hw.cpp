#include "gm6020_hw/gm6020_hw.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>    // std::sort, std::adjacent_find
#include <map>
#include <math.h> // M_PI

namespace gm6020_hw
{
hardware_interface::CallbackReturn Gm6020SystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  if (info_.joints.size() == 0){
    RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
        "No joints found");
      return hardware_interface::CallbackReturn::ERROR;
  }

  try{
    simulate_ = info_.hardware_parameters.at("simulate")=="true";
    RCLCPP_DEBUG(rclcpp::get_logger("Gm6020SystemHardware"), "got parameter simulate %s", simulate_?"true":"false");
  }
  catch (const std::out_of_range& e){
    RCLCPP_WARN(rclcpp::get_logger("Gm6020SystemHardware"),"Missing parameter: simulate. Assuming false");
    simulate_ = false;
  }

  if (simulate_)
    can_interface_ = "simulated";
  else {
    try{
      can_interface_ = info_.hardware_parameters.at("can_interface").c_str();
      RCLCPP_DEBUG(rclcpp::get_logger("Gm6020SystemHardware"), "got parameter CAN interface %s", can_interface_);
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
        "Missing parameter: can_interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_commands_.resize(info_.joints.size());

  hw_states_.resize(info_.joints.size());
  for(std::vector<double>& v : hw_states_)
    v.resize(state_interface_types_.size(), std::numeric_limits<double>::quiet_NaN());

  for (unsigned int i=0; i<info_.joints.size(); i++){
    const hardware_interface::ComponentInfo & joint = info_.joints[i];

    try{
      uint id = stoi(joint.parameters.at("gm6020_id"));
      if (id < 1 || id > 7){
        RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
          "Joint %s gm6020_id out of range [1, 7]: %u", joint.name.c_str(), id);
        return hardware_interface::CallbackReturn::ERROR;
      }
      motor_ids_.emplace_back(id);
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s missing parameter: gm6020_id", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    try{
      std::map<std::string, gm6020_can::MotorType> type_map{
        {"gm6020", gm6020_can::MotorType::GM6020},
        {"m3508",  gm6020_can::MotorType::M3508},
        {"m2006",  gm6020_can::MotorType::M2006}};
      motor_types_.emplace_back(type_map.at(joint.parameters.at("motor_type")));
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s missing or incorrect parameter: motor_type. Options are \"gm6020\", \"m3508\", \"m2006\".", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    try{
      double pos_offset = stod(joint.parameters.at("position_offset"));
      if (pos_offset < -2.0*M_PI || pos_offset > 2.0*M_PI){
        RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
          "Joint %s position_offset out of range [-2π, 2π]: %f", joint.name.c_str(), pos_offset);
        return hardware_interface::CallbackReturn::ERROR;
      }
      position_offsets_.emplace_back(pos_offset);
    }
    catch (const std::out_of_range& e){
      RCLCPP_WARN(rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s missing parameter: position_offset. Assuming 0.0", joint.name.c_str());
        position_offsets_.emplace_back(0.0);
    }

    if (joint.command_interfaces.size() != 1){
      RCLCPP_FATAL(
        rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s has %zu command interfaces. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::map<std::string, gm6020_can::CmdMode> cmd_mode_map {
      {hardware_interface::HW_IF_VELOCITY, gm6020_can::CmdMode::Velocity},
      {hardware_interface::HW_IF_EFFORT,   gm6020_can::CmdMode::Torque}
    };
    try{
      command_modes_.emplace_back(cmd_mode_map.at(joint.command_interfaces[0].name));
    }
    catch (const std::out_of_range& e){
       RCLCPP_FATAL(
        rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s has an invalid command interface: %s. Options are \"velocity\", \"effort\".", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != hw_states_[i].size()){
      RCLCPP_FATAL(
        rclcpp::get_logger("Gm6020SystemHardware"),
        "Joint %s has %zu state interfaces. %zu expected.", joint.name.c_str(),
        joint.state_interfaces.size(), hw_states_[i].size());
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

  std::vector<uint> s = motor_ids_;
  std::sort(s.begin(), s.end());
  const std::vector<uint>::iterator duplicate = std::adjacent_find(s.begin(), s.end());
  if (duplicate != s.end()){
    RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"),
        "Duplicate gm6020_id detected: %u. Each joint must have a unique ID.", *duplicate);
      return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i=0; i<motor_ids_.size(); i++){
    if (gm6020_can::init_motor(gmc_, motor_ids_[i], motor_types_[i], command_modes_[i]) < 0){
      RCLCPP_FATAL(rclcpp::get_logger("Gm6020SystemHardware"), "Unable to initialize motor: %u.", motor_ids_[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }
    else{
      RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "Initialized motor %s:%u in %s mode.",
        info_.joints[i].parameters.at("motor_type").c_str(), motor_ids_[i], info_.joints[i].command_interfaces[0].name.c_str());
    }
  }


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("Gm6020SystemHardware"), "configuring");

  // zero all states and commands
  for(std::vector<double>& v : hw_states_)
    for(double & x : v)
      x = 0.0;
  for(double& x : hw_commands_)
    x = 0.0;

  if (!simulate_){
    gmc_ = gm6020_can::init_bus(can_interface_);
    if (gmc_ == nullptr){
      RCLCPP_FATAL(
        rclcpp::get_logger("Gm6020SystemHardware"),
        "unable to configure gm6020 CAN driver on interface '%s'", can_interface_);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "configured gm6020 CAN driver on interface '%s'", can_interface_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  //TODO release socket
  RCLCPP_DEBUG(rclcpp::get_logger("Gm6020Hardware"), "cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Gm6020SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < hw_states_.size(); i++)
    for (uint j = 0; j < hw_states_[i].size(); j++)
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, state_interface_types_[j], &hw_states_[i][j]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Gm6020SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < hw_commands_.size(); i++)
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &hw_commands_[i]));

  return command_interfaces;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!simulate_){
    RCLCPP_INFO(rclcpp::get_logger("Gm6020SystemHardware"), "activated");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Gm6020SystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Gm6020Hardware"), "deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Gm6020SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger("Gm6020SystemHardware"), "Reading...");
  if (!simulate_)
    gm6020_can::run_once(gmc_);
  // Iterate through all joints
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    if (simulate_){
      hw_states_[i][3] = 27.0;                                                             // temperature
      hw_states_[i][2] = hw_commands_[i];                                                  // effort
      hw_states_[i][1] = hw_states_[i][1] + (hw_commands_[i]*0.5 - hw_states_[i][1])/2;    // velocity
      hw_states_[i][0] = hw_states_[i][0] + hw_states_[i][1]*0.5;                          // position
    }
    else{
      hw_states_[i][0] = gm6020_can::get_state(gmc_, motor_ids_[i], gm6020_can::FbField::Position) + position_offsets_[i];
      hw_states_[i][0] -= 2*M_PI*(int)(hw_states_[i][0]/(2*M_PI)); // account for position_offset shifting the output range
      hw_states_[i][1] = gm6020_can::get_state(gmc_, motor_ids_[i], gm6020_can::FbField::Velocity);
      hw_states_[i][2] = gm6020_can::get_state(gmc_, motor_ids_[i], gm6020_can::FbField::Current)*gm6020_can::nm_per_a(motor_types_[i]);
      hw_states_[i][3] = gm6020_can::get_state(gmc_, motor_ids_[i], gm6020_can::FbField::Temperature);
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("Gm6020SystemHardware"), "joints read");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Gm6020SystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("Gm6020SystemHardware"), "writing command %.5f for joint %d", hw_commands_[i], i);

    if (simulate_){
      ;
    }
    else {
      if(gm6020_can::set_cmd(gmc_, motor_ids_[i], hw_commands_[i]) < 0){
        RCLCPP_ERROR(rclcpp::get_logger("Gm6020SystemHardware"), "Error in gm6020_can::set_cmd");
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  if (!simulate_)
    gm6020_can::run_once(gmc_);

  RCLCPP_DEBUG(rclcpp::get_logger("Gm6020SystemHardware"), "joint commands written");

  return hardware_interface::return_type::OK;
}

}  // namespace gm6020_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gm6020_hw::Gm6020SystemHardware, hardware_interface::SystemInterface)
