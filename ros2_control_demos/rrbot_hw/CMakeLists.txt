cmake_minimum_required(VERSION 3.16)
project(rrbot_hw LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# TODO remove version number
# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  rrbot_hw
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(rrbot_hw PUBLIC cxx_std_17)
target_include_directories(rrbot_hw PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/rrbot_hw>
)
ament_target_dependencies(
  rrbot_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rrbot_hw.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/rrbot_hw
)

install(TARGETS rrbot_hw
  EXPORT export_rrbot_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_rrbot_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
