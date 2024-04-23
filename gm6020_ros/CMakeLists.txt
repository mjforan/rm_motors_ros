cmake_minimum_required(VERSION 3.8)
project(gm6020_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## FIND DEPENDENCIES
set (INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  gm6020_ros
  SHARED
  src/gm6020_ros.cpp
)

## COMPILE
target_compile_features(gm6020_ros PUBLIC cxx_std_17)
target_include_directories(gm6020_ros PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/gm6020_ros>
)
ament_target_dependencies(
  gm6020_ros PUBLIC
  ${INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "GM6020_ROS_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface gm6020_ros.xml)

# install(
#   DIRECTORY include/gm6020_ros/
#   DESTINATION include/gm6020_ros/
# )

install(TARGETS gm6020_ros
  EXPORT export_gm6020_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## TESTS
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


## EXPORTS
ament_export_targets(export_gm6020_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
