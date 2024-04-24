cmake_minimum_required(VERSION 3.8)
project(gm6020_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## FIND DEPENDENCIES
set (INCLUDE_DEPENDS
  gm6020_ros
  joint_state_broadcaster
  pid_controller
  ros2controlcli
  controller_manager
  rviz2
  robot_state_publisher
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


# install(
#   DIRECTORY config/
#   DESTINATION share/gm6020_ros/
# )
# install(
#   DIRECTORY launch/
#   DESTINATION share/gm6020_ros/
# )



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

ament_package()
