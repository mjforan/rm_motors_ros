cmake_minimum_required(VERSION 3.8)
project(gm6020_example)

install(
  DIRECTORY config/
  DESTINATION share/gm6020_example/config/
)
install(
  DIRECTORY launch/
  DESTINATION share/gm6020_example/launch/
)
