cmake_minimum_required(VERSION 3.8)
project(robotfun_firmware)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch firmware
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
