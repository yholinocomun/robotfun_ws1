cmake_minimum_required(VERSION 3.8)
project(robotfun_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
