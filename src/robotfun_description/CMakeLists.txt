cmake_minimum_required(VERSION 3.8)
project(robotfun_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
