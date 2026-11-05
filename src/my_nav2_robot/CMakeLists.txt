cmake_minimum_required(VERSION 3.10)
project(my_nav2_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  worlds
  maps
  config
  rviz
  behavior_trees
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
