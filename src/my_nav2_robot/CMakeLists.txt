cmake_minimum_required(VERSION 3.10)
project(my_nav2_robot)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/ground_truth_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  maps
  config
  rviz
  behavior_trees
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
