cmake_minimum_required(VERSION 3.16)
project(my_nav2_frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(OpenMP)

include(FetchContent)
FetchContent_Declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp.git
  GIT_TAG v1.0.0
  GIT_SHALLOW TRUE
)
FetchContent_GetProperties(small_gicp)
if(NOT small_gicp_POPULATED)
  FetchContent_Populate(small_gicp)
endif()

add_executable(gicp_frontend src/gicp_frontend.cpp)
target_include_directories(gicp_frontend SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${small_gicp_SOURCE_DIR}/include
)
ament_target_dependencies(gicp_frontend
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(gicp_frontend ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(gicp_frontend OpenMP::OpenMP_CXX)
endif()

install(TARGETS gicp_frontend
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
