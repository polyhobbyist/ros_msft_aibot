cmake_minimum_required(VERSION 3.5.0)
project(ros_msft_aibot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(MSVC)
  add_compile_options(/wd4251 /wd4275)
  add_definitions(-D_USE_MATH_DEFINES)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()