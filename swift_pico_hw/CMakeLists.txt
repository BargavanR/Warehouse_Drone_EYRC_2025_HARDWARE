cmake_minimum_required(VERSION 3.8)
project(swift_pico_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rc_msgs REQUIRED)
find_package(pid_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(waypoint_navigation REQUIRED)   #added waypoint nav package from waypoint_navigation


#add executables

#task4a
add_executable(arm_cpp src/arm.cpp)
add_executable(pico_controller_cpp src/pico_controller.cpp)

#task4b
add_executable(pico_server_cpp src/pico_server.cpp)
add_executable(pico_client_cpp src/pico_client.cpp)
add_executable(waypoint_service_cpp src/waypoint_service.cpp)

#add dependencies

#task4a
ament_target_dependencies(arm_cpp rclcpp rc_msgs)
ament_target_dependencies(pico_controller_cpp rclcpp rc_msgs pid_msg geometry_msgs)

#task4b
ament_target_dependencies(pico_server_cpp rclcpp rc_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(pico_client_cpp rclcpp rc_msgs pid_msg geometry_msgs waypoint_navigation)
ament_target_dependencies(waypoint_service_cpp rclcpp rc_msgs pid_msg geometry_msgs waypoint_navigation)
 
# Install executables
#Target Installations for Task 4
install(
  TARGETS 
  arm_cpp 
  pico_controller_cpp
  waypoint_service_cpp
  pico_client_cpp
  pico_server_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#Install scripts
install(PROGRAMS
  src/pico_controller.py
  src/arm.py
  src/pico_client.py
  src/pico_server.py
  src/waypoint_service.py
  DESTINATION lib/${PROJECT_NAME}
)

#install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
