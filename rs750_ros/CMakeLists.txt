cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(rs750_ros)
find_package(ament_cmake REQUIRED)

# Resources
install(DIRECTORY 
  launch
  scripts
  DESTINATION share/${PROJECT_NAME})

# scripts
install(PROGRAMS 
  scripts/twist_to_rudder.py 
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS 
  scripts/sail_control.py
  DESTINATION lib/${PROJECT_NAME})
  

ament_package()