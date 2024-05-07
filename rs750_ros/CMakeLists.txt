cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(rs750_ros)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "msg/VesselPose.msg"
)

# Resources
install(DIRECTORY 
  launch
  scripts
  msg
  DESTINATION share/${PROJECT_NAME})

# scripts
install(PROGRAMS 
  scripts/twist_to_rudder.py 
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS 
  scripts/sail_control.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS 
  scripts/heading_control.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS 
  scripts/pose_generator.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()