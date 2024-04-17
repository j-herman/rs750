cmake_minimum_required(VERSION 3.8)
project(rs750_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Find gz-math
find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
find_package(gz-common5
  REQUIRED COMPONENTS events
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})



  #--------------------------------------
  # Find gz-msgs
  find_package(gz-msgs9 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-transport
  find_package(gz-transport12 REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-sensors
  find_package(gz-sensors7 REQUIRED)
  set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})

  #--------------------------------------
  # Find gz-sim
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

  #--------------------------------------
  # Find SDFormat
  find_package(sdformat13 REQUIRED)
  set(SDF_VER ${sdformat13_VERSION_MAJOR})



# Hydrokinetic Turbine plugin
add_library(HydrokineticTurbine SHARED
  src/HydrokineticTurbine.cc
)
target_link_libraries(HydrokineticTurbine PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
)

# Sail Position Controller plugin
add_library(SailPositionControllerGES SHARED
  src/SailPositionControllerGES.cc
)
target_link_libraries(SailPositionControllerGES PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
)
install(
  TARGETS HydrokineticTurbine SailPositionControllerGES
  DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Resources
install(DIRECTORY
  config
  #launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
