cmake_minimum_required(VERSION 3.5)
project(autonomous-racing)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

## Find packages, remove those that are not needed
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(pipes 3.0.5 REQUIRED) #for using the audi library
#################################################################

## Add each node
add_executable(platform src/nodeMains/main_platform.cpp src/controller.cpp src/ackerman.cpp)
add_executable(mission src/nodeMains/main_mission.cpp src/mission.cpp)
add_executable(coneFinder src/nodeMains/main_coneFinder.cpp src/coneFinder.cpp)
add_executable(display src/nodeMains/main_display.cpp src/display.cpp)

## Ros Dependancies
ament_target_dependencies(platform rclcpp std_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)
ament_target_dependencies(mission rclcpp nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)
ament_target_dependencies(coneFinder rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(display rclcpp geometry_msgs visualization_msgs)


## Add each library

add_library(laserProcessing src/laserProcessing.cpp)
add_library(pathFinder src/pathFinder.cpp)

## Ros dependancies for each library (The use the ros msg structures to store and analyze data)
ament_target_dependencies(laserProcessing sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(pathFinder nav_msgs tf2 tf2_ros tf2_geometry_msgs)


## Link libraries to each node
target_link_libraries(mission pathFinder)
target_link_libraries(coneFinder laserProcessing)
target_link_libraries(platform audi)



install(TARGETS
  platform
  mission
  coneFinder
  display
  pathFinder
  laserProcessing
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    data
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

  #Create utests
  ament_add_ros_isolated_gtest(AdditionalGoalGeneration test/test_additionalGoalGeneration.cpp)
  ament_add_ros_isolated_gtest(IsGoalWithinCones test/test_isGoalWithinCones.cpp)
  ament_add_ros_isolated_gtest(IsObstacleInWay test/test_isObstacleInWay.cpp)
  ament_add_ros_isolated_gtest(SplineTest test/test_spline.cpp)
  

  #Link tested libraries
  target_link_libraries(AdditionalGoalGeneration pathFinder laserProcessing)
  target_link_libraries(IsGoalWithinCones pathFinder laserProcessing)
  target_link_libraries(IsObstacleInWay laserProcessing)
  target_link_libraries(SplineTest pathFinder laserProcessing)
  

  #Add ros dependancies
  ament_target_dependencies(AdditionalGoalGeneration rclcpp sensor_msgs nav_msgs rosbag2_cpp)
  ament_target_dependencies(IsGoalWithinCones rclcpp sensor_msgs nav_msgs rosbag2_cpp)
  ament_target_dependencies(IsObstacleInWay rclcpp sensor_msgs nav_msgs rosbag2_cpp)
  ament_target_dependencies(SplineTest rclcpp sensor_msgs nav_msgs rosbag2_cpp)
  

endif()

ament_package()
