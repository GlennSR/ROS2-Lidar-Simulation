cmake_minimum_required(VERSION 3.8)
project(my_simulation)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)  
include_directories(${PCL_INCLUDE_DIRS})


install(DIRECTORY
  launch
  world
  models
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(pointcloud_subscriber src/pointcloud_subscriber.cpp)
ament_target_dependencies(pointcloud_subscriber rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pointcloud_subscriber ${PCL_LIBRARIES})

add_executable(robot_control src/robot_control.cpp)
ament_target_dependencies(robot_control rclcpp geometry_msgs)
target_include_directories(robot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS robot_control pointcloud_subscriber
  DESTINATION lib/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()