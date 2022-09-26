cmake_minimum_required(VERSION 3.8)
project(stereo_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

  # for launch files
install(DIRECTORY src/${PROJECT_NAME}/launch DESTINATION share/${PROJECT_NAME}/)

include_directories(include ${catkin_INCLUDE_DIRS})
  # Add C++ excuteable nodes
add_executable(stereo_split_node /src/stereo_split_node.cpp)
ament_target_dependencies(stereo_split_node rclcpp cv_bridge image_transport camera_info_manager sensor_msgs std_msgs) 

install(TARGETS
stereo_split_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()