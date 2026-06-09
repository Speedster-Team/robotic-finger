cmake_minimum_required(VERSION 3.20)
project(finger_vision2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Armadillo REQUIRED)
find_package(fingerlib REQUIRED)
find_package(modern_robotics REQUIRED)
find_package(whackamole_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vision src/vision.cpp)
add_executable(simulation_vision src/simulation_vision.cpp)
add_executable(pixel_vision src/pixel_vision.cpp)

target_link_libraries(vision
  rclcpp::rclcpp
  ${cv_bridge_TARGETS}
  ${OpenCV_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${rcl_interfaces_TARGETS}
)

target_link_libraries(simulation_vision
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${ARMADILLO_LIBRARIES}
  fingerlib::fingerlib
  ${std_msgs_TARGETS}
)

target_link_libraries(pixel_vision
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_TARGETS}
  ${ARMADILLO_LIBRARIES}
  fingerlib::fingerlib
  ${whackamole_interfaces_TARGETS}
  ${std_msgs_TARGETS}
)

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

install(
  TARGETS vision simulation_vision pixel_vision
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
