cmake_minimum_required(VERSION 3.20)
project(finger_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(finger_interfaces REQUIRED)
find_package(fingerlib REQUIRED)
find_package(modern_robotics REQUIRED)

add_executable(finger_control src/finger_control.cpp)

target_link_libraries(finger_control
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${finger_interfaces_TARGETS}
  ${ARMADILLO_LIBRARIES}
  fingerlib::fingerlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS finger_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()