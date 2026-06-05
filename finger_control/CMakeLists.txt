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
find_package(Armadillo REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create a library.
add_library(finger_control
    include/finger_control/finger_control_base.hpp
)

# Use target_include_directories so that #include"finger_control/header.hpp" works
target_include_directories(finger_control
    PUBLIC
    ${serial_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
)

# specify additional compilation flags for the library
target_compile_options(finger_control PUBLIC -Wall -Wextra -pedantic)

# Enable c++23 support.
target_compile_features(finger_control PUBLIC cxx_std_23)

target_link_libraries(finger_control PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${finger_interfaces_TARGETS}
  ${ARMADILLO_LIBRARIES}
  fingerlib::fingerlib
)

add_executable(demo src/demo.cpp)
add_executable(whackamole src/whackamole.cpp)
add_executable(3d_waypoints src/3d_waypoints.cpp)

target_link_libraries(whackamole
  finger_control
  ${tf2_ros_TARGETS}
  ${tf2_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

target_link_libraries(3d_waypoints
  finger_control
  ${tf2_ros_TARGETS}
  ${tf2_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

target_link_libraries(demo
  finger_control
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS whackamole 3d_waypoints demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()