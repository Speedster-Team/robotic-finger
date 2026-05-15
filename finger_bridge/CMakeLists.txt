cmake_minimum_required(VERSION 3.20)
project(finger_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(finger_interfaces REQUIRED)
find_package(Armadillo REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Create a library.
add_library(serialinterface
    include/serial_interface/serial_interface.hpp
    src/serial_interface.cpp
)

# Use target_include_directories so that #include"fingerlib/header.hpp" works
target_include_directories(serialinterface
    PUBLIC
    ${serial_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
)

# specify additional compilation flags for the library
target_compile_options(serialinterface PUBLIC -Wall -Wextra -pedantic)

# Enable c++23 support.
target_compile_features(serialinterface PUBLIC cxx_std_23)

target_link_libraries(serialinterface PRIVATE
    ${serial_LIBRARIES}
)

add_executable(hardware_bridge src/hardware_bridge.cpp)

target_link_libraries(hardware_bridge
  rclcpp::rclcpp
  ${finger_interfaces_TARGETS}
  serialinterface
  ${std_msgs_TARGETS}
)

add_executable(simulation_bridge src/simulation_bridge.cpp)

target_link_libraries(simulation_bridge
  rclcpp::rclcpp
  ${finger_interfaces_TARGETS}
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

install(TARGETS hardware_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS simulation_bridge
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
