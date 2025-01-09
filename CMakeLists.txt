cmake_minimum_required(VERSION 3.8)
project(demo_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the shared library for hardware abstraction
add_library(${PROJECT_NAME}_hardware SHARED
  hardware/servo_system_hardware.cpp
)

# Set include directories
target_include_directories(
  ${PROJECT_NAME}_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(
  ${PROJECT_NAME}_hardware
  hardware_interface
  controller_manager
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

# Export hardware interface as a plugin
pluginlib_export_plugin_description_file(hardware_interface demo_arm.xml)

# Install the library and headers
install(
  TARGETS ${PROJECT_NAME}_hardware
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY hardware/include/demo_arm/
  DESTINATION include/demo_arm
)

# Install plugin description file
install(
  FILES demo_arm.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install other resources
install(
  DIRECTORY config description launch meshes
  DESTINATION share/${PROJECT_NAME}
)

# Export targets and dependencies
ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()