cmake_minimum_required(VERSION 3.16)
project(demo_arm LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# manually adding libserial-dev
# include_directories(/usr/include/libserial)
# link_directories(/usr/lib/aarch64-linux-gnu)

# Create the shared library for hardware abstraction
add_library(
  demo_arm 
  SHARED
  hardware/servo_system_hardware.cpp
)

# Set include directories
target_compile_features(demo_arm PUBLIC cxx_std_17)
target_include_directories(demo_arm PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
    $<INSTALL_INTERFACE:include/demo_arm>
)

# Link dependencies
ament_target_dependencies(
  demo_arm PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# add the libserial library
# target_link_libraries(demo_arm /usr/lib/aarch64-linux-gnu/libserial.so)
target_link_libraries(demo_arm PUBLIC serial)

target_compile_definitions(${PROJECT_NAME} PRIVATE "DEMO_ARM_BUILDING_DLL")

# Export hardware interface as a plugin
pluginlib_export_plugin_description_file(hardware_interface demo_arm.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/demo_arm
)
install(
  DIRECTORY config description launch meshes
  DESTINATION share/demo_arm
)
install(TARGETS demo_arm
  EXPORT export_demo_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# Export targets and dependencies
ament_export_targets(export_demo_arm HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()