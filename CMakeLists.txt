cmake_minimum_required(VERSION 3.16)
project(ros2_impedance_controller LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()

set(DEPENDENCIES
  ament_index_cpp
  controller_interface
  Eigen3
  geometry_msgs
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(DART REQUIRED COMPONENTS utils-urdf)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  ros2_impedance_controller_parameters
  src/ros2_impedance_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED src/ros2_impedance_controller.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ros2_impedance_controller_parameters
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES} DART)
pluginlib_export_plugin_description_file(controller_interface ros2_impedance_controller.xml)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ros2_impedance_controller
    ros2_impedance_controller_parameters
  EXPORT export_ros2_impedance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_ros2_impedance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
