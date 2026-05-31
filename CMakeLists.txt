cmake_minimum_required(VERSION 3.16)
project(ros2_impedance_controller VERSION 2.0.0 LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()  # Set compiler according to ROS2 standard

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
  message(STATUS "Building Release, with flags ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

set(DEPENDENCIES
  ament_index_cpp
  Eigen3
  pinocchio
  generate_parameter_library
  controller_interface
  hardware_interface
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  visualization_msgs
  kinematic_pose_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  cartesian_controller_parameters
  src/cartesian_controller_parameters.yaml
)

generate_parameter_library(
  nonlinear_controller_parameters
  src/nonlinear_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/impedance_controller_base.cpp
  src/basic_cartesian_controller.cpp
  src/nonlinear_cartesian_controller.cpp
  src/cartesian_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  cartesian_controller_parameters
  nonlinear_controller_parameters
  Eigen3::Eigen
  pinocchio::pinocchio
  ament_index_cpp::ament_index_cpp
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  realtime_tools::realtime_tools
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${kinematic_pose_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# MPCIController (qpOASES dependent)
find_package(qpOASES QUIET)
if(qpOASES_FOUND)
  generate_parameter_library(
    mpcic_parameters
    src/mpcic_parameters.yaml
  )

  add_library(mpcic_controller SHARED
    src/taskspace_predictor.cpp
    src/mpcic_controller.cpp
  )

  target_compile_features(mpcic_controller PUBLIC cxx_std_17)

  target_include_directories(mpcic_controller PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  target_link_libraries(mpcic_controller PUBLIC
    mpcic_parameters
    Eigen3::Eigen
    pinocchio::pinocchio
    ament_index_cpp::ament_index_cpp
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    realtime_tools::realtime_tools
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${kinematic_pose_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${PROJECT_NAME}
    qpOASES
  )

  install(
    TARGETS
      ros2_impedance_controller
      cartesian_controller_parameters
      nonlinear_controller_parameters
      mpcic_controller
      mpcic_parameters
    EXPORT export_ros2_impedance_controller
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
  )
else()
  # Install only the standard controllers
  install(
    TARGETS
      ros2_impedance_controller
      cartesian_controller_parameters
      nonlinear_controller_parameters
    EXPORT export_ros2_impedance_controller
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
  )
endif()

pluginlib_export_plugin_description_file(controller_interface ros2_impedance_controller.xml)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)

  install(
    FILES
      "test/urdf/test_load.urdf.xacro"
    DESTINATION share/${PROJECT_NAME}/test
  )

  # test loading controller
  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_impedance_controller test/test_load_impedance_controller.cpp)
  target_link_libraries(test_load_impedance_controller
    controller_manager::controller_manager
    hardware_interface::hardware_interface
  )
endif()

ament_export_targets(export_ros2_impedance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES} qpOASES)
ament_package()
