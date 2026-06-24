cmake_minimum_required(VERSION 3.14)
project(dynamics_mpc_controller LANGUAGES CXX)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_msgs
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_ros_interfaces
  ocs2_sqp
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  urdf
  visualization_msgs
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(inverse_dynamics_mpc_controller_parameters
  src/inverse_dynamics_mpc/inverse_dynamics_mpc_controller_parameter.yaml
)
generate_parameter_library(forward_dynamics_mpc_controller_parameters
  src/forward_dynamics_mpc/forward_dynamics_mpc_controller_parameter.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/pinocchio_utils.cpp
  src/inverse_dynamics_mpc/constraint/inverse_dynamics_constraint_cppad.cpp
  src/inverse_dynamics_mpc/constraint/inverse_dynamics_with_ee_wrench_constraint_cppad.cpp
  src/inverse_dynamics_mpc/cost/inverse_dynamics_state_input_cost.cpp
  src/inverse_dynamics_mpc/diagnostics/mpc_policy_publisher.cpp
  src/inverse_dynamics_mpc/dynamics/inverse_dynamics_kinematic_dynamics_ad.cpp
  src/estimation/momentum_observer_wrench_estimator.cpp
  src/inverse_dynamics_mpc/initialization/inverse_dynamics_initializer.cpp
  src/inverse_dynamics_mpc/reference/joint_tracking_target.cpp
  src/inverse_dynamics_mpc/visualization/optimized_state_trajectory_visualization.cpp
  src/inverse_dynamics_mpc/inverse_dynamics_mpc_controller.cpp
  src/inverse_dynamics_mpc/inverse_dynamics_mpc_interface.cpp
  src/inverse_dynamics_mpc/inverse_dynamics_mpc_model.cpp
  src/forward_dynamics_mpc/cost/forward_dynamics_state_input_cost.cpp
  src/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.cpp
  src/forward_dynamics_mpc/initialization/forward_dynamics_initializer.cpp
  src/forward_dynamics_mpc/reference/joint_tracking_target.cpp
  src/forward_dynamics_mpc/forward_dynamics_mpc_controller.cpp
  src/forward_dynamics_mpc/forward_dynamics_mpc_interface.cpp
  src/forward_dynamics_mpc/forward_dynamics_mpc_model.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    inverse_dynamics_mpc_controller_parameters
    forward_dynamics_mpc_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(
  controller_interface
  dynamics_mpc_controller_plugin.xml
)

add_executable(optimized_state_trajectory_visualization_node
  src/inverse_dynamics_mpc/visualization/optimized_state_trajectory_visualization_node.cpp
)
target_compile_features(optimized_state_trajectory_visualization_node PUBLIC cxx_std_17)
target_include_directories(optimized_state_trajectory_visualization_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(optimized_state_trajectory_visualization_node
  ${PROJECT_NAME}
)
ament_target_dependencies(optimized_state_trajectory_visualization_node
  rclcpp
  geometry_msgs
  ocs2_msgs
  ocs2_pinocchio_interface
  pinocchio
  std_msgs
  visualization_msgs
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)
install(DIRECTORY auto_generated/
  DESTINATION share/${PROJECT_NAME}/auto_generated
)
install(FILES dynamics_mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS launch/joint_tracking_target.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  inverse_dynamics_mpc_controller_parameters
  forward_dynamics_mpc_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS optimized_state_trajectory_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
