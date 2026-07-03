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
  ocs2_self_collision
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
  src/common/controller_utils.cpp
  src/common/constraint/dynamics_self_collision_constraint.cpp
  src/common/constraint/ee_contact_friction_cone_soft_constraint.cpp
  src/common/cost/ee_motion_tracking_cost.cpp
  src/common/cost/input_tracking_cost.cpp
  src/common/cost/joint_tracking_cost.cpp
  src/common/pinocchio_utils.cpp
  src/diagnostics/mpc_policy_publisher.cpp
  src/estimation/momentum_observer_wrench_estimator.cpp
  src/visualization/optimized_state_trajectory_visualization.cpp
  src/visualization/performance_visualization.cpp
  src/visualization/self_collision_visualization.cpp

  src/inverse_dynamics_mpc/constraint/inverse_dynamics_ee_wrench_tracking_constraint.cpp
  src/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_constraint_cppad.cpp
  src/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_with_ee_wrench_constraint_cppad.cpp
  src/inverse_dynamics_mpc/dynamics/inverse_dynamics_kinematic_dynamics_ad.cpp
  src/inverse_dynamics_mpc/initialization/inverse_dynamics_initializer.cpp
  src/inverse_dynamics_mpc/target/ee_motion_tracking_target.cpp
  src/inverse_dynamics_mpc/target/joint_tracking_target.cpp
  src/inverse_dynamics_mpc/inverse_dynamics_mpc_controller.cpp
  src/inverse_dynamics_mpc/inverse_dynamics_mpc_interface.cpp
  src/inverse_dynamics_mpc/inverse_dynamics_mpc_model.cpp

  src/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.cpp
  src/forward_dynamics_mpc/initialization/forward_dynamics_initializer.cpp
  src/forward_dynamics_mpc/target/ee_motion_tracking_target.cpp
  src/forward_dynamics_mpc/target/joint_tracking_target.cpp
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
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/auto_generated")
  install(DIRECTORY auto_generated/
    DESTINATION share/${PROJECT_NAME}/auto_generated
  )
endif()
install(FILES dynamics_mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
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
