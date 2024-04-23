cmake_minimum_required(VERSION 3.14)
project(cem_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/cem_controller/cem_controller.hpp
  include/cem_controller/visibility_control.hpp
  src/cem_controller.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/cem_controller/cem_controller_node.hpp
  src/cem_controller_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "cem_controller::CemControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_cem_controller
    test/test_cem_controller.cpp
  )
  ament_target_dependencies(test_cem_controller ${TEST_SOURCES})
  target_link_libraries(test_cem_controller ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
