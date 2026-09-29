cmake_minimum_required(VERSION 3.5)
project(dae_relay_controller_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetAllRelays.srv"
  "srv/SetAllRelays.srv"
  "srv/SetRelay.srv"
)

ament_python_install_package(dae_RelayBoard
  PACKAGE_DIR "src/dae_RelayBoard"
)

install(
  PROGRAMS
    scripts/relay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
