cmake_minimum_required(VERSION 3.0.2)
project(dae_relay_controller_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()

add_service_files(
  DIRECTORY srv
  FILES
    GetAllRelays.srv
    SetAllRelays.srv
    SetRelay.srv
)

generate_messages()

catkin_package(
	CATKIN_DEPENDS message_runtime
)

install(
  DIRECTORY 
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
  scripts/relay_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
