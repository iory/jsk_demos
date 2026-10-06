cmake_minimum_required(VERSION 3.0.2)
project(k_arm_teleop)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  catkin_virtualenv
  control_msgs
  message_generation
  pluginlib
  roscpp
  rviz
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

catkin_python_setup()

add_message_files(FILES LeaderServoStatus.msg ServoState.msg TeleopStatus.msg)
add_service_files(FILES MoveLeader.srv SetArmEnabled.srv SetServoDirection.srv SetString.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# feetech-cli and friends live in a uv-managed virtualenv (catkin_virtualenv).
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES TRUE
)

# -- rviz panel
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_definitions(-DQT_NO_KEYWORDS)
include_directories(${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}_rviz_plugin src/rviz_plugin/teleop_panel.cpp src/rviz_plugin/teleop_panel.h)
target_link_libraries(${PROJECT_NAME}_rviz_plugin ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(${PROJECT_NAME}_rviz_plugin ${PROJECT_NAME}_generate_messages_cpp)

# -- python nodes (wrapped by catkin_virtualenv)
catkin_install_python(
  PROGRAMS
    node_scripts/fake_trajectory_controller.py
    node_scripts/feetech_leader_driver.py
    node_scripts/leader_display_joint_states.py
    node_scripts/recenter_leader.py
    node_scripts/teleop_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/prefix_urdf.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_rviz_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES plugin_description.xml requirements.in requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config launch rviz scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch)
endif()
