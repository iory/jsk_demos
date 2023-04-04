cmake_minimum_required(VERSION 2.8.3)
project(fg_ros)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  jsk_data
)

# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# # download and install sample data
# add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)
# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES UpdateModel.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs)


generate_dynamic_reconfigure_options(
  cfg/InstanceSegmentation.cfg
)

catkin_package(
  CATKIN_DEPENDS
  jsk_data
)


if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/fg_libs/yolo7/README.md)
  message(WARNING "node_scripts/fg_libs/yolo7 is not exists, download this")
  execute_process(COMMAND git submodule init yolo7 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/fg_libs)
  execute_process(COMMAND git submodule update yolo7 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/node_scripts/fg_libs)
endif()

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS FALSE
  CHECK_VENV FALSE
)

file(GLOB SCRIPT_PROGRAMS scripts/*.py)
catkin_install_python(
  PROGRAMS ${SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
file(GLOB SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
