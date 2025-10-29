cmake_minimum_required(VERSION 2.8.3)
project(semi2025)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  catkin_virtualenv
  cv_bridge
  jsk_topic_tools
  nodelet
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager
  jsk_topic_tools
  nodelet
  sensor_msgs
  DEPENDS
)

# Generate virtualenv and install Python dependencies
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS FALSE
  CHECK_VENV FALSE
)

# Install Python scripts
file(GLOB PYTHON_SCRIPT_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
