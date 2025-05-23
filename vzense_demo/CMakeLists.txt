cmake_minimum_required(VERSION 2.8.3)
project(vzense_demo)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  sensor_msgs
  dynamic_reconfigure
  std_msgs
)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")


# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# download and install sample data
add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py --pkg-path ${PROJECT_SOURCE_DIR})

# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES PickGrape.action
)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/DrawRects.cfg
  cfg/InstanceSegmentation.cfg
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  # CATKIN_DEPENDS message_runtime
  # sensor_msgs
  # DEPENDS
  # # INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  # LIBRARIES ${PROJECT_NAME}
)
