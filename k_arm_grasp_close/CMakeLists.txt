cmake_minimum_required(VERSION 3.0.2)
project(k_arm_grasp_close)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
    node_scripts/grasp_close_node.py
    scripts/make_dataset.py
    scripts/train.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config data launch models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
