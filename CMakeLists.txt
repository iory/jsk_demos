cmake_minimum_required(VERSION 3.0.2)
project(balloon_detection)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_package()

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
)

catkin_install_python(
  PROGRAMS
  scripts/detect_objects_node.py
  scripts/dog_pose_node.py
  scripts/train_dog_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RENAME_PROCESS FALSE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY pooh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.mp4"
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
