cmake_minimum_required(VERSION 3.0.2)
project(k_arm_descriptions)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Only the gripper meshes are redistributable; the arm links of
# K_ARM_DUALARM_primitive.urdf are primitives and need no mesh files.
install(DIRECTORY meshes/HARVEST_GRIPPER_meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
