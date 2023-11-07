cmake_minimum_required(VERSION 2.8.3)
project(r8eus)

find_package(catkin REQUIRED
  catkin_virtualenv
  seed_ros_controller
)

if(EXISTS ${seed_ros_controller_SOURCE_PREFIX}/../robots/r8_5/model/r8_5.urdf.xacro)
  set(r8_xacro ${seed_ros_controller_SOURCE_PREFIX}/../robots/r8_5/model/r8_5.urdf.xacro)
  set(r8_urdf ${seed_ros_controller_SOURCE_PREFIX}/../robots/r8_5/model/r8_5.urdf)
else()
  message(WARNING "Could not found r8_5.urdf.xacro in ${r8_5_SOURCE_PREFIX}/model/r8_5.urdf.xacro")
endif()

catkin_package(
  DEPENDS
  LIBRARIES ${PROJECT_NAME}
)

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES FALSE
  ISOLATE_REQUIREMENTS FALSE
  CHECK_VENV FALSE
)

set(_urdf2eus "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/venv/bin/urdf2eus")
set(_venv_setup "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/venv/bin/activate")
set(_yaml_path ${PROJECT_SOURCE_DIR}/r8_5.yaml)

add_custom_command(OUTPUT ${r8_urdf}
  COMMAND xacro ${r8_xacro} > ${r8_urdf}
  DEPENDS ${r8_xacro})
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/r8_5.l
  COMMAND echo "${_urdf2eus} ${r8_urdf} ${PROJECT_SOURCE_DIR}/r8_5.l"
  COMMAND . ${_venv_setup} && ${_urdf2eus} ${r8_urdf} ${PROJECT_SOURCE_DIR}/r8_5.l --yaml-path ${_yaml_path}
  DEPENDS ${r8_urdf} ${_urdf2eus} ${_venv_setup} ${_yaml_path})

add_custom_target(compile_r8 ALL DEPENDS ${PROJECT_SOURCE_DIR}/r8_5.l ${PROJECT_NAME}_generate_virtualenv)
