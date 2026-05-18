cmake_minimum_required(VERSION 3.0.2)
project(balloon_detection)

# Build modes:
#   * Default (USE_UV=OFF): use catkin_virtualenv (system python3).
#   * USE_UV=ON          : use uv to manage a pinned Python interpreter and
#                          install dependencies from pyproject.toml.
#                          Useful when the system python (e.g. Ubuntu 20.04's
#                          python 3.8) is too old for the runtime deps.
option(USE_UV "Manage the Python environment with uv instead of catkin_virtualenv" OFF)

if(USE_UV)
  find_package(catkin REQUIRED)
  catkin_package()

  # ANSI color helpers so the "uv missing" error stands out in catkin logs.
  string(ASCII 27 _Esc)
  set(_ColorReset "${_Esc}[m")
  set(_BoldRed    "${_Esc}[1;31m")
  set(_BoldYellow "${_Esc}[1;33m")

  find_program(UV_EXECUTABLE uv)
  if(NOT UV_EXECUTABLE)
    message(FATAL_ERROR
      "\n"
      "${_BoldRed}"
      "============================================================\n"
      " ERROR: 'uv' was not found in PATH (USE_UV=ON was requested)\n"
      "============================================================${_ColorReset}\n"
      "${_BoldYellow}"
      " Install uv (one-liner):\n"
      "   curl -LsSf https://astral.sh/uv/install.sh | sh\n"
      "\n"
      " Then re-open the shell (or `source ~/.bashrc`) and rebuild:\n"
      "   catkin build balloon_detection --cmake-args -DUSE_UV=ON\n"
      "\n"
      " Or fall back to catkin_virtualenv:\n"
      "   catkin build balloon_detection --cmake-args -DUSE_UV=OFF"
      "${_ColorReset}\n")
  else()
    message(STATUS "${_BoldYellow}balloon_detection: using uv at ${UV_EXECUTABLE}${_ColorReset}")
  endif()

  # Run `uv sync` at build time so that catkin build sets up the venv too.
  add_custom_target(${PROJECT_NAME}_uv_sync ALL
    COMMAND ${UV_EXECUTABLE} sync
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Syncing Python deps for ${PROJECT_NAME} via uv (project: ${CMAKE_CURRENT_SOURCE_DIR})"
    VERBATIM)

  # Bake the package source dir into the wrapper so devel/install-space copies
  # still know where to find pyproject.toml.
  set(balloon_detection_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/detect_events_node_uv.in
    ${CMAKE_CURRENT_BINARY_DIR}/detect_events_node_uv
    @ONLY)

  file(COPY ${CMAKE_CURRENT_BINARY_DIR}/detect_events_node_uv
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
    FILE_PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE)

  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/detect_events_node_uv
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
  find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)
  catkin_package()

  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
  )

  catkin_install_python(
    PROGRAMS scripts/detect_events_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    RENAME_PROCESS FALSE
  )

  # img_utils.py must sit next to detect_events_node.py so the node can
  # `from img_utils import ...` without extending PYTHONPATH.
  install(FILES scripts/img_utils.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY pooh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.mp4"
)
