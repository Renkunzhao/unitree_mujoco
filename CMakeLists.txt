cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

set(interfaces_target ${PROJECT_NAME}_interfaces)
rosidl_generate_interfaces(${interfaces_target}
  "msg/EpisodeStatus.msg"
  "srv/ResetEpisode.srv"
  "srv/StartEpisode.srv"
  DEPENDENCIES builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/simulate/mujoco" CACHE PATH
    "Path to an extracted MuJoCo distribution")
set(MUJOCO_LIBRARY "${MUJOCO_DIR}/lib/libmujoco.so")
if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h" OR
   NOT EXISTS "${MUJOCO_LIBRARY}")
  message(FATAL_ERROR
    "MuJoCo was not found at ${MUJOCO_DIR}. Set MUJOCO_DIR or create "
    "simulate/mujoco pointing to an extracted MuJoCo distribution.")
endif()
file(STRINGS "${MUJOCO_DIR}/include/mujoco/mujoco.h" MUJOCO_VERSION_HEADER
  REGEX "^#define mjVERSION_HEADER ")
if(NOT MUJOCO_VERSION_HEADER STREQUAL "#define mjVERSION_HEADER 4000000")
  message(FATAL_ERROR "This branch requires MuJoCo 4.0.0; check MUJOCO_DIR.")
endif()

add_executable(unitree_mujoco
  simulate/src/main.cc
  simulate/src/beam_episode_monitor.cc
  simulate/src/depth_camera.cc
  simulate/src/depth_noise.cc
  simulate/src/episode_manager.cc
  simulate/src/episode_monitor.cc
  simulate/src/inertial_gt.cc
  simulate/src/payload.cc
  simulate/src/ros2_bridge.cc
  simulate/src/joystick/joystick.cc
  simulate/src/lodepng/lodepng.cpp
  ${MUJOCO_DIR}/simulate/glfw_adapter.cc
  ${MUJOCO_DIR}/simulate/glfw_dispatch.cc
  ${MUJOCO_DIR}/simulate/platform_ui_adapter.cc
  ${MUJOCO_DIR}/simulate/simulate.cc
)
target_include_directories(unitree_mujoco PRIVATE
  simulate/src
  simulate/src/lodepng
  ${MUJOCO_DIR}/include
  ${MUJOCO_DIR}/simulate
)
ament_target_dependencies(unitree_mujoco
  ament_index_cpp
  builtin_interfaces
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  unitree_go
  unitree_hg
)
target_link_libraries(unitree_mujoco
  ${MUJOCO_LIBRARY}
  Boost::program_options
  glfw
  Threads::Threads
  yaml-cpp::yaml-cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${interfaces_target} "rosidl_typesupport_cpp")
target_link_libraries(unitree_mujoco "${cpp_typesupport_target}")
set_target_properties(unitree_mujoco PROPERTIES
  BUILD_RPATH "${MUJOCO_DIR}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

add_executable(jstest
  simulate/src/joystick/jstest.cc
  simulate/src/joystick/joystick.cc
)
target_include_directories(jstest PRIVATE simulate/src/joystick)

if(BUILD_TESTING)
  enable_testing()
  add_executable(scene_smoke test/scene_smoke.cc)
  target_include_directories(scene_smoke PRIVATE "${MUJOCO_DIR}/include")
  target_link_libraries(scene_smoke "${MUJOCO_LIBRARY}")
  file(GLOB scene_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/unitree_robots/*/scene*.xml")
  foreach(scene IN LISTS scene_files)
    file(RELATIVE_PATH scene_name "${CMAKE_CURRENT_SOURCE_DIR}/unitree_robots" "${scene}")
    string(REPLACE "/" "_" scene_name "${scene_name}")
    add_test(NAME "scene_${scene_name}" COMMAND scene_smoke "${scene}")
  endforeach()
endif()

install(TARGETS unitree_mujoco jstest
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  simulate/config.yaml
  simulate/config/go2.yaml
  simulate/config/eval_beam.yaml
  simulate/config/eval_trampoline.yaml
  simulate/camera_profiles.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY unitree_robots/
  DESTINATION share/${PROJECT_NAME}/unitree_robots
)

ament_export_dependencies(rosidl_default_runtime)

get_filename_component(MUJOCO_LIBRARY_REAL "${MUJOCO_LIBRARY}" REALPATH)
install(FILES "${MUJOCO_LIBRARY_REAL}" DESTINATION lib)
# Remove plugins left by older distributions before installing the current set.
install(CODE "file(REMOVE_RECURSE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mujoco_plugin\")")
install(DIRECTORY "${MUJOCO_DIR}/bin/mujoco_plugin/"
  DESTINATION lib/${PROJECT_NAME}/mujoco_plugin
  FILES_MATCHING PATTERN "*.so"
)

ament_package()
