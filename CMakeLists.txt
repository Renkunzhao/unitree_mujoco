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
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/simulate/mujoco" CACHE PATH
    "Path to an extracted MuJoCo distribution")
set(MUJOCO_LIBRARY "${MUJOCO_DIR}/lib/libmujoco.so")
if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h" OR
   NOT EXISTS "${MUJOCO_LIBRARY}")
  message(FATAL_ERROR
    "MuJoCo was not found at ${MUJOCO_DIR}. Set MUJOCO_DIR or create "
    "simulate/mujoco pointing to an extracted MuJoCo distribution.")
endif()

add_executable(unitree_mujoco
  simulate/src/main.cc
  simulate/src/depth_camera.cc
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
set_target_properties(unitree_mujoco PROPERTIES
  BUILD_RPATH "${MUJOCO_DIR}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

add_executable(jstest
  simulate/src/joystick/jstest.cc
  simulate/src/joystick/joystick.cc
)
target_include_directories(jstest PRIVATE simulate/src/joystick)
install(TARGETS unitree_mujoco jstest
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  simulate/config.yaml
  simulate/camera_profiles.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY unitree_robots/
  DESTINATION share/${PROJECT_NAME}/unitree_robots
)

get_filename_component(MUJOCO_LIBRARY_REAL "${MUJOCO_LIBRARY}" REALPATH)
install(FILES "${MUJOCO_LIBRARY_REAL}" DESTINATION lib)
install(DIRECTORY "${MUJOCO_DIR}/bin/mujoco_plugin/"
  DESTINATION lib/${PROJECT_NAME}/mujoco_plugin
  FILES_MATCHING PATTERN "*.so"
)

ament_package()
