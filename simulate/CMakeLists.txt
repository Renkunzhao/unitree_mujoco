cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

include(FetchContent)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
)
link_directories(mujoco/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})
find_package(legged_base QUIET)
if(legged_base_FOUND)
    message("Found legged_base logger support")
    target_link_libraries(unitree_mujoco legged_base::legged_base)
    target_compile_definitions(unitree_mujoco PRIVATE LOGGER)
else()
    message("legged_base logger support not found")
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

install(TARGETS unitree_mujoco jstest
    DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY ../unitree_robots DESTINATION .)
install(FILES config.yaml DESTINATION bin)

set(CMAKE_BUILD_TYPE Release)
