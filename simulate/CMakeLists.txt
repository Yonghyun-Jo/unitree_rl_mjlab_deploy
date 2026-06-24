cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

# Local isolated dependency prefix (see deploy/robots/g1/CMakeLists.txt). Avoids /opt + sudo.
get_filename_component(DEPS_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../.deps" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH "${DEPS_PREFIX}")
link_directories(${DEPS_PREFIX}/lib)
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

# jstest: standalone joystick diagnostic. Disabled — its xKeySwitchUnion field names
# (right/down/left/value) don't match the bundled unitree_sdk2 version. Not needed for
# sim2sim (the simulator is unitree_mujoco). Re-enable only if the SDK struct matches.
# add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)