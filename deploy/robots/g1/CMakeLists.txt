cmake_minimum_required(VERSION 3.12)
project(g1_controller)

set(CMAKE_CXX_STANDARD 17)

# 🔴 빌드 타입을 못박는다. 지정하지 않으면 cmake 는 최적화 없이(-O0) 빌드한다.
# 그러면 sim(com1)과 실기(로봇 Jetson)가 서로 다른 최적화로 돌아 sim2sim 이
# 검증 능력을 잃는다 — 실제로 2026-08-17, -O3 에서만 발현하는 UB(State_Mimic.cpp
# anchor Rot6D dangling temporary)를 com1(-O0)이 못 잡아 실기에서만 즉발 발산했다.
# 두 머신은 아키텍처가 달라 바이너리를 공유할 수 없으므로, 최소한 "같은 조건으로
# 각자 빌드"는 보장해야 한다. 명시적으로 -DCMAKE_BUILD_TYPE=... 을 주면 그것이 이긴다.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "build type (sim/실기 동일하게 유지)" FORCE)
endif()
message(STATUS "[g1_controller] CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

# Local isolated dependency prefix (unitree_sdk2 + bundled cyclonedds, installed via
# .deps/src/unitree_sdk2 -> make install -DCMAKE_INSTALL_PREFIX). Avoids /usr/local + sudo.
get_filename_component(DEPS_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../../.deps" ABSOLUTE)
link_directories(${DEPS_PREFIX}/lib)
list(APPEND CMAKE_PREFIX_PATH ${DEPS_PREFIX})

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

include_directories(
  /usr/include/eigen3
  ${DEPS_PREFIX}/include
  ${DEPS_PREFIX}/include/ddscxx
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
  ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy
)

file(GLOB CNPY_SRC ${PROJECT_SOURCE_DIR}/../../thirdparty/cnpy/*.cpp)

add_library(cnpy_lib STATIC ${CNPY_SRC})

link_libraries(
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
  Boost::program_options
  yaml-cpp
  fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
  cnpy_lib
  ZLIB::ZLIB
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

# 빌드 정보(커밋 · 고친 파일 여부) — 계측 CSV(G1_OBS_CSV) 머리에 싣는다. 빌드마다 git 을 읽고, 바뀌었을 때만 헤더를
# 갈아 끼운다(같으면 src/BuildInfo.cpp 도 다시 컴파일하지 않는다). git 이 없으면 "unknown" — 빌드는 멈추지 않는다.
set(G1_BUILD_INFO_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_custom_target(g1_build_info
  COMMAND ${CMAKE_COMMAND} -DSRC=${PROJECT_SOURCE_DIR} -DOUT=${G1_BUILD_INFO_DIR}/g1_build_info.h
          -P ${PROJECT_SOURCE_DIR}/cmake/build_info.cmake
  BYPRODUCTS ${G1_BUILD_INFO_DIR}/g1_build_info.h
  COMMENT "g1 build info (commit)")
file(MAKE_DIRECTORY ${G1_BUILD_INFO_DIR})

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
add_dependencies(${PROJECT_NAME}_lib g1_build_info)
target_include_directories(${PROJECT_NAME}_lib PRIVATE ${G1_BUILD_INFO_DIR})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)