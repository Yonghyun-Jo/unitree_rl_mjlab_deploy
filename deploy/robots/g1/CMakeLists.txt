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

# 아케이드(g1_arcade core + 네이티브 호스트 층 — include/ArcadeHost.h) — G1_ARCADE_DIR 이 있을 때만.
#   G1_ARCADE_DIR = g1_arcade 폴더(core/CMakeLists.txt · host/CMakeLists.txt). 기본 = third_party/g1_arcade,
#                   com1 = -DG1_ARCADE_DIR=/home/piene/g1_arcade_2b.
#   없으면: 아케이드 코드 없이 지금과 같은 빌드(경고 한 줄 — 소스 목록 · 플래그 그대로).
#   있으면: arcade_core(g1 드라이버 헤더 14 개 = 이 폴더 include/ 가 정본) + arcade_host_native(ORT = 이 빌드의 ONNXRUNTIME_DIR —
#          ORT 두 벌 금지)를 g1_controller_lib 에 링크, G1_HAVE_ARCADE=1, src/ArcadeHost.cpp 를 넣는다. 시험 타깃 test_arcade_host
#          (EXCLUDE_FROM_ALL — 기본 make 에 안 든다, tests/run_unit_tests.sh 가 이것만 빌드해 돌린다).
# 🔴 아래 전역 include_directories · link_libraries 보다 먼저 넣는다 — 전역 값(이 폴더 include/ · DDS · cnpy …)이 아케이드 타깃에
#    섞이지 않게(디렉터리 속성은 그 뒤에 넣는 하위 폴더로 물려 내려간다).
set(G1_ARCADE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/g1_arcade" CACHE PATH "g1_arcade 폴더 (없으면 아케이드 없이 빌드)")
if(EXISTS "${G1_ARCADE_DIR}/core/CMakeLists.txt" AND EXISTS "${G1_ARCADE_DIR}/host/CMakeLists.txt")
  set(G1_HAVE_ARCADE TRUE)
  set(ARCADE_G1_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
  set(ARCADE_ORT_DIR ${ONNXRUNTIME_DIR})
  set(ARCADE_HOST_TESTS OFF)
  add_subdirectory(${G1_ARCADE_DIR}/core ${CMAKE_CURRENT_BINARY_DIR}/g1_arcade/core)
  add_subdirectory(${G1_ARCADE_DIR}/host ${CMAKE_CURRENT_BINARY_DIR}/g1_arcade/host)
  message(STATUS "[g1_controller] 아케이드 켬: ${G1_ARCADE_DIR}")
else()
  set(G1_HAVE_ARCADE FALSE)
  message(WARNING "[g1_controller] 아케이드 없음(G1_ARCADE_DIR=${G1_ARCADE_DIR}) — 아케이드 코드 없이 빌드한다")
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
if(NOT G1_HAVE_ARCADE)
  list(FILTER ADD_SRC_LIST EXCLUDE REGEX "/src/Arcade(Host|Link)\\.cpp$")  # 아케이드 없는 빌드 = 지금과 같은 소스 목록
endif()

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
if(G1_HAVE_ARCADE)
  target_link_libraries(${PROJECT_NAME}_lib PUBLIC arcade_host_native)
  target_compile_definitions(${PROJECT_NAME}_lib PUBLIC G1_HAVE_ARCADE=1)
  # 시험은 ArcadeHost.cpp 를 직접 넣는다(g1_controller_lib 링크 = State_Mimic 전체 빌드를 피한다) — 그래서 아래 link_libraries
  # (g1_controller_lib) 보다 앞에 만든다. 묶음 경로 = 환경 G1_ARCADE_BUNDLE(없으면 시험이 skip 한 줄).
  add_executable(test_arcade_host EXCLUDE_FROM_ALL tests/test_arcade_host.cpp src/ArcadeHost.cpp)
  target_compile_definitions(test_arcade_host PRIVATE G1_HAVE_ARCADE=1 G1_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  target_compile_options(test_arcade_host PRIVATE -Wall -Wextra)
  target_link_libraries(test_arcade_host PRIVATE arcade_host_native arcade_onnxruntime ${CMAKE_DL_LIBS})
  # 아케이드 경로 판정(ArcadeGate.h) + 한 정책 틱(ArcadeLink: 의도 칸 → core → 상태 칸)의 malloc 수준 할당 0 · config_arcade.yaml 대조.
  # 같은 시험 파일을 run_unit_tests.sh 가 아케이드 없이도 g++ 로 바로 돌린다(그때는 ArcadeLink 틱 시험만 빠진다).
  add_executable(test_arcade_gate EXCLUDE_FROM_ALL tests/test_arcade_gate.cpp src/ArcadeLink.cpp src/ArcadeHost.cpp)
  target_compile_definitions(test_arcade_gate PRIVATE G1_HAVE_ARCADE=1 G1_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  target_compile_options(test_arcade_gate PRIVATE -Wall -Wextra)
  target_include_directories(test_arcade_gate PRIVATE /usr/include/eigen3)
  target_link_libraries(test_arcade_gate PRIVATE arcade_host_native arcade_onnxruntime yaml-cpp ${CMAKE_DL_LIBS})
endif()
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)