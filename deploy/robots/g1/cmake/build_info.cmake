# build_info.cmake — 빌드마다(add_custom_target g1_build_info) 소스의 커밋 · 고친 파일 여부를 g1_build_info.h 로 쓴다.
# 내용이 같으면 파일을 건드리지 않는다(copy_if_different) → BuildInfo.cpp 도 다시 컴파일하지 않는다.
# 계측 CSV(G1_OBS_CSV) 머리에 «이 바이너리가 어느 소스로 빌드됐나» 를 싣기 위한 것 — 실행 때 작업 트리를 읽으면
# «작업 트리 ≠ 빌드한 트리» 를 못 가린다(memory working-tree-builds-is-not-head-builds).
#   cmake -DSRC=<g1 폴더> -DOUT=<헤더 경로> -P build_info.cmake
# git 이 없거나 실패하면 commit "unknown" · dirty −1 (빌드는 멈추지 않는다).
execute_process(COMMAND git -C "${SRC}" rev-parse --short=12 HEAD
                OUTPUT_VARIABLE G1_C RESULT_VARIABLE rc OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT rc EQUAL 0 OR "${G1_C}" STREQUAL "")
  set(G1_C "unknown")
endif()
# 추적 중인 파일의 변경만 본다(남의 미추적 파일은 빌드 결과와 무관하다)
execute_process(COMMAND git -C "${SRC}" status --porcelain --untracked-files=no
                OUTPUT_VARIABLE G1_S RESULT_VARIABLE rc2 ERROR_QUIET)
if(NOT rc2 EQUAL 0)
  set(G1_D -1)
elseif("${G1_S}" STREQUAL "")
  set(G1_D 0)
else()
  set(G1_D 1)
endif()
file(WRITE "${OUT}.tmp" "// 생성 파일(cmake/build_info.cmake) — 고치지 말 것\n#pragma once\n#define G1_BUILD_COMMIT \"${G1_C}\"\n#define G1_BUILD_DIRTY ${G1_D}\n")
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OUT}.tmp" "${OUT}")
file(REMOVE "${OUT}.tmp")
