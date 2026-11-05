// BuildInfo.cpp — 이 바이너리를 빌드한 소스의 커밋(cmake/build_info.cmake 가 빌드마다 g1_build_info.h 를 쓴다).
// 계측 CSV(G1_OBS_CSV) 머리가 읽는다. 커밋이 바뀔 때만 이 파일이 다시 컴파일된다.
#include "g1_build_info.h"

namespace g1 {
const char* build_commit() { return G1_BUILD_COMMIT; }
int build_dirty() { return G1_BUILD_DIRTY; }   // 0 깨끗 · 1 추적 파일 변경 있음 · −1 모름
}  // namespace g1
