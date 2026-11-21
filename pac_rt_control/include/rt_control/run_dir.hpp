// 실행 기록 폴더 정하기·만들기 (ROS 의존 없음, 계약 v0.3 인터페이스 5-2)
//
// run_id 파라미터가 비어 있으면: <log_root>/<시각>[_<run_tag>]/ 를 새로 만든다 (예전 동작)
// run_id 파라미터가 있으면   : <log_root>/<run_id>/ 를 그대로 쓴다 (run_tag는 무시)
//   - 폴더가 이미 있어도 됨 (다른 계층이 같은 run_id 폴더를 먼저 만들었을 수 있음)
//   - 그 안에 rt.csv가 이미 있으면 덮어쓰지 않고 오류 (이전 실행 기록 보호)
//   - run_id에 '/'가 있거나 "." / ".." 이면 오류 (log_root 밖으로 나가지 않게)
#pragma once

#include <string>

namespace rt_control {

struct RunDir {
  bool ok = false;
  std::string run_id;  // 실제로 쓴 run_id
  std::string path;    // <log_root>/<run_id>
  std::string error;   // ok가 false일 때 이유
};

// time_stamp: 시작 시각 문자열 (YYYYMMDD_HHMMSS). run_id_param이 비어 있을 때만 씀
RunDir prepare_run_dir(const std::string& log_root, const std::string& run_id_param, const std::string& run_tag,
                       const std::string& time_stamp);

}  // namespace rt_control
