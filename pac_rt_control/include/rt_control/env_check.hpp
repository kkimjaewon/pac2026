// 시작할 때 한 번 하는 환경 점검 (ROS 의존 없음)
//
// 실시간 성능에 영향을 주는 PC 설정을 읽어서, 문제가 있으면 경고 문장을 만든다.
//   - 커널에 PREEMPT_RT가 없음
//   - 전원 프로파일이 performance가 아님 (못 읽으면 건너뜀)
//   - fifo를 켰는데 실시간 우선순위 한도(ulimit -r)가 0 (root면 괜찮음)
// 읽기(read_env_info)와 판단(env_warnings)을 나눠서, 판단은 단위 테스트로 확인한다.
#pragma once

#include <string>
#include <vector>

namespace rt_control {

struct EnvInfo {
  bool kernel_known = false;    // 커널 정보를 읽었는지
  bool preempt_rt = false;      // /sys/kernel/realtime == 1 또는 uname -v에 PREEMPT_RT
  std::string kernel_release;   // uname -r (경고 문장용)
  std::string power_profile;    // "performance", "balanced", ... 못 읽으면 빈 문자열
  bool rtprio_known = false;
  long rtprio_limit = 0;        // RLIMIT_RTPRIO 현재값 (ulimit -r)
  bool is_root = false;
  bool fifo_requested = false;  // 파라미터 fifo
};

// 지금 PC에서 읽음. 전원 프로파일은 powerprofilesctl get, 안 되면 /sys/firmware/acpi/platform_profile
EnvInfo read_env_info(bool fifo_requested);

// 경고 문장 목록 (문제가 없으면 빈 목록)
std::vector<std::string> env_warnings(const EnvInfo& info);

}  // namespace rt_control
