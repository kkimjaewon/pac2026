#include "rt_control/env_check.hpp"

#include <sys/resource.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>

namespace rt_control {

namespace {

std::string trim(const std::string& s) {
  const size_t a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  const size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

std::string read_first_line(const char* path) {
  std::ifstream f(path);
  std::string line;
  std::getline(f, line);
  return trim(line);
}

// 명령 출력 첫 줄 (실패하면 빈 문자열). 시작할 때 한 번만 부르므로 popen 사용
std::string command_first_line(const char* cmd) {
  std::FILE* p = popen(cmd, "r");
  if (!p) return "";
  char buf[128] = {};
  const bool got = std::fgets(buf, sizeof(buf), p) != nullptr;
  const int status = pclose(p);
  return (got && status == 0) ? trim(buf) : "";
}

}  // namespace

EnvInfo read_env_info(bool fifo_requested) {
  EnvInfo info;
  info.fifo_requested = fifo_requested;

  utsname un{};
  if (uname(&un) == 0) {
    info.kernel_known = true;
    info.kernel_release = un.release;
    const std::string version = un.version;
    info.preempt_rt = read_first_line("/sys/kernel/realtime") == "1" ||
                      version.find("PREEMPT_RT") != std::string::npos;
  }

  info.power_profile = command_first_line("powerprofilesctl get 2>/dev/null");
  if (info.power_profile.empty()) info.power_profile = read_first_line("/sys/firmware/acpi/platform_profile");

  rlimit rl{};
  if (getrlimit(RLIMIT_RTPRIO, &rl) == 0) {
    info.rtprio_known = true;
    info.rtprio_limit = rl.rlim_cur == RLIM_INFINITY ? 99 : static_cast<long>(rl.rlim_cur);
  }
  info.is_root = geteuid() == 0;
  return info;
}

std::vector<std::string> env_warnings(const EnvInfo& info) {
  std::vector<std::string> w;
  if (info.kernel_known && !info.preempt_rt) {
    w.push_back("커널에 PREEMPT_RT가 없음 (" + info.kernel_release +
                "). 지터가 수백 us까지 커질 수 있음 (RT 커널로 부팅 권장)");
  }
  if (!info.power_profile.empty() && info.power_profile != "performance") {
    w.push_back("전원 프로파일이 " + info.power_profile +
                " (performance 아님). 측정 전에: powerprofilesctl set performance");
  }
  if (info.fifo_requested && info.rtprio_known && info.rtprio_limit == 0 && !info.is_root) {
    w.push_back("fifo:=true인데 실시간 우선순위 한도(ulimit -r)가 0. SCHED_FIFO 설정이 실패함 "
                "(/etc/security/limits.d 에 rtprio 설정 필요)");
  }
  return w;
}

}  // namespace rt_control
