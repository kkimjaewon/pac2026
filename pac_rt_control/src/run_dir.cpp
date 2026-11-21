#include "rt_control/run_dir.hpp"

#include <filesystem>
#include <system_error>

namespace rt_control {

RunDir prepare_run_dir(const std::string& log_root, const std::string& run_id_param, const std::string& run_tag,
                       const std::string& time_stamp) {
  namespace fs = std::filesystem;
  RunDir out;
  const bool fixed = !run_id_param.empty();
  if (fixed) {
    if (run_id_param.find('/') != std::string::npos || run_id_param == "." || run_id_param == "..") {
      out.error = "run_id에 '/'를 넣거나 '.', '..'를 쓸 수 없음: " + run_id_param;
      return out;
    }
    out.run_id = run_id_param;
  } else {
    out.run_id = time_stamp + (run_tag.empty() ? "" : "_" + run_tag);
  }
  out.path = log_root + "/" + out.run_id;

  std::error_code ec;
  fs::create_directories(out.path, ec);
  if (ec) {
    out.error = "실행 기록 폴더를 만들 수 없음: " + out.path + " (" + ec.message() + ")";
    return out;
  }
  if (!fs::is_directory(out.path, ec)) {
    out.error = "실행 기록 경로가 폴더가 아님: " + out.path;
    return out;
  }
  // 정한 run_id 폴더는 다시 쓸 수 있으니, 이전 rt.csv가 있으면 멈춤
  if (fixed && fs::exists(out.path + "/rt.csv", ec)) {
    out.error = "이미 rt.csv가 있음 (덮어쓰지 않음): " + out.path + "/rt.csv";
    return out;
  }
  out.ok = true;
  return out;
}

}  // namespace rt_control
