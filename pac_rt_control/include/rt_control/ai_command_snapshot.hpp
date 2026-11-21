// A 명령 한 개를 B 안에서 들고 다니는 형식 (ROS 의존 없음)
//
// 수신기(ai_command_receiver)가 메시지를 이 구조체로 바꿔 Seqlock 버퍼에 쓰고,
// 1ms 루프와 gate가 이 구조체를 읽는다. gate·테스트가 ROS 없이 빌드되도록 따로 둔다.
#pragma once

#include <cstdint>

namespace rt_control {

// Seqlock 버퍼에 들어가는 A 명령 한 개 (메시지 필드 + B가 받은 시각)
struct AiCommandSnapshot {
  uint64_t seq;
  uint32_t writer_boot_id;
  uint16_t layout_version;
  uint8_t command_type;
  uint8_t status;
  double target[4];
  double confidence;
  int64_t age_ns;        // A가 보낼 때까지 이미 지난 시간
  int64_t recv_mono_ns;  // B가 받은 시각 (CLOCK_MONOTONIC)
};

}  // namespace rt_control
