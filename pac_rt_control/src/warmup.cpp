#include "rt_control/warmup.hpp"

namespace rt_control {

uint32_t StartupWarmup::update(const WarmupInput& in) {
  if (!started_) {
    started_ = true;
    start_ns_ = in.now_ns;
    last_overruns_ = in.publish_overruns;
    stable_since_ns_ = in.now_ns;
    ticks_at_stable_ = in.publish_ticks;
  }
  if (in.new_state && states_ < config_.states) ++states_;

  // overrun이 늘면 안정 구간을 처음부터 다시 셈 (유지 명령 구간도)
  const bool overrun = in.publish_overruns != last_overruns_;
  if (overrun) {
    last_overruns_ = in.publish_overruns;
    stable_since_ns_ = in.now_ns;
    ticks_at_stable_ = in.publish_ticks;
  }
  // 유지 명령: 끊기면 처음부터, 다시 시작하거나 overrun이 나면 그 시각부터 셈
  if (in.holding && (!was_holding_ || overrun)) {
    hold_since_ns_ = in.now_ns;
    published_at_hold_ = in.published;
  }
  was_holding_ = in.holding;
  const uint64_t min_ticks =
      config_.publish_period_ns > 0 ? static_cast<uint64_t>(config_.stable_ns / config_.publish_period_ns / 2) : 0;
  const bool publish_stable = in.now_ns - stable_since_ns_ >= config_.stable_ns &&
                              in.publish_ticks - ticks_at_stable_ >= min_ticks && in.publish_ticks > 0;

  uint32_t met = 0;
  if (states_ >= config_.states) met |= WARM_STATES;
  if (publish_stable) met |= WARM_PUBLISH;
  if (in.subscribers >= 1) met |= WARM_SUBSCRIBER;
  // 유지 명령이 hold_ns 동안 이어졌고, 그동안 발행 스레드가 실제로 절반 이상 내보냄
  const uint64_t min_published =
      config_.publish_period_ns > 0 ? static_cast<uint64_t>(config_.hold_ns / config_.publish_period_ns / 2) : 0;
  if (config_.hold_ns <= 0 || (in.holding && in.now_ns - hold_since_ns_ >= config_.hold_ns &&
                               in.published - published_at_hold_ >= min_published)) {
    met |= WARM_HOLD;
  }
  met_ = met;
  return met;
}

}  // namespace rt_control
