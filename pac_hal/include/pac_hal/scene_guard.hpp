// Copyright 2026 Jaewon Kim
#pragma once
#include <memory>
#include "pac_hal/hal_types.hpp"

namespace pac_hal
{
// Fixed-tool Gazebo workcell guard for the configured robot. Unmodeled obstacles/tools are not covered.
class SceneGuard
{
public:
  SceneGuard(const Config & config, const std::string & scene_file);
  ~SceneGuard();
  bool enabled() const;
  bool ready() const;
  Result initialize(const std::string & urdf);
  Result check(const Joints & q);
  Result check_path(const Joints & from, const Joints & to, int64_t deadline_ns = 0);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace pac_hal
