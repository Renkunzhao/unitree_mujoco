#pragma once

#include <array>
#include <optional>

#include <mujoco/mujoco.h>

#include "episode_result.h"

class BeamEpisodeMonitor
{
public:
  explicit BeamEpisodeMonitor(const mjModel* model);

  std::optional<TerminalResult> Update(const mjData* data) const;

private:
  int RequireObject(mjtObj type, const char* name) const;
  int RequireSensor(const char* name, int dimension) const;
  void EvaluateContacts(
      const mjData* data, bool* illegal_foot_contact, double* max_self_force,
      double* max_nonfoot_force, std::array<bool, 4>* feet_on_end_platform) const;

  const mjModel* model_ = nullptr;
  int floor_geom_id_ = -1;
  int platform_b_geom_id_ = -1;
  int imu_quaternion_address_ = -1;
  std::array<int, 4> foot_geom_ids_{};
  std::array<int, 4> foot_site_ids_{};
};
