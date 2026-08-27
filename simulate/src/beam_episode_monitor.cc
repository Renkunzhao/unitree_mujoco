#include "beam_episode_monitor.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <unitree_mujoco/msg/episode_status.hpp>

namespace
{
using EpisodeStatus = unitree_mujoco::msg::EpisodeStatus;

constexpr double kContactForceThresholdN = 10.0;
constexpr double kFallAngleRad = 70.0 * 3.14159265358979323846 / 180.0;
constexpr std::array<const char*, 4> kFootNames{"FL", "FR", "RL", "RR"};

double VectorNorm(const mjtNum* values)
{
  return std::sqrt(
      values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}
}  // namespace

BeamEpisodeMonitor::BeamEpisodeMonitor(const mjModel* model) : model_(model)
{
  if (!model_)
  {
    throw std::runtime_error("beam episode monitor requires a loaded MuJoCo model");
  }

  floor_geom_id_ = RequireObject(mjOBJ_GEOM, "floor");
  platform_b_geom_id_ = RequireObject(mjOBJ_GEOM, "platform_b");
  if (model_->geom_type[platform_b_geom_id_] != mjGEOM_BOX ||
      model_->geom_bodyid[platform_b_geom_id_] != 0)
  {
    throw std::runtime_error("beam monitor end platform must be a world-body box geom");
  }
  if (model_->geom_bodyid[floor_geom_id_] != 0)
  {
    throw std::runtime_error("beam monitor floor must be attached to the world body");
  }

  imu_quaternion_address_ = RequireSensor("imu_quat", 4);
  for (std::size_t index = 0; index < kFootNames.size(); ++index)
  {
    foot_geom_ids_[index] = RequireObject(mjOBJ_GEOM, kFootNames[index]);
    const std::string site_name = std::string(kFootNames[index]) + "_site";
    foot_site_ids_[index] = RequireObject(mjOBJ_SITE, site_name.c_str());
  }
}

int BeamEpisodeMonitor::RequireObject(mjtObj type, const char* name) const
{
  const int id = mj_name2id(model_, type, name);
  if (id < 0)
  {
    throw std::runtime_error("beam monitor requires MuJoCo object '" + std::string(name) + "'");
  }
  return id;
}

int BeamEpisodeMonitor::RequireSensor(const char* name, int dimension) const
{
  const int sensor_id = RequireObject(mjOBJ_SENSOR, name);
  if (model_->sensor_dim[sensor_id] != dimension)
  {
    throw std::runtime_error(
        "beam monitor requires sensor '" + std::string(name) + "' with dimension " +
        std::to_string(dimension));
  }
  return model_->sensor_adr[sensor_id];
}

void BeamEpisodeMonitor::EvaluateContacts(
    const mjData* data, bool* illegal_foot_contact, double* max_self_force,
    double* max_nonfoot_force, std::array<bool, 4>* feet_on_end_platform) const
{
  *illegal_foot_contact = false;
  *max_self_force = 0.0;
  *max_nonfoot_force = 0.0;
  feet_on_end_platform->fill(false);
  mjtNum force[6]{};

  for (int contact_index = 0; contact_index < data->ncon; ++contact_index)
  {
    const auto& contact = data->contact[contact_index];
    const int geom1 = contact.geom[0];
    const int geom2 = contact.geom[1];
    const int body1 = model_->geom_bodyid[geom1];
    const int body2 = model_->geom_bodyid[geom2];
    mj_contactForce(model_, data, contact_index, force);
    const double force_magnitude = VectorNorm(force);

    for (std::size_t foot_index = 0; foot_index < foot_geom_ids_.size(); ++foot_index)
    {
      const int foot_geom = foot_geom_ids_[foot_index];
      if ((geom1 == foot_geom && geom2 == platform_b_geom_id_) ||
          (geom2 == foot_geom && geom1 == platform_b_geom_id_))
      {
        (*feet_on_end_platform)[foot_index] = true;
      }
      if ((geom1 == foot_geom && geom2 == floor_geom_id_) ||
          (geom2 == foot_geom && geom1 == floor_geom_id_))
      {
        *illegal_foot_contact = true;
      }
    }

    if (body1 > 0 && body2 > 0)
    {
      *max_self_force = std::max(*max_self_force, force_magnitude);
    }
    else if ((body1 > 0) != (body2 > 0))
    {
      const int robot_geom = body1 > 0 ? geom1 : geom2;
      if (std::find(foot_geom_ids_.begin(), foot_geom_ids_.end(), robot_geom) ==
          foot_geom_ids_.end())
      {
        *max_nonfoot_force = std::max(*max_nonfoot_force, force_magnitude);
      }
    }
  }
}

std::optional<TerminalResult> BeamEpisodeMonitor::Update(const mjData* data) const
{
  bool illegal_foot_contact = false;
  double max_self_force = 0.0;
  double max_nonfoot_force = 0.0;
  std::array<bool, 4> feet_on_end_platform{};
  EvaluateContacts(
      data, &illegal_foot_contact, &max_self_force, &max_nonfoot_force,
      &feet_on_end_platform);

  if (illegal_foot_contact)
  {
    return TerminalResult{EpisodeStatus::ILLEGAL_CONTACT, "foot contacted the catch floor"};
  }
  if (max_self_force > kContactForceThresholdN)
  {
    return TerminalResult{
        EpisodeStatus::SELF_COLLISION, "self-collision force exceeded 10 N"};
  }
  if (max_nonfoot_force > kContactForceThresholdN)
  {
    return TerminalResult{
        EpisodeStatus::ILLEGAL_CONTACT, "non-foot environment contact exceeded 10 N"};
  }

  const mjtNum* imu_quaternion = data->sensordata + imu_quaternion_address_;
  const mjtNum inverse_quaternion[4]{
      imu_quaternion[0], -imu_quaternion[1], -imu_quaternion[2], -imu_quaternion[3]};
  const mjtNum gravity[3]{0.0, 0.0, -1.0};
  mjtNum projected_gravity[3]{};
  mju_rotVecQuat(projected_gravity, gravity, inverse_quaternion);
  const double tilt = std::acos(std::clamp(-projected_gravity[2], -1.0, 1.0));
  if (tilt > kFallAngleRad)
  {
    return TerminalResult{EpisodeStatus::FELL_OVER, "base tilt exceeded 70 degrees"};
  }

  if (!std::all_of(
          feet_on_end_platform.begin(), feet_on_end_platform.end(),
          [](bool value) { return value; }))
  {
    return std::nullopt;
  }

  for (int site_id : foot_site_ids_)
  {
    const mjtNum* foot_position = data->site_xpos + 3 * site_id;
    const mjtNum* center = model_->geom_pos + 3 * platform_b_geom_id_;
    const mjtNum* half_size = model_->geom_size + 3 * platform_b_geom_id_;
    const bool inside =
        std::abs(foot_position[0] - center[0]) <= half_size[0] &&
        std::abs(foot_position[1] - center[1]) <= half_size[1] &&
        foot_position[2] >= center[2] + half_size[2];
    if (!inside)
    {
      return std::nullopt;
    }
  }
  return TerminalResult{EpisodeStatus::SUCCESS, "all feet reached the end platform"};
}
