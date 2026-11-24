#include "motor_curve.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>

namespace
{
std::string ToLowerAscii(std::string text)
{
  for (char& ch : text)
  {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return text;
}

bool ContainsToken(const std::string& haystack, const char* needle)
{
  return haystack.find(needle) != std::string::npos;
}

const param::MotorGroupCurveConfig& ResolveGroupConfig(
    const mjModel* model, int actuator_id, int joint_id, const param::MotorCurveConfig& config)
{
  const char* raw_act_name = mj_id2name(model, mjOBJ_ACTUATOR, actuator_id);
  const char* raw_jnt_name = mj_id2name(model, mjOBJ_JOINT, joint_id);
  const std::string act_name = raw_act_name ? ToLowerAscii(raw_act_name) : "";
  const std::string jnt_name = raw_jnt_name ? ToLowerAscii(raw_jnt_name) : "";

  if (ContainsToken(act_name, "calf") || ContainsToken(act_name, "knee") ||
      ContainsToken(jnt_name, "calf") || ContainsToken(jnt_name, "knee"))
  {
    return config.calf;
  }
  if (ContainsToken(act_name, "hip") || ContainsToken(act_name, "thigh") ||
      ContainsToken(jnt_name, "hip") || ContainsToken(jnt_name, "thigh"))
  {
    return config.hip_thigh;
  }

  throw std::runtime_error(
      "motor_curve cannot classify actuator[" + std::to_string(actuator_id) + "] ('" +
      act_name + "', joint '" + jnt_name + "') into hip_thigh or calf");
}
}  // namespace

MotorCurveMode MotorCurveLimiter::ParseMode(const std::string& mode)
{
  if (mode == "dc_motor")
  {
    return MotorCurveMode::kDcMotor;
  }
  if (mode == "motoring_only")
  {
    return MotorCurveMode::kMotoringOnly;
  }
  throw std::runtime_error("unsupported motor_curve.mode: " + mode);
}

MotorCurveLimiter MotorCurveLimiter::FromModel(
    const mjModel* model, const param::MotorCurveConfig& config)
{
  MotorCurveLimiter limiter;
  limiter.nominal_voltage_v_ = config.nominal_voltage_v;
  limiter.bus_voltage_v_ = config.bus_voltage_v;
  limiter.mode_ = ParseMode(config.mode);

  if (!config.enabled)
  {
    return limiter;
  }
  if (!model)
  {
    throw std::runtime_error("MotorCurveLimiter::FromModel requires a non-null mjModel");
  }
  if (!std::isfinite(config.nominal_voltage_v) || config.nominal_voltage_v <= 0.0 ||
      !std::isfinite(config.bus_voltage_v) || config.bus_voltage_v <= 0.0)
  {
    throw std::runtime_error("motor_curve voltages must be finite and positive");
  }

  const double v_scale = config.bus_voltage_v / config.nominal_voltage_v;
  limiter.specs_.reserve(static_cast<std::size_t>(model->nu));

  for (int i = 0; i < model->nu; ++i)
  {
    if (model->actuator_trntype[i] != mjTRN_JOINT)
    {
      throw std::runtime_error(
          "motor_curve requires joint actuators; actuator[" + std::to_string(i) + "] is not mjTRN_JOINT");
    }
    const int joint_id = model->actuator_trnid[2 * i];
    if (joint_id < 0 || joint_id >= model->njnt || model->jnt_type[joint_id] != mjJNT_HINGE)
    {
      throw std::runtime_error(
          "motor_curve requires hinge joint actuators for actuator[" + std::to_string(i) + "]");
    }
    const int dof_adr = model->jnt_dofadr[joint_id];
    const double gear = model->actuator_gear[6 * i];
    if (!std::isfinite(gear) || gear <= 0.0)
    {
      throw std::runtime_error(
          "motor_curve requires positive gear ratio for actuator[" + std::to_string(i) + "]");
    }

    const auto& group = ResolveGroupConfig(model, i, joint_id, config);
    double effort_limit = group.effort_limit;
    if (model->actuator_ctrllimited[i])
    {
      const double lo = model->actuator_ctrlrange[2 * i];
      const double hi = model->actuator_ctrlrange[2 * i + 1];
      const double xml_effort_limit = std::min(std::abs(lo), std::abs(hi)) * gear;
      effort_limit = std::min(effort_limit, xml_effort_limit);
    }

    MotorActuatorSpec spec;
    spec.actuator_id = i;
    spec.dof_adr = dof_adr;
    spec.gear = gear;
    spec.effort_limit = effort_limit;
    spec.saturation_effort = group.saturation_effort * v_scale;
    spec.velocity_limit = group.velocity_limit * v_scale;
    limiter.specs_.push_back(spec);
  }

  limiter.enabled_ = true;
  return limiter;
}

double MotorCurveLimiter::ClipTorque(
    double tau_cmd, double dq, const MotorActuatorSpec& spec, MotorCurveMode mode)
{
  if (mode == MotorCurveMode::kDcMotor)
  {
    const double w_sat = spec.velocity_limit * (1.0 + spec.effort_limit / spec.saturation_effort);
    const double v_c = std::clamp(dq, -w_sat, w_sat);
    const double tau_max =
        std::min(spec.effort_limit, spec.saturation_effort * (1.0 - v_c / spec.velocity_limit));
    const double tau_min =
        std::max(-spec.effort_limit, spec.saturation_effort * (-1.0 - v_c / spec.velocity_limit));
    return std::clamp(tau_cmd, tau_min, tau_max);
  }

  const double foc_cap = std::min(
      spec.effort_limit,
      std::max(0.0, spec.saturation_effort * (1.0 - std::abs(dq) / spec.velocity_limit)));
  const double tau_max = (dq >= 0.0) ? foc_cap : spec.effort_limit;
  const double tau_min = (dq <= 0.0) ? -foc_cap : -spec.effort_limit;
  return std::clamp(tau_cmd, tau_min, tau_max);
}

void MotorCurveLimiter::Apply(mjData* data) const
{
  if (!enabled_ || !data)
  {
    return;
  }
  for (const auto& spec : specs_)
  {
    const double dq = data->qvel[spec.dof_adr];
    const double tau_cmd = data->ctrl[spec.actuator_id] * spec.gear;
    const double tau_out = ClipTorque(tau_cmd, dq, spec, mode_);
    data->ctrl[spec.actuator_id] = tau_out / spec.gear;
  }
}
