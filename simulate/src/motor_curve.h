#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "param.h"

enum class MotorCurveMode
{
  kDcMotor,
  kMotoringOnly,
};

struct MotorActuatorSpec
{
  int actuator_id = -1;
  int dof_adr = -1;
  double gear = 1.0;
  double effort_limit = 0.0;
  double saturation_effort = 0.0;
  double velocity_limit = 0.0;
};

// Enforces physical DC-motor torque-speed (T-omega) limits at every physics step
// using instantaneous joint velocities (data->qvel) right before mj_step / mj_forward.
class MotorCurveLimiter
{
public:
  MotorCurveLimiter() = default;

  static MotorCurveLimiter FromModel(const mjModel* model, const param::MotorCurveConfig& config);
  static MotorCurveMode ParseMode(const std::string& mode);
  static double ClipTorque(
      double tau_cmd, double dq, const MotorActuatorSpec& spec, MotorCurveMode mode);

  bool enabled() const { return enabled_; }
  MotorCurveMode mode() const { return mode_; }
  double nominal_voltage_v() const { return nominal_voltage_v_; }
  double bus_voltage_v() const { return bus_voltage_v_; }
  const std::vector<MotorActuatorSpec>& specs() const { return specs_; }

  void Apply(mjData* data) const;

private:
  bool enabled_ = false;
  MotorCurveMode mode_ = MotorCurveMode::kDcMotor;
  double nominal_voltage_v_ = 28.8;
  double bus_voltage_v_ = 28.8;
  std::vector<MotorActuatorSpec> specs_;
};
