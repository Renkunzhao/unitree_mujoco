#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "motor_curve.h"
#include "param.h"

namespace
{
void CheckNear(double actual, double expected, double tol, const char* label)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > tol)
  {
    throw std::runtime_error(
        std::string(label) + ": expected " + std::to_string(expected) + ", got " +
        std::to_string(actual));
  }
}

void TestYamlLoading(const std::filesystem::path& config_path)
{
  param::config = param::SimulationConfig{};
  param::config.load_from_yaml(config_path, std::nullopt, std::nullopt);
  if (param::config.depth_camera.enabled)
  {
    throw std::runtime_error("Expected go2.yaml depth_camera.enabled=false by default");
  }
  if (!param::config.motor_curve.enabled)
  {
    throw std::runtime_error("motor_curve must still load when depth_camera.enabled=false");
  }
  if (param::config.motor_curve.mode != "dc_motor")
  {
    throw std::runtime_error("Expected go2.yaml motor_curve.mode=dc_motor");
  }
  CheckNear(param::config.motor_curve.nominal_voltage_v, 28.8, 1e-9, "nominal_voltage_v");
  CheckNear(param::config.motor_curve.bus_voltage_v, 28.8, 1e-9, "bus_voltage_v");
  CheckNear(param::config.motor_curve.hip_thigh.effort_limit, 23.70, 1e-9, "hip_thigh.effort");
  CheckNear(param::config.motor_curve.hip_thigh.saturation_effort, 66.33, 1e-9, "hip_thigh.stall");
  CheckNear(param::config.motor_curve.hip_thigh.velocity_limit, 34.64, 1e-9, "hip_thigh.w0");
  CheckNear(param::config.motor_curve.calf.effort_limit, 45.43, 1e-9, "calf.effort");
  CheckNear(param::config.motor_curve.calf.saturation_effort, 127.15, 1e-9, "calf.stall");
  CheckNear(param::config.motor_curve.calf.velocity_limit, 18.07, 1e-9, "calf.w0");
}

void TestReferenceEnvelopePoints()
{
  const MotorActuatorSpec calf_28p8{0, 0, 1.0, 45.43, 127.15, 18.07};
  // 28.8 V calf dc_motor checkpoints from actuator_verification.json / go2_motor_characteristics_zh.tex
  CheckNear(MotorCurveLimiter::ClipTorque(60.0, 0.0, calf_28p8, MotorCurveMode::kDcMotor),
            45.43, 1e-6, "calf 28.8V stall box");
  CheckNear(MotorCurveLimiter::ClipTorque(45.43, 11.6126, calf_28p8, MotorCurveMode::kDcMotor),
            45.43, 1e-6, "calf 28.8V knee corner");
  CheckNear(MotorCurveLimiter::ClipTorque(45.43, 18.07, calf_28p8, MotorCurveMode::kDcMotor),
            0.0, 1e-6, "calf 28.8V no-load w0");
  const double expected_18p9 = 127.15 * (1.0 - 18.90 / 18.07);  // -5.84031544 N*m
  CheckNear(MotorCurveLimiter::ClipTorque(10.0, 18.90, calf_28p8, MotorCurveMode::kDcMotor),
            expected_18p9, 1e-5, "calf 28.8V forced overspeed braking at 18.90");
  CheckNear(MotorCurveLimiter::ClipTorque(0.0, 21.50, calf_28p8, MotorCurveMode::kDcMotor),
            127.15 * (1.0 - 21.50 / 18.07), 1e-5, "calf 28.8V zero-cmd braking at 21.50");
  CheckNear(MotorCurveLimiter::ClipTorque(0.0, 24.53, calf_28p8, MotorCurveMode::kDcMotor),
            -45.43, 1e-5, "calf 28.8V saturated braking at w_sat=24.53");

  // 25.5 V scaled calf envelope
  const double sc_25p5 = 25.5 / 28.8;
  const MotorActuatorSpec calf_25p5{
      0, 0, 1.0, 45.43, 127.15 * sc_25p5, 18.07 * sc_25p5};
  CheckNear(calf_25p5.velocity_limit, 15.999479, 1e-4, "calf 25.5V w0");
  CheckNear(calf_25p5.saturation_effort, 112.580729, 1e-4, "calf 25.5V stall");
  CheckNear(
      MotorCurveLimiter::ClipTorque(
          20.0, calf_25p5.velocity_limit, calf_25p5, MotorCurveMode::kDcMotor),
      0.0, 1e-6, "calf 25.5V at scaled w0");

  // motoring_only removes Q2/Q4 forced overspeed braking while keeping Q1/Q3 derating
  CheckNear(MotorCurveLimiter::ClipTorque(10.0, 18.90, calf_28p8, MotorCurveMode::kMotoringOnly),
            0.0, 1e-6, "motoring_only 18.90 req +10 -> 0");
  CheckNear(MotorCurveLimiter::ClipTorque(0.0, 18.90, calf_28p8, MotorCurveMode::kMotoringOnly),
            0.0, 1e-6, "motoring_only 18.90 req 0 -> 0");
  CheckNear(MotorCurveLimiter::ClipTorque(-15.0, 18.90, calf_28p8, MotorCurveMode::kMotoringOnly),
            -15.0, 1e-6, "motoring_only 18.90 regen -15 preserved");
}

void TestMujocoStepConsistency(
    const std::filesystem::path& scene_path, const std::filesystem::path& config_path)
{
  char error[2048]{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
      mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  if (!m)
  {
    throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
  }
  std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
  if (!d)
  {
    throw std::runtime_error("mj_makeData failed");
  }

  param::config = param::SimulationConfig{};
  param::config.load_from_yaml(config_path, std::nullopt, std::nullopt);
  const MotorCurveLimiter limiter = MotorCurveLimiter::FromModel(m.get(), param::config.motor_curve);
  if (!limiter.enabled() || static_cast<int>(limiter.specs().size()) != m->nu)
  {
    throw std::runtime_error("Expected enabled MotorCurveLimiter for all 12 Go2 actuators");
  }

  const std::vector<double> velocities = {-35.0, -24.53, -18.90, -12.0, 0.0, 12.0, 18.90, 24.53, 35.0};
  const std::vector<double> commands = {-100.0, -15.0, 0.0, 15.0, 100.0};

  for (const double vel : velocities)
  {
    for (const double cmd : commands)
    {
      mj_resetData(m.get(), d.get());
      for (const auto& spec : limiter.specs())
      {
        d->qvel[spec.dof_adr] = vel;
        d->ctrl[spec.actuator_id] = cmd;
      }
      limiter.Apply(d.get());
      mj_step(m.get(), d.get());

      for (int i = 0; i < m->nu; ++i)
      {
        const auto& spec = limiter.specs()[static_cast<std::size_t>(i)];
        const double dq_sensor = d->sensordata[i + m->nu];
        // Note: go2.xml has noise="0.01" on jointactuatorfrc sensors, so check actuator_force
        // for exact clipping and sensordata velocity matching the pre-step instantaneous qvel.
        CheckNear(dq_sensor, vel, 1e-9, "sensordata dq matches pre-step qvel");
        const double tau_act = d->actuator_force[i];
        const double w_sat =
            spec.velocity_limit * (1.0 + spec.effort_limit / spec.saturation_effort);
        const double v_c = std::clamp(dq_sensor, -w_sat, w_sat);
        const double tau_max = std::min(
            spec.effort_limit, spec.saturation_effort * (1.0 - v_c / spec.velocity_limit));
        const double tau_min = std::max(
            -spec.effort_limit, spec.saturation_effort * (-1.0 - v_c / spec.velocity_limit));
        if (tau_act > tau_max + 1e-9 || tau_act < tau_min - 1e-9)
        {
          throw std::runtime_error(
              "actuator_force[" + std::to_string(i) + "]=" + std::to_string(tau_act) +
              " violated [" + std::to_string(tau_min) + ", " + std::to_string(tau_max) +
              "] at dq=" + std::to_string(dq_sensor));
        }
      }
    }
  }

  // Verify non-Go2 fallback (disabled config is a no-op)
  param::MotorCurveConfig disabled_cfg = param::config.motor_curve;
  disabled_cfg.enabled = false;
  const MotorCurveLimiter disabled_limiter = MotorCurveLimiter::FromModel(m.get(), disabled_cfg);
  if (disabled_limiter.enabled())
  {
    throw std::runtime_error("Expected disabled_limiter.enabled() == false");
  }
  mj_resetData(m.get(), d.get());
  for (int i = 0; i < m->nu; ++i)
  {
    d->qvel[limiter.specs()[static_cast<std::size_t>(i)].dof_adr] = 30.0;
    d->ctrl[i] = 45.43;
  }
  disabled_limiter.Apply(d.get());
  CheckNear(d->ctrl[2], 45.43, 1e-9, "disabled limiter leaves ctrl untouched");
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "Usage: %s <go2_scene.xml> <go2.yaml>\n", argv[0]);
    return 2;
  }
  try
  {
    TestYamlLoading(argv[2]);
    TestReferenceEnvelopePoints();
    TestMujocoStepConsistency(argv[1], argv[2]);
    std::printf("PASS motor_curve_test (%s, %s)\n", argv[1], argv[2]);
    return 0;
  }
  catch (const std::exception& exc)
  {
    std::fprintf(stderr, "FAIL motor_curve_test: %s\n", exc.what());
    return 1;
  }
}
