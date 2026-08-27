#include "episode_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace
{
constexpr auto kStatusPeriod = std::chrono::milliseconds(50);

double VectorNorm(const mjtNum* values)
{
  return std::sqrt(
      values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
}

bool IsTaskTerminalOutcome(std::uint8_t outcome)
{
  using EpisodeStatus = unitree_mujoco::msg::EpisodeStatus;
  return outcome == EpisodeStatus::SUCCESS || outcome == EpisodeStatus::SELF_COLLISION ||
         outcome == EpisodeStatus::ILLEGAL_CONTACT || outcome == EpisodeStatus::FELL_OVER;
}
}  // namespace

EpisodeManager::EpisodeManager(
    rclcpp::Node::SharedPtr node, const mjModel* model, double episode_timeout_s)
    : node_(std::move(node)),
      episode_timeout_s_(episode_timeout_s)
{
  if (!model)
  {
    throw std::runtime_error("episode manager requires a loaded MuJoCo model");
  }
  if (!std::isfinite(episode_timeout_s_) || episode_timeout_s_ <= 0.0)
  {
    throw std::runtime_error("episode timeout must be finite and positive");
  }

  for (int joint_id = 0; joint_id < model->njnt; ++joint_id)
  {
    if (model->jnt_type[joint_id] == mjJNT_FREE)
    {
      if (base_qpos_address_ >= 0)
      {
        throw std::runtime_error("episode reset requires exactly one free joint");
      }
      base_qpos_address_ = model->jnt_qposadr[joint_id];
      base_dof_address_ = model->jnt_dofadr[joint_id];
    }
  }
  if (base_qpos_address_ < 0)
  {
    throw std::runtime_error("episode reset requires a free base joint");
  }

  status_.outcome = EpisodeStatus::WAITING;
  status_.message = "waiting for reset";

  reset_service_ = node_->create_service<ResetEpisode>(
      "/unitree_mujoco/reset_episode",
      std::bind(&EpisodeManager::HandleReset, this, std::placeholders::_1, std::placeholders::_2));
  start_service_ = node_->create_service<StartEpisode>(
      "/unitree_mujoco/start_episode",
      std::bind(&EpisodeManager::HandleStart, this, std::placeholders::_1, std::placeholders::_2));
  status_publisher_ = node_->create_publisher<EpisodeStatus>(
      "/unitree_mujoco/episode_status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  status_timer_ = node_->create_wall_timer(kStatusPeriod, [this]() { PublishStatus(); });

  RCLCPP_INFO(
      node_->get_logger(), "Episode services ready with %.3f s timeout",
      episode_timeout_s_);
}

EpisodeManager::~EpisodeManager()
{
  Shutdown();
}

void EpisodeManager::Shutdown()
{
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    shutting_down_ = true;
  }
  operation_cv_.notify_all();
}

std::optional<EpisodeManager::ResetRequest> EpisodeManager::TakeResetRequest()
{
  std::lock_guard<std::mutex> lock(operation_mutex_);
  if (!pending_reset_)
  {
    return std::nullopt;
  }
  auto request = pending_reset_;
  pending_reset_.reset();
  return request;
}

bool EpisodeManager::TakeStartRequest()
{
  std::lock_guard<std::mutex> lock(operation_mutex_);
  if (!pending_start_)
  {
    return false;
  }
  pending_start_ = false;
  return true;
}

void EpisodeManager::CompleteReset(
    bool success, std::string message, std::uint64_t episode_id,
    const builtin_interfaces::msg::Time& stamp)
{
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    reset_result_ = OperationResult{success, std::move(message), episode_id, stamp};
  }
  operation_cv_.notify_all();
}

void EpisodeManager::CompleteStart(
    bool success, std::string message, std::uint64_t episode_id)
{
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    start_result_ = OperationResult{
        success, std::move(message), episode_id, builtin_interfaces::msg::Time()};
  }
  operation_cv_.notify_all();
}

std::uint64_t EpisodeManager::Reset(
    const mjData* data, const builtin_interfaces::msg::Time& reset_stamp,
    std::uint64_t expected_depth_generation)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    const std::uint64_t next_episode_id = status_.episode_id + 1;
    status_ = EpisodeStatus{};
    status_.stamp = reset_stamp;
    status_.episode_id = next_episode_id;
    status_.outcome = EpisodeStatus::WAITING;
    status_.message = "waiting for start";
    expected_depth_generation_ = expected_depth_generation;
    CaptureKinematics(data, &status_);
  }
  PublishStatus();
  return episode_id();
}

bool EpisodeManager::Start(
    const mjData* data, const builtin_interfaces::msg::Time& start_stamp, std::string* error)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_.episode_id == 0)
    {
      *error = "reset the simulator before starting an episode";
      return false;
    }
    if (status_.outcome != EpisodeStatus::WAITING)
    {
      *error = "episode is not waiting for start";
      return false;
    }

    status_.stamp = start_stamp;
    status_.outcome = EpisodeStatus::RUNNING;
    status_.elapsed_time_s = 0.0;
    status_.message = "running";
    episode_start_sim_time_ = data->time;
    CaptureKinematics(data, &status_);
  }
  PublishStatus();
  return true;
}

void EpisodeManager::CaptureKinematics(const mjData* data, EpisodeStatus* status) const
{
  const mjtNum* qpos = data->qpos + base_qpos_address_;
  const double w = qpos[3];
  const double x = qpos[4];
  const double y = qpos[5];
  const double z = qpos[6];
  status->roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  status->pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));

  const mjtNum* qvel = data->qvel + base_dof_address_;
  status->linear_velocity_m_s = VectorNorm(qvel);
  status->angular_velocity_rad_s = VectorNorm(qvel + 3);
}

void EpisodeManager::Update(
    const mjData* data, std::optional<TerminalResult> terminal_result)
{
  if (terminal_result && !IsTaskTerminalOutcome(terminal_result->outcome))
  {
    throw std::runtime_error("episode monitor returned a non-task terminal outcome");
  }

  bool terminal = false;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_.outcome == EpisodeStatus::WAITING)
    {
      if (terminal_result)
      {
        throw std::runtime_error("episode monitor returned a result before the episode started");
      }
      CaptureKinematics(data, &status_);
      status_.stamp = node_->now();
      return;
    }
    if (status_.outcome != EpisodeStatus::RUNNING)
    {
      return;
    }

    CaptureKinematics(data, &status_);
    status_.stamp = node_->now();
    status_.elapsed_time_s = std::max(0.0, data->time - episode_start_sim_time_);

    if (terminal_result)
    {
      status_.outcome = terminal_result->outcome;
      status_.message = std::move(terminal_result->message);
    }
    else if (status_.elapsed_time_s >= episode_timeout_s_)
    {
      status_.outcome = EpisodeStatus::TIMEOUT;
      status_.message = "episode timeout";
    }
    terminal = status_.outcome != EpisodeStatus::RUNNING;
  }

  if (terminal)
  {
    PublishStatus();
  }
}

void EpisodeManager::UpdateDepthStatus(
    std::uint64_t published_generation,
    const builtin_interfaces::msg::Time& published_stamp)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  if (!status_.depth_ready && expected_depth_generation_ != 0 &&
      published_generation >= expected_depth_generation_)
  {
    status_.depth_ready = true;
    status_.depth_stamp = published_stamp;
  }
}

void EpisodeManager::PublishStatus()
{
  std::lock_guard<std::mutex> publish_lock(publish_mutex_);
  EpisodeStatus message;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    message = status_;
  }
  status_publisher_->publish(message);
}

std::uint64_t EpisodeManager::episode_id() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_.episode_id;
}

bool EpisodeManager::running() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_.outcome == EpisodeStatus::RUNNING;
}

void EpisodeManager::HandleReset(
    const std::shared_ptr<ResetEpisode::Request> request,
    std::shared_ptr<ResetEpisode::Response> response)
{
  if (!std::isfinite(request->base_x_offset) || !std::isfinite(request->base_y_offset) ||
      !std::isfinite(request->base_yaw_offset))
  {
    response->success = false;
    response->message = "reset offsets must be finite";
    response->episode_id = episode_id();
    return;
  }

  std::unique_lock<std::mutex> lock(operation_mutex_);
  if (pending_reset_ || pending_start_ || reset_result_ || start_result_)
  {
    response->success = false;
    response->message = "another episode operation is pending";
    lock.unlock();
    response->episode_id = episode_id();
    return;
  }
  pending_reset_ = ResetRequest{
      request->base_x_offset, request->base_y_offset, request->base_yaw_offset};
  operation_cv_.wait(lock, [this]() { return reset_result_.has_value() || shutting_down_; });
  if (!reset_result_)
  {
    response->success = false;
    response->message = "simulator stopped before reset completed";
    lock.unlock();
    response->episode_id = episode_id();
    return;
  }

  const auto result = std::move(*reset_result_);
  reset_result_.reset();
  response->success = result.success;
  response->message = result.message;
  response->episode_id = result.episode_id;
  response->reset_stamp = result.stamp;
}

void EpisodeManager::HandleStart(
    const std::shared_ptr<StartEpisode::Request>,
    std::shared_ptr<StartEpisode::Response> response)
{
  std::unique_lock<std::mutex> lock(operation_mutex_);
  if (pending_reset_ || pending_start_ || reset_result_ || start_result_)
  {
    response->success = false;
    response->message = "another episode operation is pending";
    lock.unlock();
    response->episode_id = episode_id();
    return;
  }
  pending_start_ = true;
  operation_cv_.wait(lock, [this]() { return start_result_.has_value() || shutting_down_; });
  if (!start_result_)
  {
    response->success = false;
    response->message = "simulator stopped before episode start completed";
    lock.unlock();
    response->episode_id = episode_id();
    return;
  }

  const auto result = std::move(*start_result_);
  start_result_.reset();
  response->success = result.success;
  response->message = result.message;
  response->episode_id = result.episode_id;
}
