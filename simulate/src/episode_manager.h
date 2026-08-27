#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <unitree_mujoco/msg/episode_status.hpp>
#include <unitree_mujoco/srv/reset_episode.hpp>
#include <unitree_mujoco/srv/start_episode.hpp>

#include "episode_result.h"

class EpisodeManager
{
public:
  struct ResetRequest
  {
    double base_x_offset = 0.0;
    double base_y_offset = 0.0;
    double base_yaw_offset = 0.0;
  };

  EpisodeManager(
      rclcpp::Node::SharedPtr node, const mjModel* model, double episode_timeout_s);
  ~EpisodeManager();

  EpisodeManager(const EpisodeManager&) = delete;
  EpisodeManager& operator=(const EpisodeManager&) = delete;

  void Shutdown();

  std::optional<ResetRequest> TakeResetRequest();
  bool TakeStartRequest();
  void CompleteReset(
      bool success, std::string message, std::uint64_t episode_id,
      const builtin_interfaces::msg::Time& stamp);
  void CompleteStart(
      bool success, std::string message, std::uint64_t episode_id);

  int base_qpos_address() const { return base_qpos_address_; }
  std::uint64_t Reset(
      const mjData* data, const builtin_interfaces::msg::Time& reset_stamp,
      std::uint64_t expected_depth_generation);
  bool Start(
      const mjData* data, const builtin_interfaces::msg::Time& start_stamp,
      std::string* error);
  void Update(const mjData* data, std::optional<TerminalResult> terminal_result);
  void UpdateDepthStatus(
      std::uint64_t published_generation,
      const builtin_interfaces::msg::Time& published_stamp);
  std::uint64_t episode_id() const;
  bool running() const;

private:
  using EpisodeStatus = unitree_mujoco::msg::EpisodeStatus;
  using ResetEpisode = unitree_mujoco::srv::ResetEpisode;
  using StartEpisode = unitree_mujoco::srv::StartEpisode;

  struct OperationResult
  {
    bool success = false;
    std::string message;
    std::uint64_t episode_id = 0;
    builtin_interfaces::msg::Time stamp;
  };

  void CaptureKinematics(const mjData* data, EpisodeStatus* status) const;
  void PublishStatus();

  void HandleReset(
      const std::shared_ptr<ResetEpisode::Request> request,
      std::shared_ptr<ResetEpisode::Response> response);
  void HandleStart(
      const std::shared_ptr<StartEpisode::Request> request,
      std::shared_ptr<StartEpisode::Response> response);

  rclcpp::Node::SharedPtr node_;
  double episode_timeout_s_ = 20.0;
  int base_qpos_address_ = -1;
  int base_dof_address_ = -1;

  mutable std::mutex operation_mutex_;
  std::condition_variable operation_cv_;
  std::optional<ResetRequest> pending_reset_;
  bool pending_start_ = false;
  std::optional<OperationResult> reset_result_;
  std::optional<OperationResult> start_result_;
  bool shutting_down_ = false;

  std::mutex publish_mutex_;
  mutable std::mutex status_mutex_;
  EpisodeStatus status_;
  double episode_start_sim_time_ = 0.0;
  std::uint64_t expected_depth_generation_ = 0;

  rclcpp::Service<ResetEpisode>::SharedPtr reset_service_;
  rclcpp::Service<StartEpisode>::SharedPtr start_service_;
  rclcpp::Publisher<EpisodeStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
