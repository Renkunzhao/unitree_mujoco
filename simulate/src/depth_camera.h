/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Adapted from mujoco_ros2_control's camera plugin for unitree_mujoco.
 */

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "param.h"

class DepthCameraPublisher
{
public:
  struct FrameStatus
  {
    std::uint64_t generation = 0;
    builtin_interfaces::msg::Time stamp;
  };

  DepthCameraPublisher(rclcpp::Node::SharedPtr node, param::DepthCameraConfig config);
  ~DepthCameraPublisher();

  DepthCameraPublisher(const DepthCameraPublisher&) = delete;
  DepthCameraPublisher& operator=(const DepthCameraPublisher&) = delete;

  bool ConfigureModel(mjModel* model, std::string* error);
  bool Start(const mjModel* model, std::string* error);

  // The caller must hold the simulator mutex while copying the current state.
  void CaptureIfDue(const mjData* data);
  std::uint64_t ResetAndCapture(
      const mjData* data, const builtin_interfaces::msg::Time& stamp);
  FrameStatus GetFrameStatus() const;
  void Stop();

private:
  void InitializeMessages();
  void PublishStaticTransforms();
  void RenderingLoop();
  void RenderAndPublish(
      mjData* data, const builtin_interfaces::msg::Time& stamp, std::uint64_t generation);
  void CompleteRendererInitialization(bool success, std::string error = {});

  rclcpp::Node::SharedPtr node_;
  param::DepthCameraConfig config_;

  const mjModel* model_ = nullptr;
  int camera_id_ = -1;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  sensor_msgs::msg::Image depth_message_;
  sensor_msgs::msg::CameraInfo camera_info_message_;

  mjData* pending_data_ = nullptr;
  mjData* render_data_ = nullptr;
  mjvCamera render_camera_{};
  mjvOption render_option_{};
  mjvScene render_scene_{};
  mjrContext render_context_{};
  mjrRect viewport_{};

  std::vector<float> raw_depth_;
  std::vector<std::uint16_t> depth_units_;

  float near_distance_ = 0.0f;
  float depth_linearization_scale_ = 0.0f;

  std::thread rendering_thread_;
  std::atomic_bool running_{false};
  std::mutex publish_mutex_;
  mutable std::mutex data_mutex_;
  std::condition_variable data_cv_;
  std::condition_variable initialization_cv_;
  bool stop_requested_ = false;
  bool frame_pending_ = false;
  bool initialization_complete_ = false;
  bool initialization_success_ = false;
  std::string initialization_error_;
  builtin_interfaces::msg::Time pending_stamp_;
  std::uint64_t generation_ = 0;
  std::uint64_t pending_generation_ = 0;
  std::uint64_t published_generation_ = 0;
  builtin_interfaces::msg::Time published_stamp_;

  bool capture_schedule_initialized_ = false;
  std::chrono::steady_clock::time_point last_capture_wall_time_;
};
