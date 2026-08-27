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

#include "depth_camera.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include <GLFW/glfw3.h>
#include <sensor_msgs/image_encodings.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kCameraVisualGeomGroup = 1;
// D435/D435i nominal depth, width, and height: 25 x 90 x 25 mm.
constexpr std::array<mjtNum, 3> kD435iHalfSizeM{0.0125, 0.045, 0.0125};
constexpr std::array<float, 4> kD435iRgba{0.05f, 0.75f, 0.95f, 1.0f};

std::array<mjtNum, 4> QuaternionFromRpy(const std::array<double, 3>& rpy)
{
  const double half_roll = 0.5 * rpy[0];
  const double half_pitch = 0.5 * rpy[1];
  const double half_yaw = 0.5 * rpy[2];
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);

  return {
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
  };
}

std::array<mjtNum, 4> MultiplyQuaternions(
    const std::array<mjtNum, 4>& lhs, const std::array<mjtNum, 4>& rhs)
{
  std::array<mjtNum, 4> result{};
  mju_mulQuat(result.data(), lhs.data(), rhs.data());
  mju_normalize4(result.data());
  return result;
}

bool IsBigEndian()
{
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 0x01;
}
}  // namespace

DepthCameraPublisher::DepthCameraPublisher(rclcpp::Node::SharedPtr node, param::DepthCameraConfig config)
    : node_(std::move(node)), config_(std::move(config))
{
}

DepthCameraPublisher::~DepthCameraPublisher()
{
  Stop();
}

bool DepthCameraPublisher::ConfigureModel(mjModel* model, std::string* error)
{
  const int camera_id = mj_name2id(model, mjOBJ_CAMERA, config_.camera_name.c_str());
  if (camera_id < 0)
  {
    *error = "MuJoCo model does not contain camera '" + config_.camera_name + "'";
    return false;
  }

  const int parent_id = mj_name2id(model, mjOBJ_BODY, config_.parent_frame.c_str());
  if (parent_id < 0 || model->cam_bodyid[camera_id] != parent_id)
  {
    *error = "camera '" + config_.camera_name + "' must be attached directly to " + config_.parent_frame;
    return false;
  }

  const std::string visual_name = config_.camera_name + "_visual";
  const int visual_id = mj_name2id(model, mjOBJ_GEOM, visual_name.c_str());
  if (visual_id < 0 || model->geom_bodyid[visual_id] != parent_id ||
      model->geom_type[visual_id] != mjGEOM_BOX)
  {
    *error = "camera visual '" + visual_name + "' must be a box geom attached directly to " +
             config_.parent_frame;
    return false;
  }

  const auto mount_quaternion = QuaternionFromRpy(config_.mount.rpy_rad);
  model->geom_sameframe[visual_id] = mjSAMEFRAME_NONE;

  // MuJoCo camera frame: +X right, +Y up, -Z forward.
  // ROS camera_link frame: +X forward, +Y left, +Z up.
  constexpr std::array<mjtNum, 4> camera_link_to_mujoco{0.5, 0.5, -0.5, -0.5};
  const auto mujoco_quaternion = MultiplyQuaternions(mount_quaternion, camera_link_to_mujoco);

  model->cam_mode[camera_id] = mjCAMLIGHT_FIXED;
  model->cam_orthographic[camera_id] = 0;
  for (int i = 0; i < 3; ++i)
  {
    model->cam_pos[3 * camera_id + i] = config_.mount.position_m[i];
  }
  for (int i = 0; i < 4; ++i)
  {
    model->cam_quat[4 * camera_id + i] = mujoco_quaternion[i];
    model->geom_quat[4 * visual_id + i] = mount_quaternion[i];
    model->geom_rgba[4 * visual_id + i] = kD435iRgba[i];
  }
  for (int i = 0; i < 3; ++i)
  {
    model->geom_pos[3 * visual_id + i] = config_.mount.position_m[i];
    model->geom_size[3 * visual_id + i] = kD435iHalfSizeM[i];
  }

  const auto& profile = config_.profile;
  const auto& intrinsics = profile.intrinsics;
  const double horizontal_fov =
      std::atan(intrinsics.cx / intrinsics.fx) +
      std::atan((profile.width - intrinsics.cx) / intrinsics.fx);
  const double vertical_fov =
      std::atan(intrinsics.cy / intrinsics.fy) +
      std::atan((profile.height - intrinsics.cy) / intrinsics.fy);
  model->cam_resolution[2 * camera_id] = profile.width;
  model->cam_resolution[2 * camera_id + 1] = profile.height;
  model->cam_sensorsize[2 * camera_id] = 1.0f;
  model->cam_sensorsize[2 * camera_id + 1] = 1.0f;
  model->cam_intrinsic[4 * camera_id] = static_cast<float>(intrinsics.fx / profile.width);
  model->cam_intrinsic[4 * camera_id + 1] = static_cast<float>(intrinsics.fy / profile.height);
  model->cam_intrinsic[4 * camera_id + 2] = static_cast<float>(0.5 - intrinsics.cx / profile.width);
  // MuJoCo uses a bottom-left image origin. The published image is flipped to
  // ROS's top-left origin, so the vertical principal-point offset changes sign.
  model->cam_intrinsic[4 * camera_id + 3] = static_cast<float>(intrinsics.cy / profile.height - 0.5);
  model->cam_fovy[camera_id] =
      2.0 * std::atan(profile.height / (2.0 * intrinsics.fy)) * 180.0 / kPi;

  camera_id_ = camera_id;
  model_ = model;

  RCLCPP_INFO(
      node_->get_logger(),
      "Configured %s with %s/%s: %dx%d at %.1f Hz, FOV %.2f x %.2f deg, "
      "fx=%.6f fy=%.6f cx=%.6f cy=%.6f",
      config_.camera_name.c_str(),
      config_.model.c_str(),
      config_.profile_name.c_str(),
      profile.width,
      profile.height,
      profile.fps,
      horizontal_fov * 180.0 / kPi,
      vertical_fov * 180.0 / kPi,
      intrinsics.fx,
      intrinsics.fy,
      intrinsics.cx,
      intrinsics.cy);
  return true;
}

bool DepthCameraPublisher::Start(const mjModel* model, std::string* error)
{
  if (model_ != model || camera_id_ < 0)
  {
    *error = "depth camera model configuration was not applied before Start";
    return false;
  }
  if (running_.load() || rendering_thread_.joinable())
  {
    *error = "depth camera publisher is already running";
    return false;
  }

  const float far_distance = static_cast<float>(model_->vis.map.zfar * model_->stat.extent);
  near_distance_ = static_cast<float>(model_->vis.map.znear * model_->stat.extent);
  if (!(near_distance_ > 0.0f && far_distance > near_distance_))
  {
    *error = "MuJoCo model has invalid rendering near/far planes";
    return false;
  }
  depth_linearization_scale_ = 1.0f - near_distance_ / far_distance;

  pending_data_ = mj_makeData(model_);
  render_data_ = mj_makeData(model_);
  if (!pending_data_ || !render_data_)
  {
    *error = "failed to allocate depth camera state snapshots";
    Stop();
    return false;
  }

  InitializeMessages();

  auto depth_qos = rclcpp::SensorDataQoS();
  depth_qos.keep_last(1);
  const auto info_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  depth_publisher_ =
      node_->create_publisher<sensor_msgs::msg::Image>(config_.ros.depth_topic, depth_qos);
  camera_info_publisher_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>(config_.ros.camera_info_topic, info_qos);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    stop_requested_ = false;
    frame_pending_ = false;
    initialization_complete_ = false;
    initialization_success_ = false;
    initialization_error_.clear();
    generation_ = 0;
    pending_generation_ = 0;
    published_generation_ = 0;
    published_stamp_ = builtin_interfaces::msg::Time();
  }
  capture_schedule_initialized_ = false;
  rendering_thread_ = std::thread(&DepthCameraPublisher::RenderingLoop, this);

  std::unique_lock<std::mutex> lock(data_mutex_);
  initialization_cv_.wait(lock, [this] { return initialization_complete_; });
  if (!initialization_success_)
  {
    *error = initialization_error_;
    lock.unlock();
    Stop();
    return false;
  }
  lock.unlock();

  PublishStaticTransforms();
  RCLCPP_INFO(
      node_->get_logger(),
      "Publishing RealSense-compatible depth on %s and %s",
      config_.ros.depth_topic.c_str(),
      config_.ros.camera_info_topic.c_str());
  return true;
}

void DepthCameraPublisher::InitializeMessages()
{
  const auto& profile = config_.profile;
  const auto pixel_count = static_cast<std::size_t>(profile.width) * profile.height;

  raw_depth_.resize(pixel_count);
  depth_units_.resize(pixel_count);

  depth_message_.header.frame_id = config_.ros.optical_frame_id;
  depth_message_.height = profile.height;
  depth_message_.width = profile.width;
  depth_message_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_message_.is_bigendian = IsBigEndian();
  depth_message_.step = profile.width * sizeof(std::uint16_t);
  depth_message_.data.resize(pixel_count * sizeof(std::uint16_t));

  const auto& intrinsics = profile.intrinsics;
  camera_info_message_.header.frame_id = config_.ros.optical_frame_id;
  camera_info_message_.height = profile.height;
  camera_info_message_.width = profile.width;
  camera_info_message_.distortion_model = intrinsics.distortion_model;
  camera_info_message_.d.assign(intrinsics.distortion.begin(), intrinsics.distortion.end());
  camera_info_message_.k.fill(0.0);
  camera_info_message_.r.fill(0.0);
  camera_info_message_.p.fill(0.0);
  camera_info_message_.k[0] = intrinsics.fx;
  camera_info_message_.k[2] = intrinsics.cx;
  camera_info_message_.k[4] = intrinsics.fy;
  camera_info_message_.k[5] = intrinsics.cy;
  camera_info_message_.k[8] = 1.0;
  camera_info_message_.r[0] = 1.0;
  camera_info_message_.r[4] = 1.0;
  camera_info_message_.r[8] = 1.0;
  camera_info_message_.p[0] = intrinsics.fx;
  camera_info_message_.p[2] = intrinsics.cx;
  camera_info_message_.p[5] = intrinsics.fy;
  camera_info_message_.p[6] = intrinsics.cy;
  camera_info_message_.p[10] = 1.0;

  render_camera_.type = mjCAMERA_FIXED;
  render_camera_.fixedcamid = camera_id_;
  viewport_ = {0, 0, profile.width, profile.height};
}

void DepthCameraPublisher::PublishStaticTransforms()
{
  const auto stamp = node_->now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms(3);

  const auto mount_quaternion = QuaternionFromRpy(config_.mount.rpy_rad);
  auto& mount = transforms[0];
  mount.header.stamp = stamp;
  mount.header.frame_id = config_.parent_frame;
  mount.child_frame_id = config_.ros.camera_link_frame_id;
  mount.transform.translation.x = config_.mount.position_m[0];
  mount.transform.translation.y = config_.mount.position_m[1];
  mount.transform.translation.z = config_.mount.position_m[2];
  mount.transform.rotation.w = mount_quaternion[0];
  mount.transform.rotation.x = mount_quaternion[1];
  mount.transform.rotation.y = mount_quaternion[2];
  mount.transform.rotation.z = mount_quaternion[3];

  auto& depth = transforms[1];
  depth.header.stamp = stamp;
  depth.header.frame_id = config_.ros.camera_link_frame_id;
  depth.child_frame_id = config_.ros.depth_frame_id;
  depth.transform.rotation.w = 1.0;

  auto& optical = transforms[2];
  optical.header.stamp = stamp;
  optical.header.frame_id = config_.ros.depth_frame_id;
  optical.child_frame_id = config_.ros.optical_frame_id;
  optical.transform.rotation.w = 0.5;
  optical.transform.rotation.x = -0.5;
  optical.transform.rotation.y = 0.5;
  optical.transform.rotation.z = -0.5;

  static_tf_broadcaster_->sendTransform(transforms);
}

void DepthCameraPublisher::CaptureIfDue(const mjData* data)
{
  if (!running_.load())
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / config_.profile.fps));
  if (capture_schedule_initialized_)
  {
    const auto elapsed = now - last_capture_wall_time_;
    if (elapsed < period)
    {
      return;
    }
    last_capture_wall_time_ += (elapsed / period) * period;
  }
  else
  {
    capture_schedule_initialized_ = true;
    last_capture_wall_time_ = now;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (stop_requested_)
    {
      return;
    }
    mjv_copyData(pending_data_, model_, data);
    pending_stamp_ = node_->now();
    pending_generation_ = generation_;
    frame_pending_ = true;
  }
  data_cv_.notify_one();
}

std::uint64_t DepthCameraPublisher::ResetAndCapture(
    const mjData* data, const builtin_interfaces::msg::Time& stamp)
{
  if (!running_.load())
  {
    return 0;
  }

  std::uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (stop_requested_)
    {
      return 0;
    }
    generation_ += 1;
    generation = generation_;
    mjv_copyData(pending_data_, model_, data);
    pending_stamp_ = stamp;
    pending_generation_ = generation;
    frame_pending_ = true;
    capture_schedule_initialized_ = true;
    last_capture_wall_time_ = std::chrono::steady_clock::now();
  }
  data_cv_.notify_one();
  return generation;
}

DepthCameraPublisher::FrameStatus DepthCameraPublisher::GetFrameStatus() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return FrameStatus{published_generation_, published_stamp_};
}

void DepthCameraPublisher::CompleteRendererInitialization(bool success, std::string error)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    initialization_success_ = success;
    initialization_error_ = std::move(error);
    initialization_complete_ = true;
    running_.store(success);
  }
  initialization_cv_.notify_one();
}

void DepthCameraPublisher::RenderingLoop()
{
  if (!glfwInit())
  {
    CompleteRendererInitialization(false, "GLFW initialization failed for depth camera rendering");
    return;
  }

  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(1, 1, "unitree_mujoco_depth_camera", nullptr, nullptr);
  glfwDefaultWindowHints();
  if (!window)
  {
    const char* description = nullptr;
    const int code = glfwGetError(&description);
    CompleteRendererInitialization(
        false,
        "failed to create hidden GLFW camera context (" + std::to_string(code) + "): " +
            (description ? description : "unknown error"));
    return;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);

  mjv_defaultOption(&render_option_);
  mjv_defaultScene(&render_scene_);
  mjr_defaultContext(&render_context_);

  render_option_.flags[mjVIS_RANGEFINDER] = 0;
  render_option_.geomgroup[kCameraVisualGeomGroup] = 0;
  if (config_.exclude_self)
  {
    render_option_.geomgroup[2] = 0;
  }
  render_option_.geomgroup[3] = 0;
  for (int i = 0; i < mjNGROUP; ++i)
  {
    render_option_.sitegroup[i] = 0;
  }

  mjv_makeScene(model_, &render_scene_, 2000);
  mjr_makeContext(model_, &render_context_, mjFONTSCALE_150);
  render_context_.offSamples = 1;
  mjr_resizeOffscreen(config_.profile.width, config_.profile.height, &render_context_);
  render_context_.readDepthMap = mjDEPTH_ZERONEAR;
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);

  render_scene_.flags[mjRND_SHADOW] = 0;
  render_scene_.flags[mjRND_REFLECTION] = 0;
  render_scene_.flags[mjRND_SKYBOX] = 0;
  render_scene_.flags[mjRND_FOG] = 0;
  render_scene_.flags[mjRND_HAZE] = 0;

  if (!render_context_.glInitialized || render_context_.currentBuffer != mjFB_OFFSCREEN ||
      render_context_.offWidth < config_.profile.width || render_context_.offHeight < config_.profile.height)
  {
    mjr_freeContext(&render_context_);
    mjv_freeScene(&render_scene_);
    glfwDestroyWindow(window);
    CompleteRendererInitialization(false, "MuJoCo failed to initialize the requested offscreen framebuffer");
    return;
  }

  CompleteRendererInitialization(true);

  while (true)
  {
    builtin_interfaces::msg::Time stamp;
    std::uint64_t generation = 0;
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      data_cv_.wait(lock, [this] { return frame_pending_ || stop_requested_; });
      if (stop_requested_)
      {
        break;
      }
      std::swap(pending_data_, render_data_);
      stamp = pending_stamp_;
      generation = pending_generation_;
      frame_pending_ = false;
    }

    RenderAndPublish(render_data_, stamp, generation);
  }

  running_.store(false);
  mjr_freeContext(&render_context_);
  mjv_freeScene(&render_scene_);
  glfwDestroyWindow(window);
}

void DepthCameraPublisher::RenderAndPublish(
    mjData* data, const builtin_interfaces::msg::Time& stamp, std::uint64_t generation)
{
  mjv_updateScene(
      model_, data, &render_option_, nullptr, &render_camera_, mjCAT_ALL, &render_scene_);
  mjr_render(viewport_, &render_scene_, &render_context_);
  mjr_readPixels(nullptr, raw_depth_.data(), viewport_, &render_context_);

  const double max_units = std::numeric_limits<std::uint16_t>::max();
  for (int source_y = 0; source_y < config_.profile.height; ++source_y)
  {
    const float* source =
        raw_depth_.data() + static_cast<std::size_t>(source_y) * config_.profile.width;
    std::uint16_t* destination =
        depth_units_.data() +
        static_cast<std::size_t>(config_.profile.height - 1 - source_y) * config_.profile.width;
    for (int x = 0; x < config_.profile.width; ++x)
    {
      if (!std::isfinite(source[x]) || source[x] >= 1.0f - 1.0e-6f)
      {
        destination[x] = 0;
        continue;
      }

      const double depth_m = near_distance_ / (1.0 - source[x] * depth_linearization_scale_);
      const double units = depth_m / config_.profile.depth_scale_m;
      if (!std::isfinite(units) || units < 1.0 || units > max_units)
      {
        destination[x] = 0;
      }
      else
      {
        destination[x] = static_cast<std::uint16_t>(std::llround(units));
      }
    }
  }

  std::memcpy(
      depth_message_.data.data(), depth_units_.data(), depth_units_.size() * sizeof(std::uint16_t));
  {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (generation != generation_ || stop_requested_)
      {
        return;
      }
    }
    depth_message_.header.stamp = stamp;
    camera_info_message_.header.stamp = stamp;
    depth_publisher_->publish(depth_message_);
    camera_info_publisher_->publish(camera_info_message_);

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (generation != generation_ || stop_requested_)
    {
      return;
    }
    published_generation_ = generation;
    published_stamp_ = stamp;
  }
}

void DepthCameraPublisher::Stop()
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    stop_requested_ = true;
    frame_pending_ = false;
  }
  data_cv_.notify_one();
  initialization_cv_.notify_one();

  if (rendering_thread_.joinable())
  {
    rendering_thread_.join();
  }

  running_.store(false);
  static_tf_broadcaster_.reset();
  camera_info_publisher_.reset();
  depth_publisher_.reset();
  if (pending_data_)
  {
    mj_deleteData(pending_data_);
    pending_data_ = nullptr;
  }
  if (render_data_)
  {
    mj_deleteData(render_data_);
    render_data_ = nullptr;
  }
}
