#include "ros2_bridge.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <unitree_go/msg/detail/low_state__struct.h>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_hg/msg/bms_state.hpp>
#include <unitree_hg/msg/detail/low_state__struct.h>
#include <unitree_hg/msg/imu_state.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "param.h"

namespace
{
constexpr int kGoMotorCount = 20;
constexpr int kG1MotorCount = 35;
constexpr auto kStatePublishPeriod = std::chrono::milliseconds(2);

const rclcpp::QoS& StateQos()
{
  static const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  return qos;
}

const rclcpp::QoS& CommandQos()
{
  static const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  return qos;
}

std::uint32_t ProtocolCrc32(const std::uint32_t* words, std::size_t count)
{
  std::uint32_t crc = 0xFFFFFFFF;
  constexpr std::uint32_t polynomial = 0x04C11DB7;
  for (std::size_t i = 0; i < count; ++i)
  {
    std::uint32_t bit = 1U << 31;
    const std::uint32_t data = words[i];
    for (int j = 0; j < 32; ++j)
    {
      if (crc & 0x80000000U)
      {
        crc = (crc << 1) ^ polynomial;
      }
      else
      {
        crc <<= 1;
      }
      if (data & bit)
      {
        crc ^= polynomial;
      }
      bit >>= 1;
    }
  }
  return crc;
}

template <typename Message>
std::uint32_t MessageCrc(const Message& message)
{
  static_assert(std::is_trivially_copyable_v<Message>);
  static_assert(sizeof(Message) % sizeof(std::uint32_t) == 0);
  std::array<std::uint32_t, sizeof(Message) / sizeof(std::uint32_t)> words{};
  std::memcpy(words.data(), &message, sizeof(message));
  return ProtocolCrc32(words.data(), words.size() - 1);
}

template <typename Command>
void ApplyMotorCommand(
    const Command& command, int motor_count, int sensor_stride, const mjData* data, mjtNum* control)
{
  for (int i = 0; i < motor_count; ++i)
  {
    const auto& motor = command.motor_cmd[i];
    control[i] = motor.tau + motor.kp * (motor.q - data->sensordata[i]) +
                 motor.kd * (motor.dq - data->sensordata[i + sensor_stride]);
  }
}

template <typename State>
void AssignImu(const Ros2BridgeBase::ImuSample& sample, State* state)
{
  state->quaternion = sample.quaternion;
  state->gyroscope = sample.gyroscope;
  state->accelerometer = sample.accelerometer;
  state->rpy = sample.rpy;
}

class Go2Bridge final : public Ros2BridgeBase
{
public:
  Go2Bridge(rclcpp::Node::SharedPtr node, mjModel* model, mjData* data)
      : Ros2BridgeBase(std::move(node), model, data)
  {
    if (num_motor_ > kGoMotorCount)
    {
      throw std::runtime_error("Go/B2 ROS message supports at most 20 motors");
    }

    low_state_publisher_ =
        node_->create_publisher<unitree_go::msg::LowState>("/lowstate", StateQos());
    low_command_subscription_ = node_->create_subscription<unitree_go::msg::LowCmd>(
        "/lowcmd", CommandQos(),
        [this](unitree_go::msg::LowCmd::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          low_command_ = *message;
          has_command_ = true;
        });

    foot_force_address_ = SensorAddress("FR_touch");
    foot_position_address_ = SensorAddress("FR_pos");
    foot_velocity_address_ = SensorAddress("FR_linvel");
    const int foot_geom = mj_name2id(model_, mjOBJ_GEOM, "FL");
    if (foot_geom >= 0)
    {
      foot_size_ = model_->geom_size[3 * foot_geom];
    }
  }

  void ApplyCommand() override
  {
    unitree_go::msg::LowCmd command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!has_command_)
      {
        mju_zero(data_->ctrl, num_motor_);
        return;
      }
      command = low_command_;
    }
    ApplyMotorCommand(command, num_motor_, num_motor_, data_, data_->ctrl);
  }

  void CaptureState() override
  {
    Snapshot next;
    next.common = CaptureCommonState();
    for (int i = 0; i < num_motor_; ++i)
    {
      next.low_state.motor_state[i].q = data_->sensordata[i];
      next.low_state.motor_state[i].dq = data_->sensordata[i + num_motor_];
      next.low_state.motor_state[i].tau_est = data_->sensordata[i + 2 * num_motor_];
    }

    const auto imu = ReadImu(
        imu_quaternion_address_, imu_gyroscope_address_, imu_accelerometer_address_);
    AssignImu(imu, &next.low_state.imu_state);
    next.low_state.tick = static_cast<std::uint32_t>(std::llround(data_->time / 1.0e-3));
    next.low_state.wireless_remote = next.common.wireless_remote;

    if (foot_force_address_ >= 0)
    {
      for (int i = 0; i < 4; ++i)
      {
        const auto force = std::clamp(
            std::llround(data_->sensordata[foot_force_address_ + i]),
            static_cast<long long>(std::numeric_limits<std::int16_t>::min()),
            static_cast<long long>(std::numeric_limits<std::int16_t>::max()));
        next.low_state.foot_force[i] = static_cast<std::int16_t>(force);
      }
    }

    if (foot_position_address_ >= 0)
    {
      for (int i = 0; i < 12; ++i)
      {
        next.common.sport_mode_state.foot_position_body[i] =
            data_->sensordata[foot_position_address_ + i];
      }
      for (int i = 0; i < 4; ++i)
      {
        next.common.sport_mode_state.foot_position_body[3 * i + 2] -= foot_size_;
      }
    }
    if (foot_velocity_address_ >= 0)
    {
      for (int i = 0; i < 12; ++i)
      {
        next.common.sport_mode_state.foot_speed_body[i] =
            data_->sensordata[foot_velocity_address_ + i];
      }
    }

    std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
    if (lock.owns_lock())
    {
      snapshot_ = std::move(next);
      has_snapshot_ = true;
    }
  }

  void ClearForReset() override
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    low_command_ = unitree_go::msg::LowCmd();
    has_command_ = false;
  }

private:
  struct Snapshot
  {
    unitree_go::msg::LowState low_state;
    CommonStateSnapshot common;
  };

  void PublishSnapshot() override
  {
    Snapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (!has_snapshot_)
      {
        return;
      }
      snapshot = snapshot_;
    }

    snapshot.low_state.crc = MessageCrc(snapshot.low_state);
    low_state_publisher_->publish(snapshot.low_state);
    PublishCommonState(snapshot.common);
  }

  static_assert(sizeof(unitree_go::msg::LowState) == sizeof(unitree_go__msg__LowState));
  static_assert(offsetof(unitree_go::msg::LowState, crc) ==
                offsetof(unitree_go__msg__LowState, crc));

  std::mutex command_mutex_;
  unitree_go::msg::LowCmd low_command_;
  bool has_command_ = false;
  std::mutex snapshot_mutex_;
  Snapshot snapshot_;
  bool has_snapshot_ = false;
  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr low_state_publisher_;
  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr low_command_subscription_;
  int foot_force_address_ = -1;
  int foot_position_address_ = -1;
  int foot_velocity_address_ = -1;
  double foot_size_ = 0.0;
};

class G1Bridge final : public Ros2BridgeBase
{
public:
  G1Bridge(rclcpp::Node::SharedPtr node, mjModel* model, mjData* data)
      : Ros2BridgeBase(std::move(node), model, data)
  {
    if (num_motor_ > kG1MotorCount)
    {
      throw std::runtime_error("G1 ROS message supports at most 35 motors");
    }

    mode_machine_ =
        param::config.robot_scene.filename().string().find("23") != std::string::npos ? 4 : 5;
    low_state_publisher_ =
        node_->create_publisher<unitree_hg::msg::LowState>("/lowstate", StateQos());
    low_command_subscription_ = node_->create_subscription<unitree_hg::msg::LowCmd>(
        "/lowcmd", CommandQos(),
        [this](unitree_hg::msg::LowCmd::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          low_command_ = *message;
          has_command_ = true;
        });
    bms_state_publisher_ =
        node_->create_publisher<unitree_hg::msg::BmsState>("/lf/bmsstate", StateQos());
    secondary_imu_publisher_ =
        node_->create_publisher<unitree_hg::msg::IMUState>("/secondary_imu", StateQos());
  }

  void ApplyCommand() override
  {
    unitree_hg::msg::LowCmd command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!has_command_)
      {
        mju_zero(data_->ctrl, num_motor_);
        return;
      }
      command = low_command_;
    }
    ApplyMotorCommand(command, num_motor_, num_motor_, data_, data_->ctrl);
  }

  void CaptureState() override
  {
    Snapshot next;
    next.common = CaptureCommonState();
    next.low_state.mode_machine = mode_machine_;
    next.bms_state.soc = 100;
    for (int i = 0; i < num_motor_; ++i)
    {
      next.low_state.motor_state[i].q = data_->sensordata[i];
      next.low_state.motor_state[i].dq = data_->sensordata[i + num_motor_];
      next.low_state.motor_state[i].tau_est = data_->sensordata[i + 2 * num_motor_];
    }

    const auto imu = ReadImu(
        imu_quaternion_address_, imu_gyroscope_address_, imu_accelerometer_address_);
    AssignImu(imu, &next.low_state.imu_state);
    next.low_state.tick = static_cast<std::uint32_t>(std::llround(data_->time / 1.0e-3));
    next.low_state.wireless_remote = next.common.wireless_remote;

    const auto secondary_imu = ReadImu(
        secondary_imu_quaternion_address_, secondary_imu_gyroscope_address_,
        secondary_imu_accelerometer_address_);
    AssignImu(secondary_imu, &next.secondary_imu_state);

    std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
    if (lock.owns_lock())
    {
      snapshot_ = std::move(next);
      has_snapshot_ = true;
    }
  }

  void ClearForReset() override
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    low_command_ = unitree_hg::msg::LowCmd();
    has_command_ = false;
  }

private:
  struct Snapshot
  {
    unitree_hg::msg::LowState low_state;
    unitree_hg::msg::BmsState bms_state;
    unitree_hg::msg::IMUState secondary_imu_state;
    CommonStateSnapshot common;
  };

  void PublishSnapshot() override
  {
    Snapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (!has_snapshot_)
      {
        return;
      }
      snapshot = snapshot_;
    }

    snapshot.low_state.crc = MessageCrc(snapshot.low_state);
    low_state_publisher_->publish(snapshot.low_state);
    secondary_imu_publisher_->publish(snapshot.secondary_imu_state);
    bms_state_publisher_->publish(snapshot.bms_state);
    PublishCommonState(snapshot.common);
  }

  static_assert(sizeof(unitree_hg::msg::LowState) == sizeof(unitree_hg__msg__LowState));
  static_assert(offsetof(unitree_hg::msg::LowState, crc) ==
                offsetof(unitree_hg__msg__LowState, crc));

  std::mutex command_mutex_;
  unitree_hg::msg::LowCmd low_command_;
  bool has_command_ = false;
  std::mutex snapshot_mutex_;
  Snapshot snapshot_;
  bool has_snapshot_ = false;
  std::uint8_t mode_machine_ = 0;
  rclcpp::Publisher<unitree_hg::msg::LowState>::SharedPtr low_state_publisher_;
  rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr low_command_subscription_;
  rclcpp::Publisher<unitree_hg::msg::BmsState>::SharedPtr bms_state_publisher_;
  rclcpp::Publisher<unitree_hg::msg::IMUState>::SharedPtr secondary_imu_publisher_;
};
}  // namespace

Ros2BridgeBase::Ros2BridgeBase(rclcpp::Node::SharedPtr node, mjModel* model, mjData* data)
    : node_(std::move(node)), model_(model), data_(data), num_motor_(model->nu)
{
  if (model_->nsensordata < 3 * num_motor_)
  {
    throw std::runtime_error("MuJoCo model does not expose q, dq, and torque sensors for every motor");
  }

  imu_quaternion_address_ = SensorAddress("imu_quat");
  imu_gyroscope_address_ = SensorAddress("imu_gyro");
  imu_accelerometer_address_ = SensorAddress("imu_acc");
  frame_position_address_ = SensorAddress("frame_pos");
  frame_velocity_address_ = SensorAddress("frame_vel");
  secondary_imu_quaternion_address_ = SensorAddress("secondary_imu_quat");
  secondary_imu_gyroscope_address_ = SensorAddress("secondary_imu_gyro");
  secondary_imu_accelerometer_address_ = SensorAddress("secondary_imu_acc");

  if (param::config.print_scene_information == 1)
  {
    PrintSceneInformation();
  }

  if (param::config.use_joystick == 1)
  {
    if (param::config.joystick_type == "xbox")
    {
      joystick_ = std::make_unique<XBoxJoystick>(
          param::config.joystick_device, param::config.joystick_bits);
    }
    else if (param::config.joystick_type == "switch")
    {
      joystick_ = std::make_unique<SwitchJoystick>(
          param::config.joystick_device, param::config.joystick_bits);
    }
    else
    {
      throw std::runtime_error("unsupported joystick type: " + param::config.joystick_type);
    }
  }

  sport_mode_state_publisher_ =
      node_->create_publisher<unitree_go::msg::SportModeState>("/sportmodestate", StateQos());
  if (joystick_)
  {
    wireless_controller_publisher_ = node_->create_publisher<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", StateQos());
  }
}

void Ros2BridgeBase::StartStatePublisher()
{
  if (state_publish_timer_)
  {
    throw std::runtime_error("ROS 2 state publisher is already running");
  }
  state_publish_timer_ =
      node_->create_wall_timer(kStatePublishPeriod, [this]() { PublishSnapshot(); });
}

int Ros2BridgeBase::SensorAddress(const char* name) const
{
  const int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, name);
  return sensor_id >= 0 ? model_->sensor_adr[sensor_id] : -1;
}

Ros2BridgeBase::ImuSample Ros2BridgeBase::ReadImu(
    int quaternion_address, int gyroscope_address, int accelerometer_address) const
{
  ImuSample sample;
  if (quaternion_address >= 0)
  {
    for (int i = 0; i < 4; ++i)
    {
      sample.quaternion[i] = data_->sensordata[quaternion_address + i];
    }

    const double w = sample.quaternion[0];
    const double x = sample.quaternion[1];
    const double y = sample.quaternion[2];
    const double z = sample.quaternion[3];
    sample.rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    sample.rpy[1] = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    sample.rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }
  if (gyroscope_address >= 0)
  {
    for (int i = 0; i < 3; ++i)
    {
      sample.gyroscope[i] = data_->sensordata[gyroscope_address + i];
    }
  }
  if (accelerometer_address >= 0)
  {
    for (int i = 0; i < 3; ++i)
    {
      sample.accelerometer[i] = data_->sensordata[accelerometer_address + i];
    }
  }
  return sample;
}

Ros2BridgeBase::CommonStateSnapshot Ros2BridgeBase::CaptureCommonState()
{
  CommonStateSnapshot snapshot;
  const auto seconds = static_cast<std::int32_t>(data_->time);
  snapshot.sport_mode_state.stamp.sec = seconds;
  snapshot.sport_mode_state.stamp.nanosec =
      static_cast<std::uint32_t>((data_->time - seconds) * 1.0e9);
  if (frame_position_address_ >= 0)
  {
    for (int i = 0; i < 3; ++i)
    {
      snapshot.sport_mode_state.position[i] = data_->sensordata[frame_position_address_ + i];
    }
  }
  if (frame_velocity_address_ >= 0)
  {
    for (int i = 0; i < 3; ++i)
    {
      snapshot.sport_mode_state.velocity[i] = data_->sensordata[frame_velocity_address_ + i];
    }
  }

  if (!joystick_)
  {
    return snapshot;
  }
  joystick_->Update();
  const auto& state = joystick_->State();
  snapshot.wireless_remote = state.RemoteData();
  snapshot.wireless_controller.lx = state.lx;
  snapshot.wireless_controller.ly = state.ly;
  snapshot.wireless_controller.rx = state.rx;
  snapshot.wireless_controller.ry = state.ry;
  snapshot.wireless_controller.keys = state.Buttons();
  return snapshot;
}

void Ros2BridgeBase::PublishCommonState(const CommonStateSnapshot& snapshot)
{
  sport_mode_state_publisher_->publish(snapshot.sport_mode_state);

  if (joystick_)
  {
    wireless_controller_publisher_->publish(snapshot.wireless_controller);
  }
}

void Ros2BridgeBase::PrintSceneInformation() const
{
  const auto print_objects = [this](const char* title, int count, int type, auto get_index) {
    std::cout << "<<------------- " << title << " ------------->>\n";
    for (int i = 0; i < count; ++i)
    {
      const char* name = mj_id2name(model_, type, i);
      if (!name)
      {
        continue;
      }
      std::cout << title << "_index: " << get_index(i) << ", name: " << name;
      if (type == mjOBJ_JOINT)
      {
        std::cout << ", jointrange: [" << model_->jnt_range[2 * i] << ", "
                  << model_->jnt_range[2 * i + 1] << "]";
      }
      if (type == mjOBJ_ACTUATOR)
      {
        std::cout << ", ctrlrange: [" << model_->actuator_ctrlrange[2 * i] << ", "
                  << model_->actuator_ctrlrange[2 * i + 1] << "]";
      }
      if (type == mjOBJ_SENSOR)
      {
        std::cout << ", dim: " << model_->sensor_dim[i];
      }
      std::cout << '\n';
    }
    std::cout << '\n';
  };

  print_objects("Link", model_->nbody, mjOBJ_BODY, [](int i) { return i; });
  print_objects("Joint", model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
  print_objects("Actuator", model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
  int sensor_index = 0;
  print_objects("Sensor", model_->nsensor, mjOBJ_SENSOR, [this, &sensor_index](int i) {
    const int current = sensor_index;
    sensor_index += model_->sensor_dim[i];
    return current;
  });
}

std::unique_ptr<Ros2BridgeBase> CreateRos2Bridge(
    const rclcpp::Node::SharedPtr& node, mjModel* model, mjData* data)
{
  std::unique_ptr<Ros2BridgeBase> bridge;
  if (model->nu > kGoMotorCount)
  {
    RCLCPP_INFO(node->get_logger(), "Using unitree_hg ROS 2 bridge for %d motors", model->nu);
    bridge = std::make_unique<G1Bridge>(node, model, data);
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Using unitree_go ROS 2 bridge for %d motors", model->nu);
    bridge = std::make_unique<Go2Bridge>(node, model, data);
  }
  bridge->StartStatePublisher();
  return bridge;
}
