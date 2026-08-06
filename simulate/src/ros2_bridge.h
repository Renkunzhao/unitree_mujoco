#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include "physics_joystick.h"

class Ros2BridgeBase
{
public:
  Ros2BridgeBase(rclcpp::Node::SharedPtr node, mjModel* model, mjData* data);
  virtual ~Ros2BridgeBase() = default;

  virtual void ApplyCommand() = 0;
  virtual void CaptureState() = 0;

  void StartStatePublisher();

  struct ImuSample
  {
    std::array<float, 4> quaternion{};
    std::array<float, 3> gyroscope{};
    std::array<float, 3> accelerometer{};
    std::array<float, 3> rpy{};
  };

protected:
  struct CommonStateSnapshot
  {
    unitree_go::msg::SportModeState sport_mode_state;
    unitree_go::msg::WirelessController wireless_controller;
    std::array<std::uint8_t, 40> wireless_remote{};
  };

  int SensorAddress(const char* name) const;
  ImuSample ReadImu(int quaternion_address, int gyroscope_address, int accelerometer_address) const;
  CommonStateSnapshot CaptureCommonState();
  void PublishCommonState(const CommonStateSnapshot& snapshot);
  virtual void PublishSnapshot() = 0;
  void PrintSceneInformation() const;

  rclcpp::Node::SharedPtr node_;
  mjModel* model_;
  mjData* data_;
  int num_motor_ = 0;

  int imu_quaternion_address_ = -1;
  int imu_gyroscope_address_ = -1;
  int imu_accelerometer_address_ = -1;
  int frame_position_address_ = -1;
  int frame_velocity_address_ = -1;
  int secondary_imu_quaternion_address_ = -1;
  int secondary_imu_gyroscope_address_ = -1;
  int secondary_imu_accelerometer_address_ = -1;

  std::unique_ptr<PhysicsJoystick> joystick_;

private:
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
  rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sport_mode_state_publisher_;
  rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_publisher_;
};

std::unique_ptr<Ros2BridgeBase> CreateRos2Bridge(
    const rclcpp::Node::SharedPtr& node, mjModel* model, mjData* data);
