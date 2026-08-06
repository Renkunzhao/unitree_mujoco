#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "joystick/joystick.h"

inline int JoystickMaxValue(int bits)
{
  if (bits < 2 || bits > 30)
  {
    throw std::runtime_error("joystick_bits must be between 2 and 30");
  }
  return 1 << (bits - 1);
}

struct JoystickState
{
  bool r1 = false;
  bool l1 = false;
  bool start = false;
  bool select = false;
  bool f1 = false;
  bool f2 = false;
  bool a = false;
  bool b = false;
  bool x = false;
  bool y = false;
  bool up = false;
  bool right = false;
  bool down = false;
  bool left = false;
  float lt = 0.0f;
  float rt = 0.0f;
  float lx = 0.0f;
  float ly = 0.0f;
  float rx = 0.0f;
  float ry = 0.0f;

  std::uint16_t Buttons() const
  {
    std::uint16_t buttons = 0;
    buttons |= static_cast<std::uint16_t>(r1) << 0;
    buttons |= static_cast<std::uint16_t>(l1) << 1;
    buttons |= static_cast<std::uint16_t>(start) << 2;
    buttons |= static_cast<std::uint16_t>(select) << 3;
    buttons |= static_cast<std::uint16_t>(rt > 0.5f) << 4;
    buttons |= static_cast<std::uint16_t>(lt > 0.5f) << 5;
    buttons |= static_cast<std::uint16_t>(f1) << 6;
    buttons |= static_cast<std::uint16_t>(f2) << 7;
    buttons |= static_cast<std::uint16_t>(a) << 8;
    buttons |= static_cast<std::uint16_t>(b) << 9;
    buttons |= static_cast<std::uint16_t>(x) << 10;
    buttons |= static_cast<std::uint16_t>(y) << 11;
    buttons |= static_cast<std::uint16_t>(up) << 12;
    buttons |= static_cast<std::uint16_t>(right) << 13;
    buttons |= static_cast<std::uint16_t>(down) << 14;
    buttons |= static_cast<std::uint16_t>(left) << 15;
    return buttons;
  }

  std::array<std::uint8_t, 40> RemoteData() const
  {
    struct RemoteDataLayout
    {
      std::uint8_t head[2];
      std::uint16_t buttons;
      float lx;
      float rx;
      float ry;
      float l2;
      float ly;
      std::uint8_t reserved[16];
    };
    static_assert(sizeof(RemoteDataLayout) == 40);

    RemoteDataLayout data{};
    data.head[0] = 0xFE;
    data.head[1] = 0xEF;
    data.buttons = Buttons();
    data.lx = lx;
    data.rx = rx;
    data.ry = ry;
    data.l2 = lt;
    data.ly = ly;

    std::array<std::uint8_t, 40> bytes{};
    std::memcpy(bytes.data(), &data, bytes.size());
    return bytes;
  }
};

class PhysicsJoystick
{
public:
  virtual ~PhysicsJoystick() = default;
  virtual void Update() = 0;

  const JoystickState& State() const
  {
    return state_;
  }

protected:
  static float SmoothAxis(float current, float input)
  {
    constexpr float kDeadZone = 0.01f;
    constexpr float kSmooth = 0.03f;
    const float filtered = std::abs(input) < kDeadZone ? 0.0f : input;
    return current * (1.0f - kSmooth) + filtered * kSmooth;
  }

  JoystickState state_;
};

class XBoxJoystick final : public PhysicsJoystick
{
public:
  XBoxJoystick(const std::string& device, int bits)
      : joystick_(std::make_unique<Joystick>(device)), max_value_(JoystickMaxValue(bits))
  {
    if (!joystick_->isFound())
    {
      throw std::runtime_error("failed to open joystick device " + device);
    }
  }

  void Update() override
  {
    joystick_->getState();
    state_.select = joystick_->button_[6];
    state_.start = joystick_->button_[7];
    state_.l1 = joystick_->button_[4];
    state_.r1 = joystick_->button_[5];
    state_.a = joystick_->button_[0];
    state_.b = joystick_->button_[1];
    state_.x = joystick_->button_[2];
    state_.y = joystick_->button_[3];
    state_.up = joystick_->axis_[7] < 0;
    state_.down = joystick_->axis_[7] > 0;
    state_.left = joystick_->axis_[6] < 0;
    state_.right = joystick_->axis_[6] > 0;
    state_.lt = SmoothAxis(state_.lt, joystick_->axis_[2] > 0 ? 1.0f : 0.0f);
    state_.rt = SmoothAxis(state_.rt, joystick_->axis_[5] > 0 ? 1.0f : 0.0f);
    state_.lx = SmoothAxis(state_.lx, Normalize(joystick_->axis_[0]));
    state_.ly = SmoothAxis(state_.ly, -Normalize(joystick_->axis_[1]));
    state_.rx = SmoothAxis(state_.rx, Normalize(joystick_->axis_[3]));
    state_.ry = SmoothAxis(state_.ry, -Normalize(joystick_->axis_[4]));
  }

private:
  float Normalize(int value) const
  {
    return std::clamp(static_cast<float>(value) / max_value_, -1.0f, 1.0f);
  }

  std::unique_ptr<Joystick> joystick_;
  int max_value_;
};

class SwitchJoystick final : public PhysicsJoystick
{
public:
  SwitchJoystick(const std::string& device, int bits)
      : joystick_(std::make_unique<Joystick>(device)), max_value_(JoystickMaxValue(bits))
  {
    if (!joystick_->isFound())
    {
      throw std::runtime_error("failed to open joystick device " + device);
    }
  }

  void Update() override
  {
    joystick_->getState();
    state_.select = joystick_->button_[10];
    state_.start = joystick_->button_[11];
    state_.l1 = joystick_->button_[6];
    state_.r1 = joystick_->button_[7];
    state_.a = joystick_->button_[0];
    state_.b = joystick_->button_[1];
    state_.x = joystick_->button_[3];
    state_.y = joystick_->button_[4];
    state_.up = joystick_->axis_[7] < 0;
    state_.down = joystick_->axis_[7] > 0;
    state_.left = joystick_->axis_[6] < 0;
    state_.right = joystick_->axis_[6] > 0;
    state_.lt = SmoothAxis(state_.lt, joystick_->axis_[5] > 0 ? 1.0f : 0.0f);
    state_.rt = SmoothAxis(state_.rt, joystick_->axis_[4] > 0 ? 1.0f : 0.0f);
    state_.lx = SmoothAxis(state_.lx, Normalize(joystick_->axis_[0]));
    state_.ly = SmoothAxis(state_.ly, -Normalize(joystick_->axis_[1]));
    state_.rx = SmoothAxis(state_.rx, Normalize(joystick_->axis_[2]));
    state_.ry = SmoothAxis(state_.ry, -Normalize(joystick_->axis_[3]));
  }

private:
  float Normalize(int value) const
  {
    return std::clamp(static_cast<float>(value) / max_value_, -1.0f, 1.0f);
  }

  std::unique_ptr<Joystick> joystick_;
  int max_value_;
};
