#pragma once

#include <array>
#include <memory>
#include <mutex>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace inertial_gt
{

std::array<float, 10> ComputeBaseInertialDelta(const mjModel* model);

class Publisher
{
public:
    Publisher(rclcpp::Node::SharedPtr node, const mjModel* model);
    void Update(const mjModel* model);

private:
    std::mutex mutex_;
    std_msgs::msg::Float32MultiArray message_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace inertial_gt
