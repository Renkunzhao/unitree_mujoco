#include "inertial_gt.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace inertial_gt
{
namespace
{

constexpr double kEpsilon = 1.0e-9;
constexpr auto kPublishPeriod = std::chrono::milliseconds(20);
constexpr const char* kTopic = "/unitree_go2/inertial_delta";

using Matrix3 = std::array<double, 9>;
using Vector3 = std::array<double, 3>;

struct InertialComponent
{
    double mass;
    Vector3 com;
    Matrix3 inertia_com;
};

Matrix3 RotationMatrix(const mjtNum* quaternion)
{
    mjtNum matrix[9]{};
    mju_quat2Mat(matrix, quaternion);
    Matrix3 result{};
    std::copy(matrix, matrix + result.size(), result.begin());
    return result;
}

Matrix3 Multiply(const Matrix3& left, const Matrix3& right)
{
    Matrix3 result{};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            for (int inner = 0; inner < 3; ++inner)
            {
                result[3 * row + column] +=
                    left[3 * row + inner] * right[3 * inner + column];
            }
        }
    }
    return result;
}

Vector3 Rotate(const Matrix3& rotation, const mjtNum* vector)
{
    Vector3 result{};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            result[row] += rotation[3 * row + column] * vector[column];
        }
    }
    return result;
}

Matrix3 RotateDiagonalInertia(
    const Matrix3& rotation, const mjtNum* diagonal)
{
    Matrix3 result{};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                result[3 * row + column] +=
                    rotation[3 * row + axis] * diagonal[axis] *
                    rotation[3 * column + axis];
            }
        }
    }
    return result;
}

InertialComponent ReadComponent(
    const mjModel* model, int body_id, int base_body_id)
{
    Matrix3 body_rotation{1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0};
    Vector3 body_position{};
    if (body_id != base_body_id)
    {
        if (model->body_parentid[body_id] != base_body_id ||
            model->body_jntnum[body_id] != 0)
        {
            throw std::runtime_error(
                "Basket and payload bodies must be fixed direct children of base_link.");
        }
        body_rotation = RotationMatrix(model->body_quat + 4 * body_id);
        std::copy(
            model->body_pos + 3 * body_id,
            model->body_pos + 3 * body_id + 3,
            body_position.begin());
    }

    const Vector3 rotated_com = Rotate(
        body_rotation, model->body_ipos + 3 * body_id);
    Vector3 com{};
    for (int axis = 0; axis < 3; ++axis)
    {
        com[axis] = body_position[axis] + rotated_com[axis];
    }

    const Matrix3 inertia_rotation = Multiply(
        body_rotation,
        RotationMatrix(model->body_iquat + 4 * body_id));
    return {
        model->body_mass[body_id],
        com,
        RotateDiagonalInertia(
            inertia_rotation, model->body_inertia + 3 * body_id),
    };
}

InertialComponent Compose(const std::vector<InertialComponent>& components)
{
    InertialComponent total{};
    for (const InertialComponent& component : components)
    {
        if (!std::isfinite(component.mass) || component.mass <= 0.0)
        {
            throw std::runtime_error("Inertial component mass must be finite and positive.");
        }
        total.mass += component.mass;
        for (int axis = 0; axis < 3; ++axis)
        {
            total.com[axis] += component.mass * component.com[axis];
        }
    }
    for (double& value : total.com)
    {
        value /= total.mass;
    }

    for (const InertialComponent& component : components)
    {
        Vector3 offset{};
        double squared_norm = 0.0;
        for (int axis = 0; axis < 3; ++axis)
        {
            offset[axis] = component.com[axis] - total.com[axis];
            squared_norm += offset[axis] * offset[axis];
        }
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                const int index = 3 * row + column;
                total.inertia_com[index] += component.inertia_com[index];
                total.inertia_com[index] += component.mass *
                    ((row == column ? squared_norm : 0.0) -
                     offset[row] * offset[column]);
            }
        }
    }
    return total;
}

std::array<double, 10> LogCholeskyParameters(
    const InertialComponent& component)
{
    const Matrix3& inertia = component.inertia_com;
    const double trace = inertia[0] + inertia[4] + inertia[8];
    Matrix3 sigma{};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            const int index = 3 * row + column;
            sigma[index] =
                ((row == column ? 0.5 * trace : 0.0) - inertia[index]) /
                component.mass;
        }
    }

    const double d3 = std::sqrt(std::max(sigma[8], kEpsilon));
    const double s23 = sigma[5] / d3;
    const double s13 = sigma[2] / d3;
    const double d2 = std::sqrt(std::max(sigma[4] - s23 * s23, kEpsilon));
    const double s12 = (sigma[1] - s13 * s23) / d2;
    const double d1 = std::sqrt(std::max(
        sigma[0] - s12 * s12 - s13 * s13, kEpsilon));

    return {
        0.5 * std::log(std::max(component.mass, kEpsilon)),
        std::log(d1),
        std::log(d2),
        std::log(d3),
        s12,
        s23,
        s13,
        component.com[0],
        component.com[1],
        component.com[2],
    };
}

bool IsPayloadBody(const char* name)
{
    if (!name)
    {
        return false;
    }
    const std::string body_name(name);
    return body_name == "basket" || body_name.rfind("payload_", 0) == 0;
}

}  // namespace

std::array<float, 10> ComputeBaseInertialDelta(const mjModel* model)
{
    if (!model)
    {
        throw std::invalid_argument("Inertial GT requires a non-null mjModel.");
    }
    const int base_body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    if (base_body_id < 0)
    {
        throw std::runtime_error("Inertial GT requires a base_link body.");
    }

    const InertialComponent nominal = ReadComponent(
        model, base_body_id, base_body_id);
    std::vector<InertialComponent> components{nominal};
    for (int body_id = 0; body_id < model->nbody; ++body_id)
    {
        if (IsPayloadBody(mj_id2name(model, mjOBJ_BODY, body_id)))
        {
            components.push_back(ReadComponent(model, body_id, base_body_id));
        }
    }

    const auto nominal_parameters = LogCholeskyParameters(nominal);
    const auto current_parameters = LogCholeskyParameters(Compose(components));
    std::array<float, 10> delta{};
    for (std::size_t index = 0; index < delta.size(); ++index)
    {
        const double value = current_parameters[index] - nominal_parameters[index];
        if (!std::isfinite(value))
        {
            throw std::runtime_error("Computed inertial GT contains NaN/Inf.");
        }
        delta[index] = static_cast<float>(value);
    }
    return delta;
}

Publisher::Publisher(rclcpp::Node::SharedPtr node, const mjModel* model)
{
    if (!node)
    {
        throw std::invalid_argument("Inertial GT publisher requires a ROS 2 node.");
    }
    const auto delta = ComputeBaseInertialDelta(model);
    message_.data.assign(delta.begin(), delta.end());
    publisher_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
        kTopic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    timer_ = node->create_wall_timer(
        kPublishPeriod, [this]() { publisher_->publish(message_); });

    std::cout << "[INFO] Publishing base inertial GT delta on " << kTopic << ":";
    for (const float value : message_.data)
    {
        std::cout << ' ' << value;
    }
    std::cout << '\n';
}

}  // namespace inertial_gt
