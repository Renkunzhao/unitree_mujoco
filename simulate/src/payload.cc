#include "payload.h"

#include <array>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace payload
{
namespace
{

constexpr int kColumnsX = 2;
constexpr int kColumnsY = 3;
constexpr int kLayers = 6;
constexpr int kColumnCount = kColumnsX * kColumnsY;
constexpr int kCapacity = kColumnCount * kLayers;

constexpr double kItemMassKg = 1.0;
constexpr std::array<double, 3> kItemDimensionsM{0.15, 0.08, 0.03};
constexpr std::array<double, 2> kBasketBottomDimensionsM{0.30, 0.24};
constexpr double kBasketBottomZM = 0.09;
constexpr double kBasketMassKg = 0.76;
constexpr std::array<double, 3> kBasketComM{0.0, 0.0, 0.165};
constexpr std::array<double, 3> kBasketInertiaKgM2{
    0.005073, 0.007125, 0.009348};

void SetName(mjsElement* element, const std::string& name)
{
    if (mjs_setName(element, name.c_str()) != 0)
    {
        throw std::runtime_error("Failed to name payload element: " + name);
    }
}

void SetExplicitInertia(
    mjsBody* body,
    double mass,
    const std::array<double, 3>& com,
    const std::array<double, 3>& inertia)
{
    body->mass = mass;
    body->explicitinertial = 1;
    for (int axis = 0; axis < 3; ++axis)
    {
        body->ipos[axis] = com[axis];
        body->inertia[axis] = inertia[axis];
    }
}

mjsGeom* AddVisualGeom(mjsBody* body, const std::string& name)
{
    mjsGeom* geom = mjs_addGeom(body, nullptr);
    if (!geom)
    {
        throw std::runtime_error("Failed to create payload geom: " + name);
    }
    SetName(geom->element, name);
    geom->contype = 0;
    geom->conaffinity = 0;
    geom->group = 2;
    return geom;
}

std::array<double, 3> BoxInertia(
    double mass,
    const std::array<double, 3>& dimensions)
{
    const double x2 = dimensions[0] * dimensions[0];
    const double y2 = dimensions[1] * dimensions[1];
    const double z2 = dimensions[2] * dimensions[2];
    return {
        mass * (y2 + z2) / 12.0,
        mass * (x2 + z2) / 12.0,
        mass * (x2 + y2) / 12.0,
    };
}

}  // namespace

Sample AddRandomFixedPayload(mjSpec* spec, int max_payload_count)
{
    if (!spec)
    {
        throw std::invalid_argument("Payload requires a non-null mjSpec.");
    }
    if (max_payload_count < 0 || max_payload_count > kCapacity)
    {
        throw std::invalid_argument(
            "max_payload_count must lie in [0, " +
            std::to_string(kCapacity) + "].");
    }
    mjsBody* base_link = mjs_findBody(spec, "base_link");
    if (!base_link)
    {
        throw std::runtime_error("Payload requires a base_link body.");
    }
    if (!mjs_findElement(spec, mjOBJ_MESH, "basket") ||
        !mjs_findElement(spec, mjOBJ_MATERIAL, "basket") ||
        !mjs_findElement(spec, mjOBJ_MATERIAL, "payload"))
    {
        throw std::runtime_error(
            "Payload requires the basket mesh and basket/payload materials.");
    }

    mjsBody* basket = mjs_addBody(base_link, nullptr);
    if (!basket)
    {
        throw std::runtime_error("Failed to create basket body.");
    }
    SetName(basket->element, "basket");
    SetExplicitInertia(
        basket, kBasketMassKg, kBasketComM, kBasketInertiaKgM2);
    mjsGeom* basket_geom = AddVisualGeom(basket, "basket_visual");
    basket_geom->type = mjGEOM_MESH;
    basket_geom->pos[2] = kBasketComM[2];
    mjs_setString(basket_geom->meshname, "basket");
    mjs_setString(basket_geom->material, "basket");

    std::mt19937 random_engine(std::random_device{}());
    std::uniform_int_distribution<int> count_distribution(
        0, max_payload_count);
    const int payload_count = count_distribution(random_engine);
    std::array<int, kColumnCount> column_heights{};
    for (int item = 0; item < payload_count; ++item)
    {
        std::vector<int> available_columns;
        available_columns.reserve(kColumnCount);
        for (int column = 0; column < kColumnCount; ++column)
        {
            if (column_heights[column] < kLayers)
            {
                available_columns.push_back(column);
            }
        }
        std::uniform_int_distribution<std::size_t> column_distribution(
            0, available_columns.size() - 1);
        ++column_heights[available_columns[column_distribution(random_engine)]];
    }

    const std::array<double, 3> item_inertia =
        BoxInertia(kItemMassKg, kItemDimensionsM);
    int item_index = 0;
    for (int column = 0; column < kColumnCount; ++column)
    {
        const int x_index = column / kColumnsY;
        const int y_index = column % kColumnsY;
        const double x = (x_index + 0.5) * kItemDimensionsM[0] -
                         kBasketBottomDimensionsM[0] / 2.0;
        const double y = (y_index + 0.5) * kItemDimensionsM[1] -
                         kBasketBottomDimensionsM[1] / 2.0;
        for (int layer = 0; layer < column_heights[column]; ++layer)
        {
            const double z = kBasketBottomZM +
                             (layer + 0.5) * kItemDimensionsM[2];
            mjsBody* item = mjs_addBody(base_link, nullptr);
            if (!item)
            {
                throw std::runtime_error("Failed to create payload body.");
            }
            const std::string item_name =
                "payload_" + std::to_string(item_index++);
            SetName(item->element, item_name);
            item->pos[0] = x;
            item->pos[1] = y;
            item->pos[2] = z;
            SetExplicitInertia(item, kItemMassKg, {0.0, 0.0, 0.0}, item_inertia);

            mjsGeom* geom = AddVisualGeom(item, item_name + "_visual");
            geom->type = mjGEOM_BOX;
            for (int axis = 0; axis < 3; ++axis)
            {
                geom->size[axis] = kItemDimensionsM[axis] / 2.0;
            }
            mjs_setString(geom->material, "payload");
        }
    }

    return {payload_count, kBasketMassKg + payload_count * kItemMassKg};
}

}  // namespace payload
