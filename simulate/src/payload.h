#pragma once

#include <array>
#include <random>

#include <mujoco/mujoco.h>

namespace payload
{

struct Sample
{
    int count;
    double total_added_mass_kg;
    std::array<int, 6> column_heights;
};

void AddPayloadSlots(mjSpec* spec);
Sample SamplePayload(int max_payload_count, std::mt19937& random_engine);
// The caller must run mj_setConst before resetting and forwarding the data.
void ApplyPayload(mjModel* model, const Sample& sample);

}  // namespace payload
