#pragma once

#include <mujoco/mujoco.h>

namespace payload
{

struct Sample
{
    int count;
    double total_added_mass_kg;
};

Sample AddRandomFixedPayload(mjSpec* spec, int max_payload_count);

}  // namespace payload
