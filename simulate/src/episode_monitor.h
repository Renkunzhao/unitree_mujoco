#pragma once

#include <memory>
#include <optional>
#include <string>

#include <mujoco/mujoco.h>

#include "episode_result.h"

// A task-specific outcome monitor.  The physics loop only knows this interface;
// EpisodeManager applies the returned terminal result, and a scene without a
// monitor still exposes reset/start services for client-side scoring.
class EpisodeMonitor
{
public:
  virtual ~EpisodeMonitor() = default;

  virtual std::optional<TerminalResult> Update(const mjData* data) const = 0;
};

// Empty name returns nullptr (no monitor).  Unknown names throw.
std::unique_ptr<EpisodeMonitor> CreateEpisodeMonitor(
    const std::string& name, const mjModel* model);
