#include "episode_monitor.h"

#include <stdexcept>

#include "beam_episode_monitor.h"

std::unique_ptr<EpisodeMonitor> CreateEpisodeMonitor(
    const std::string& name, const mjModel* model)
{
  if (name.empty())
  {
    return nullptr;
  }
  if (name == "beam")
  {
    return std::make_unique<BeamEpisodeMonitor>(model);
  }
  throw std::runtime_error(
      "unknown episode_monitor '" + name + "'; supported: 'beam' or empty");
}
