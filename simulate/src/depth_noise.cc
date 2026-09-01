#include "depth_noise.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace
{
void RequireMapping(const YAML::Node& node, const std::string& name)
{
  if (!node || !node.IsMap())
  {
    throw std::runtime_error(name + " must be a mapping");
  }
}

void RequireExactKeys(
    const YAML::Node& node,
    const std::set<std::string>& required,
    const std::string& name)
{
  RequireMapping(node, name);
  std::set<std::string> actual;
  for (const auto& item : node)
  {
    actual.insert(item.first.as<std::string>());
  }
  if (actual == required)
  {
    return;
  }

  std::ostringstream message;
  message << name << " keys do not match the required schema";
  for (const auto& key : required)
  {
    if (!actual.count(key))
    {
      message << "; missing " << key;
    }
  }
  for (const auto& key : actual)
  {
    if (!required.count(key))
    {
      message << "; unknown " << key;
    }
  }
  throw std::runtime_error(message.str());
}

void RequireFinitePositive(double value, const std::string& name)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    throw std::runtime_error(name + " must be finite and positive");
  }
}
}  // namespace

DepthNoiseSimulator::DepthNoiseSimulator(
    const std::filesystem::path& config_path, Camera camera)
    : camera_(camera)
{
  if (!config_path.is_absolute())
  {
    throw std::runtime_error("depth noise config path must be absolute: " + config_path.string());
  }
  if (!std::filesystem::is_regular_file(config_path))
  {
    throw std::runtime_error("depth noise config is not a file: " + config_path.string());
  }
  if (camera_.width <= 0 || camera_.height <= 0)
  {
    throw std::runtime_error("depth noise camera dimensions must be positive");
  }
  RequireFinitePositive(camera_.focal_length_px, "depth noise focal_length_px");
  RequireFinitePositive(camera_.depth_scale_m, "depth noise depth_scale_m");

  const YAML::Node document = YAML::LoadFile(config_path.string());
  RequireExactKeys(document, {"depth_image_noise"}, "depth noise document");
  const YAML::Node root = document["depth_image_noise"];
  RequireMapping(root, "depth_image_noise");

  const YAML::Node order = root["process_order"];
  if (!order || !order.IsSequence() || order.size() == 0)
  {
    throw std::runtime_error("depth_image_noise.process_order must be a nonempty sequence");
  }

  std::set<std::string> operation_names;
  std::set<std::string> required_root_keys{"seed", "process_order"};
  for (const auto& item : order)
  {
    const std::string name = item.as<std::string>();
    if (!operation_names.insert(name).second)
    {
      throw std::runtime_error("duplicate depth noise operation: " + name);
    }
    required_root_keys.insert(name);
    if (name == "stereo_occlusion")
    {
      process_order_.push_back(Operation::kStereoOcclusion);
    }
    else if (name == "correlated_dropout")
    {
      process_order_.push_back(Operation::kCorrelatedDropout);
    }
    else if (name == "pixel_dropout")
    {
      process_order_.push_back(Operation::kPixelDropout);
    }
    else
    {
      throw std::runtime_error("unknown depth noise operation: " + name);
    }
  }
  RequireExactKeys(root, required_root_keys, "depth_image_noise");

  random_.seed(root["seed"].as<std::uint32_t>());

  if (operation_names.count("stereo_occlusion"))
  {
    const YAML::Node cfg = root["stereo_occlusion"];
    RequireExactKeys(
        cfg,
        {"baseline_m", "min_depth_jump_m", "max_width_px", "side"},
        "depth_image_noise.stereo_occlusion");
    stereo_occlusion_.baseline_m = cfg["baseline_m"].as<double>();
    stereo_occlusion_.min_depth_jump_m = cfg["min_depth_jump_m"].as<double>();
    stereo_occlusion_.max_width_px = cfg["max_width_px"].as<int>();
    const std::string side = cfg["side"].as<std::string>();
    RequireFinitePositive(stereo_occlusion_.baseline_m, "stereo_occlusion.baseline_m");
    RequireFinitePositive(
        stereo_occlusion_.min_depth_jump_m, "stereo_occlusion.min_depth_jump_m");
    if (stereo_occlusion_.max_width_px <= 0 ||
        stereo_occlusion_.max_width_px >= camera_.width)
    {
      throw std::runtime_error("stereo_occlusion.max_width_px must lie in [1, width)");
    }
    if (side != "left" && side != "right")
    {
      throw std::runtime_error("stereo_occlusion.side must be left or right");
    }
    stereo_occlusion_.left = side == "left";
  }

  if (operation_names.count("correlated_dropout"))
  {
    const YAML::Node cfg = root["correlated_dropout"];
    RequireExactKeys(
        cfg,
        {"cell_size_px", "threshold_std", "temporal_correlation"},
        "depth_image_noise.correlated_dropout");
    correlated_dropout_.cell_size_px = cfg["cell_size_px"].as<int>();
    correlated_dropout_.threshold_std = cfg["threshold_std"].as<double>();
    correlated_dropout_.temporal_correlation = cfg["temporal_correlation"].as<double>();
    if (correlated_dropout_.cell_size_px <= 0)
    {
      throw std::runtime_error("correlated_dropout.cell_size_px must be positive");
    }
    if (!std::isfinite(correlated_dropout_.threshold_std))
    {
      throw std::runtime_error("correlated_dropout.threshold_std must be finite");
    }
    if (!std::isfinite(correlated_dropout_.temporal_correlation) ||
        correlated_dropout_.temporal_correlation < 0.0 ||
        correlated_dropout_.temporal_correlation >= 1.0)
    {
      throw std::runtime_error("correlated_dropout.temporal_correlation must be in [0, 1)");
    }
    InitializeCorrelatedDropout();
  }

  if (operation_names.count("pixel_dropout"))
  {
    const YAML::Node cfg = root["pixel_dropout"];
    RequireExactKeys(cfg, {"probability"}, "depth_image_noise.pixel_dropout");
    pixel_dropout_.probability = cfg["probability"].as<double>();
    if (!std::isfinite(pixel_dropout_.probability) || pixel_dropout_.probability < 0.0 ||
        pixel_dropout_.probability > 1.0)
    {
      throw std::runtime_error("pixel_dropout.probability must be in [0, 1]");
    }
  }

  std::ostringstream description;
  description << config_path << " [";
  for (std::size_t i = 0; i < operation_names.size(); ++i)
  {
    if (i > 0)
    {
      description << ", ";
    }
    description << order[i].as<std::string>();
  }
  description << "]";
  description_ = description.str();
}

DepthNoiseSimulator::AxisInterpolation DepthNoiseSimulator::MakeAxisInterpolation(
    int output_size, int input_size)
{
  AxisInterpolation interpolation;
  interpolation.lower.resize(output_size);
  interpolation.upper.resize(output_size);
  interpolation.upper_weight.resize(output_size);
  interpolation.stddev.resize(output_size);
  for (int output = 0; output < output_size; ++output)
  {
    const double coordinate =
        (static_cast<double>(output) + 0.5) * input_size / output_size - 0.5;
    const int lower_unclamped = static_cast<int>(std::floor(coordinate));
    const int upper_unclamped = lower_unclamped + 1;
    const double upper_weight = coordinate - lower_unclamped;
    const int lower = std::clamp(lower_unclamped, 0, input_size - 1);
    const int upper = std::clamp(upper_unclamped, 0, input_size - 1);
    const double lower_weight = 1.0 - upper_weight;
    const double variance = lower == upper
                                ? 1.0
                                : lower_weight * lower_weight + upper_weight * upper_weight;
    interpolation.lower[output] = lower;
    interpolation.upper[output] = upper;
    interpolation.upper_weight[output] = upper_weight;
    interpolation.stddev[output] = std::sqrt(variance);
  }
  return interpolation;
}

void DepthNoiseSimulator::InitializeCorrelatedDropout()
{
  const int cell_size = correlated_dropout_.cell_size_px;
  field_width_ = (camera_.width + cell_size - 1) / cell_size;
  field_height_ = (camera_.height + cell_size - 1) / cell_size;
  correlated_field_.resize(static_cast<std::size_t>(field_width_) * field_height_);
  interpolation_x_ = MakeAxisInterpolation(camera_.width, field_width_);
  interpolation_y_ = MakeAxisInterpolation(camera_.height, field_height_);
  Reset();
}

void DepthNoiseSimulator::Reset()
{
  for (double& value : correlated_field_)
  {
    value = normal_(random_);
  }
}

void DepthNoiseSimulator::Apply(std::vector<std::uint16_t>* depth)
{
  if (!depth || depth->size() != static_cast<std::size_t>(camera_.width) * camera_.height)
  {
    throw std::runtime_error("depth noise input size does not match the camera profile");
  }
  for (const Operation operation : process_order_)
  {
    switch (operation)
    {
      case Operation::kStereoOcclusion:
        ApplyStereoOcclusion(depth);
        break;
      case Operation::kCorrelatedDropout:
        ApplyCorrelatedDropout(depth);
        break;
      case Operation::kPixelDropout:
        ApplyPixelDropout(depth);
        break;
    }
  }
}

void DepthNoiseSimulator::ApplyStereoOcclusion(std::vector<std::uint16_t>* depth)
{
  std::vector<bool> mask(depth->size(), false);
  const int width = camera_.width;
  const int height = camera_.height;
  for (int y = 0; y < height; ++y)
  {
    const std::size_t row = static_cast<std::size_t>(y) * width;
    for (int x = 0; x < width - 1; ++x)
    {
      const double left = (*depth)[row + x] * camera_.depth_scale_m;
      const double right = (*depth)[row + x + 1] * camera_.depth_scale_m;
      if (left <= 0.0 || right <= 0.0)
      {
        continue;
      }
      const double far = stereo_occlusion_.left ? left : right;
      const double near = stereo_occlusion_.left ? right : left;
      if (far - near < stereo_occlusion_.min_depth_jump_m)
      {
        continue;
      }
      const int occlusion_width = std::min(
          static_cast<int>(std::ceil(
              camera_.focal_length_px * stereo_occlusion_.baseline_m *
              (1.0 / near - 1.0 / far))),
          stereo_occlusion_.max_width_px);
      for (int offset = 0; offset < occlusion_width; ++offset)
      {
        const int target = stereo_occlusion_.left ? x - offset : x + 1 + offset;
        if (target < 0 || target >= width)
        {
          break;
        }
        mask[row + target] = true;
      }
    }
  }
  for (std::size_t index = 0; index < depth->size(); ++index)
  {
    if (mask[index])
    {
      (*depth)[index] = 0;
    }
  }
}

void DepthNoiseSimulator::ApplyCorrelatedDropout(std::vector<std::uint16_t>* depth)
{
  const double correlation = correlated_dropout_.temporal_correlation;
  const double innovation_scale = std::sqrt(1.0 - correlation * correlation);
  for (double& value : correlated_field_)
  {
    value = correlation * value + innovation_scale * normal_(random_);
  }

  for (int y = 0; y < camera_.height; ++y)
  {
    const int y0 = interpolation_y_.lower[y];
    const int y1 = interpolation_y_.upper[y];
    const double wy = interpolation_y_.upper_weight[y];
    for (int x = 0; x < camera_.width; ++x)
    {
      const int x0 = interpolation_x_.lower[x];
      const int x1 = interpolation_x_.upper[x];
      const double wx = interpolation_x_.upper_weight[x];
      const double top =
          (1.0 - wx) * correlated_field_[static_cast<std::size_t>(y0) * field_width_ + x0] +
          wx * correlated_field_[static_cast<std::size_t>(y0) * field_width_ + x1];
      const double bottom =
          (1.0 - wx) * correlated_field_[static_cast<std::size_t>(y1) * field_width_ + x0] +
          wx * correlated_field_[static_cast<std::size_t>(y1) * field_width_ + x1];
      const double score = ((1.0 - wy) * top + wy * bottom) /
                           (interpolation_y_.stddev[y] * interpolation_x_.stddev[x]);
      const std::size_t index = static_cast<std::size_t>(y) * camera_.width + x;
      if ((*depth)[index] != 0 && score > correlated_dropout_.threshold_std)
      {
        (*depth)[index] = 0;
      }
    }
  }
}

void DepthNoiseSimulator::ApplyPixelDropout(std::vector<std::uint16_t>* depth)
{
  std::bernoulli_distribution drop(pixel_dropout_.probability);
  for (std::uint16_t& value : *depth)
  {
    if (value != 0 && drop(random_))
    {
      value = 0;
    }
  }
}
