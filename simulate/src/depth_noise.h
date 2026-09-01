#pragma once

#include <cstdint>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

class DepthNoiseSimulator
{
public:
  struct Camera
  {
    int width = 0;
    int height = 0;
    double focal_length_px = 0.0;
    double depth_scale_m = 0.0;
  };

  DepthNoiseSimulator(const std::filesystem::path& config_path, Camera camera);

  void Reset();
  void Apply(std::vector<std::uint16_t>* depth);
  const std::string& description() const { return description_; }

private:
  enum class Operation
  {
    kStereoOcclusion,
    kCorrelatedDropout,
    kPixelDropout,
  };

  struct StereoOcclusion
  {
    double baseline_m = 0.0;
    double min_depth_jump_m = 0.0;
    int max_width_px = 0;
    bool left = true;
  };

  struct CorrelatedDropout
  {
    int cell_size_px = 0;
    double threshold_std = 0.0;
    double temporal_correlation = 0.0;
  };

  struct PixelDropout
  {
    double probability = 0.0;
  };

  struct AxisInterpolation
  {
    std::vector<int> lower;
    std::vector<int> upper;
    std::vector<double> upper_weight;
    std::vector<double> stddev;
  };

  void ApplyStereoOcclusion(std::vector<std::uint16_t>* depth);
  void ApplyCorrelatedDropout(std::vector<std::uint16_t>* depth);
  void ApplyPixelDropout(std::vector<std::uint16_t>* depth);
  void InitializeCorrelatedDropout();

  static AxisInterpolation MakeAxisInterpolation(int output_size, int input_size);

  Camera camera_;
  std::vector<Operation> process_order_;
  StereoOcclusion stereo_occlusion_;
  CorrelatedDropout correlated_dropout_;
  PixelDropout pixel_dropout_;
  std::mt19937 random_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::vector<double> correlated_field_;
  int field_width_ = 0;
  int field_height_ = 0;
  AxisInterpolation interpolation_x_;
  AxisInterpolation interpolation_y_;
  std::string description_;
};
