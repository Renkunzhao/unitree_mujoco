#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

namespace param
{

struct CameraMountConfig
{
    std::array<double, 3> position_m{};
    std::array<double, 3> rpy_rad{};
};

struct CameraIntrinsics
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::string distortion_model;
    std::array<double, 5> distortion{};
};

struct CameraProfile
{
    int width = 0;
    int height = 0;
    double fps = 0.0;
    double depth_scale_m = 0.0;
    CameraIntrinsics intrinsics;
};

struct CameraRosConfig
{
    std::string camera_link_frame_id;
    std::string depth_frame_id;
    std::string optical_frame_id;
    std::string depth_topic;
    std::string camera_info_topic;
};

struct DepthCameraConfig
{
    bool enabled = false;
    bool exclude_self = false;
    std::string camera_name;
    std::string parent_frame;
    CameraMountConfig mount;
    std::string model;
    std::string profile_name;
    std::filesystem::path noise_config_path;
    CameraProfile profile;
    CameraRosConfig ros;
};

template <std::size_t N>
std::array<double, N> read_array(const YAML::Node& node, const std::string& field_name)
{
    if (!node || !node.IsSequence() || node.size() != N)
    {
        throw std::runtime_error(field_name + " must contain exactly " + std::to_string(N) + " values");
    }

    std::array<double, N> values{};
    for (std::size_t i = 0; i < N; ++i)
    {
        values[i] = node[i].as<double>();
        if (!std::isfinite(values[i]))
        {
            throw std::runtime_error(field_name + " contains a non-finite value");
        }
    }
    return values;
}

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    bool no_viewer = false;
    bool no_joystick = false;
    bool beam_monitor = false;
    double episode_timeout_s = 20.0;
    std::optional<int> max_payload_count;
    std::string payload_mode = "fixed";
    std::int64_t payload_seed = 42;

    DepthCameraConfig depth_camera;

    void load_from_yaml(const std::filesystem::path& filename)
    {
        try
        {
            const auto cfg = YAML::LoadFile(filename.string());
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            no_viewer = cfg["no_viewer"].as<bool>();

            const auto eval_path = cfg["eval"].as<std::string>();
            if (!eval_path.empty())
            {
                const auto eval = YAML::LoadFile((filename.parent_path() / eval_path).string());
                robot_scene = eval["robot_scene"].as<std::string>();
                beam_monitor = eval["beam_monitor"].as<bool>();
                episode_timeout_s = eval["episode_timeout_s"].as<double>();
                if (eval["max_payload_count"])
                {
                    max_payload_count = eval["max_payload_count"].as<int>();
                }
                payload_mode = eval["payload_mode"].as<std::string>();
                payload_seed = eval["payload_seed"].as<std::int64_t>();
            }

            const auto camera = cfg["depth_camera"];
            depth_camera.enabled = camera["enabled"].as<bool>();
            depth_camera.exclude_self = camera["exclude_self"].as<bool>();
            depth_camera.camera_name = camera["camera_name"].as<std::string>();
            depth_camera.parent_frame = camera["parent_frame"].as<std::string>();
            depth_camera.mount.position_m =
                read_array<3>(camera["mount"]["position_m"], "depth_camera.mount.position_m");
            depth_camera.mount.rpy_rad =
                read_array<3>(camera["mount"]["rpy_rad"], "depth_camera.mount.rpy_rad");
            depth_camera.model = camera["sensor"]["model"].as<std::string>();
            depth_camera.profile_name = camera["sensor"]["profile"].as<std::string>();
            depth_camera.noise_config_path = camera["noise_config_path"].as<std::string>();
            depth_camera.ros.camera_link_frame_id =
                camera["ros"]["camera_link_frame_id"].as<std::string>();
            depth_camera.ros.depth_frame_id = camera["ros"]["depth_frame_id"].as<std::string>();
            depth_camera.ros.optical_frame_id = camera["ros"]["optical_frame_id"].as<std::string>();
            depth_camera.ros.depth_topic = camera["ros"]["depth_topic"].as<std::string>();
            depth_camera.ros.camera_info_topic = camera["ros"]["camera_info_topic"].as<std::string>();

            const auto catalog = YAML::LoadFile((filename.parent_path() / "camera_profiles.yaml").string());
            const auto model = catalog["models"][depth_camera.model];
            if (!model)
            {
                throw std::runtime_error("unknown depth camera model: " + depth_camera.model);
            }
            const auto profile = model["profiles"][depth_camera.profile_name];
            if (!profile)
            {
                throw std::runtime_error(
                    "unknown depth camera profile: " + depth_camera.model + "/" + depth_camera.profile_name);
            }

            depth_camera.profile.width = profile["width"].as<int>();
            depth_camera.profile.height = profile["height"].as<int>();
            depth_camera.profile.fps = profile["fps"].as<double>();
            depth_camera.profile.depth_scale_m = profile["depth_scale_m"].as<double>();
            depth_camera.profile.intrinsics.fx = profile["fx"].as<double>();
            depth_camera.profile.intrinsics.fy = profile["fy"].as<double>();
            depth_camera.profile.intrinsics.cx = profile["cx"].as<double>();
            depth_camera.profile.intrinsics.cy = profile["cy"].as<double>();
            depth_camera.profile.intrinsics.distortion_model = profile["distortion_model"].as<std::string>();
            depth_camera.profile.intrinsics.distortion =
                read_array<5>(profile["distortion"], "camera profile distortion");

            const auto& p = depth_camera.profile;
            if (p.width <= 0 || p.height <= 0 || !std::isfinite(p.fps) || p.fps <= 0.0 ||
                !std::isfinite(p.depth_scale_m) || p.depth_scale_m <= 0.0)
            {
                throw std::runtime_error("depth camera profile has invalid resolution, fps, or depth scale");
            }
            if (!std::isfinite(p.intrinsics.fx) || !std::isfinite(p.intrinsics.fy) ||
                !std::isfinite(p.intrinsics.cx) || !std::isfinite(p.intrinsics.cy) ||
                p.intrinsics.fx <= 0.0 || p.intrinsics.fy <= 0.0 ||
                p.intrinsics.cx < 0.0 || p.intrinsics.cx > p.width ||
                p.intrinsics.cy < 0.0 || p.intrinsics.cy > p.height)
            {
                throw std::runtime_error("depth camera profile has invalid intrinsics");
            }
            if (depth_camera.camera_name.empty() || depth_camera.parent_frame.empty() ||
                depth_camera.ros.camera_link_frame_id.empty() ||
                depth_camera.ros.depth_frame_id.empty() || depth_camera.ros.optical_frame_id.empty() ||
                depth_camera.ros.depth_topic.empty() || depth_camera.ros.camera_info_topic.empty())
            {
                throw std::runtime_error("depth camera names, frame, and topics must not be empty");
            }
            if (depth_camera.parent_frame == depth_camera.ros.camera_link_frame_id ||
                depth_camera.ros.camera_link_frame_id == depth_camera.ros.depth_frame_id ||
                depth_camera.ros.depth_frame_id == depth_camera.ros.optical_frame_id)
            {
                throw std::runtime_error("depth camera TF frames must be distinct");
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to load " << filename << ": " << e.what() << '\n';
            std::exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

// This function must be called at the beginning of main().
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("depth-camera", po::value<bool>(&config.depth_camera.enabled)->implicit_value(true),
         "Enable the configured ROS 2 depth camera")
        ("depth-noise-config", po::value<std::filesystem::path>(&config.depth_camera.noise_config_path),
         "Absolute path to the depth image noise YAML")
        ("no-viewer", po::value<bool>(&config.no_viewer)->implicit_value(true),
         "Run without the interactive MuJoCo viewer")
        ("no-joystick", po::value<bool>(&config.no_joystick)->implicit_value(true),
         "Disable the configured physical joystick publisher")
        ("beam-monitor", po::value<bool>(&config.beam_monitor)->implicit_value(true),
         "Enable ground-truth beam episode monitoring")
        ("max-payload-count", po::value<int>(),
         "Add a basket and sample a payload count uniformly from 0 through N")
        ("payload-mode", po::value<std::string>(&config.payload_mode),
         "Payload sampling: fixed or resample on ResetEpisode")
        ("payload-seed", po::value<std::int64_t>(&config.payload_seed),
         "Payload random seed in [0, 4294967295]")
        ("episode-timeout", po::value<double>(&config.episode_timeout_s),
         "Episode timeout in simulation seconds");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("max-payload-count"))
    {
        config.max_payload_count = vm["max-payload-count"].as<int>();
    }

    if (!std::isfinite(config.episode_timeout_s) || config.episode_timeout_s <= 0.0)
    {
        throw std::runtime_error("episode_timeout_s must be finite and positive");
    }
    if (config.max_payload_count &&
        (*config.max_payload_count < 0 || *config.max_payload_count > 36))
    {
        throw std::runtime_error("max_payload_count must lie in [0, 36]");
    }
    if (config.payload_mode != "fixed" && config.payload_mode != "resample")
    {
        throw std::runtime_error("payload_mode must be fixed or resample");
    }
    if (config.payload_mode == "resample" &&
        (!config.max_payload_count || !config.beam_monitor))
    {
        throw std::runtime_error("payload_mode resample requires max_payload_count and beam_monitor");
    }
    if (config.payload_seed < 0 ||
        config.payload_seed > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error("payload_seed must lie in [0, 4294967295]");
    }
    if (!config.depth_camera.noise_config_path.empty() &&
        !config.depth_camera.noise_config_path.is_absolute())
    {
        throw std::runtime_error("depth noise config path must be absolute");
    }
    if (config.no_joystick)
    {
        config.use_joystick = 0;
    }

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        std::exit(0);
    }

    return vm;
}

}  // namespace param
