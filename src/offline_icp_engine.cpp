#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <norlab_icp_mapper/Mapper.h>
#include <norlab_icp_mapper/MapperModules/PointDistanceMapperModule.h>

namespace fs = std::filesystem;

namespace {

using PM = PointMatcher<float>;
using DP = PM::DataPoints;
using TP = PM::TransformationParameters;

struct Args
{
  fs::path bag;
  fs::path output;
  fs::path mapping_config;
  std::string points_topic{"/hesai_lidar/points"};
  std::string profile{"hesai_lidar_only"};
  double local_map_radius_m{60.0};
  double global_min_dist_new_point_m{0.05};
  double local_min_dist_new_point_m{0.08};
  double min_translation_update_m{0.05};
  double min_yaw_update_deg{0.5};
  int checkpoint_every_scans{500};
  int max_clouds{0};
};

struct Stats
{
  uint64_t clouds_seen{0};
  uint64_t clouds_processed{0};
  uint64_t accepted{0};
  uint64_t rejected{0};
  uint64_t map_updates{0};
  double trajectory_length_m{0.0};
  double max_step_m{0.0};
  double max_yaw_step_deg{0.0};
};

struct PoseRow
{
  int64_t stamp_ns{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw_deg{0.0};
  bool accepted{false};
  std::string reason;
};

double yawFromTransform(const TP& t)
{
  return std::atan2(static_cast<double>(t(1, 0)), static_cast<double>(t(0, 0)));
}

double wrapToPi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

std::string param(double v)
{
  std::ostringstream ss;
  ss << v;
  return ss.str();
}

std::string param(int v)
{
  return std::to_string(v);
}

TP identity()
{
  return TP::Identity(4, 4);
}

TP yawTransform(double yaw_rad, double x, double y, double z)
{
  TP t = identity();
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  t(0, 0) = static_cast<float>(c);
  t(0, 1) = static_cast<float>(-s);
  t(1, 0) = static_cast<float>(s);
  t(1, 1) = static_cast<float>(c);
  t(0, 3) = static_cast<float>(x);
  t(1, 3) = static_cast<float>(y);
  t(2, 3) = static_cast<float>(z);
  return t;
}

TP baseLinkToHesai()
{
  // URDF net: base_link -> hesai_lidar ≈ T(-0.097, -0.050, 0.878), yaw +90 deg.
  // base_footprint -> base_link is z=-0.1, so base_link -> hesai z is 0.878.
  return yawTransform(M_PI_2, -0.0974808, -0.0500217, 0.878301);
}

bool finitePoint(const DP& cloud, int col)
{
  return std::isfinite(cloud.features(0, col)) &&
         std::isfinite(cloud.features(1, col)) &&
         std::isfinite(cloud.features(2, col));
}

DP cropRadius(const DP& map, const Eigen::Vector2f& center, double radius_m)
{
  if (map.getNbPoints() == 0) {
    return map;
  }

  const double r2 = radius_m * radius_m;
  std::vector<int> keep;
  keep.reserve(map.getNbPoints());
  for (int i = 0; i < static_cast<int>(map.getNbPoints()); ++i) {
    if (!finitePoint(map, i)) {
      continue;
    }
    const double dx = static_cast<double>(map.features(0, i) - center.x());
    const double dy = static_cast<double>(map.features(1, i) - center.y());
    if (dx * dx + dy * dy <= r2) {
      keep.push_back(i);
    }
  }

  DP out = map.createSimilarEmpty(static_cast<DP::Index>(keep.size()));
  for (int dst = 0; dst < static_cast<int>(keep.size()); ++dst) {
    out.setColFrom(dst, map, keep[dst]);
  }
  return out;
}

void ensureNormals(DP& cloud, std::shared_ptr<PM::DataPointsFilter>& normal_filter)
{
  if (!cloud.descriptorExists("normals", static_cast<unsigned>(cloud.getEuclideanDim()))) {
    normal_filter->inPlaceFilter(cloud);
  }
}

void saveTrajectoryCsv(const fs::path& path, const std::vector<PoseRow>& poses)
{
  std::ofstream out(path);
  out << "stamp_ns,x,y,z,yaw_deg,accepted,reason\n";
  for (const auto& p : poses) {
    out << p.stamp_ns << ','
        << p.x << ','
        << p.y << ','
        << p.z << ','
        << p.yaw_deg << ','
        << (p.accepted ? 1 : 0) << ','
        << '"' << p.reason << '"' << '\n';
  }
}

DP trajectoryToDataPoints(const std::vector<PoseRow>& poses)
{
  DP::Labels labels;
  labels.emplace_back("x", 1);
  labels.emplace_back("y", 1);
  labels.emplace_back("z", 1);
  labels.emplace_back("pad", 1);
  PM::Matrix features(4, poses.size());
  for (int i = 0; i < static_cast<int>(poses.size()); ++i) {
    features(0, i) = static_cast<float>(poses[i].x);
    features(1, i) = static_cast<float>(poses[i].y);
    features(2, i) = static_cast<float>(poses[i].z);
    features(3, i) = 1.0f;
  }
  return DP(features, labels);
}

void writeQualityYaml(const fs::path& path, const Args& args, const Stats& stats, const DP& global_map)
{
  std::ofstream out(path);
  const double acceptance = stats.clouds_processed > 0
    ? static_cast<double>(stats.accepted) / static_cast<double>(stats.clouds_processed)
    : 0.0;
  out << "status: " << (stats.accepted > 1 ? "OK" : "FAIL") << "\n";
  out << "engine: direct_rosbag2_libpointmatcher_v1\n";
  out << "profile: " << args.profile << "\n";
  out << "points_topic: " << args.points_topic << "\n";
  out << "mapping_config: " << args.mapping_config.string() << "\n";
  out << "clouds_seen: " << stats.clouds_seen << "\n";
  out << "clouds_processed: " << stats.clouds_processed << "\n";
  out << "accepted: " << stats.accepted << "\n";
  out << "rejected: " << stats.rejected << "\n";
  out << "acceptance_ratio: " << acceptance << "\n";
  out << "map_updates: " << stats.map_updates << "\n";
  out << "map_points: " << global_map.getNbPoints() << "\n";
  out << "trajectory_length_m: " << stats.trajectory_length_m << "\n";
  out << "max_step_m: " << stats.max_step_m << "\n";
  out << "max_yaw_step_deg: " << stats.max_yaw_step_deg << "\n";
  out << "local_map_radius_m: " << args.local_map_radius_m << "\n";
  out << "global_min_dist_new_point_m: " << args.global_min_dist_new_point_m << "\n";
}

void checkpoint(
  const Args& args,
  const Stats& stats,
  const DP& global_map,
  const std::vector<PoseRow>& poses)
{
  const fs::path checkpoint_dir = args.output / "checkpoints";
  fs::create_directories(checkpoint_dir);
  const std::string stem = "scan_" + std::to_string(stats.clouds_seen);
  global_map.save((checkpoint_dir / (stem + "_map.vtk")).string());
  trajectoryToDataPoints(poses).save((checkpoint_dir / (stem + "_trajectory.vtk")).string());
  saveTrajectoryCsv(checkpoint_dir / (stem + "_trajectory.csv"), poses);
  std::ofstream out(checkpoint_dir / (stem + "_checkpoint.yaml"));
  out << "clouds_seen: " << stats.clouds_seen << "\n";
  out << "accepted: " << stats.accepted << "\n";
  out << "map_points: " << global_map.getNbPoints() << "\n";
}

Args parseArgs(int argc, char** argv)
{
  Args args;
  std::unordered_map<std::string, std::string> kv;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key.rfind("--", 0) == 0 && i + 1 < argc) {
      kv[key] = argv[++i];
    }
  }

  auto get = [&](const std::string& key, const std::string& def = "") {
    auto it = kv.find(key);
    return it == kv.end() ? def : it->second;
  };

  args.bag = get("--bag");
  args.output = get("--output");
  args.mapping_config = get("--mapping-config");
  args.points_topic = get("--points-topic", args.points_topic);
  args.profile = get("--profile", args.profile);
  args.local_map_radius_m = std::stod(get("--local-map-radius", "60.0"));
  args.global_min_dist_new_point_m = std::stod(get("--global-min-dist-new-point", "0.05"));
  args.local_min_dist_new_point_m = std::stod(get("--local-min-dist-new-point", "0.08"));
  args.min_translation_update_m = std::stod(get("--min-translation-update", "0.05"));
  args.min_yaw_update_deg = std::stod(get("--min-yaw-update-deg", "0.5"));
  args.checkpoint_every_scans = std::stoi(get("--checkpoint-every-scans", "500"));
  args.max_clouds = std::stoi(get("--max-clouds", "0"));

  if (args.bag.empty() || args.output.empty() || args.mapping_config.empty()) {
    throw std::runtime_error(
      "Usage: offline_icp_engine --bag <bag_dir> --output <run_dir> "
      "--mapping-config <yaml> [--points-topic /hesai_lidar/points]");
  }
  return args;
}

template<class MsgT>
MsgT deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_msg)
{
  rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
  rclcpp::Serialization<MsgT> serializer;
  MsgT msg;
  serializer.deserialize_message(&serialized, &msg);
  return msg;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto wall_start = std::chrono::steady_clock::now();

  try {
    const Args args = parseArgs(argc, argv);
    fs::create_directories(args.output);
    fs::create_directories(args.output / "logs");

    auto transformation = PM::get().TransformationRegistrar.create("RigidTransformation");
    auto normal_filter = PM::get().DataPointsFilterRegistrar.create(
      "SurfaceNormalDataPointsFilter", {{"knn", param(12)}});
    auto local_mapper_module = std::make_shared<PointDistanceMapperModule>(
      PM::Parameters{{"minDistNewPoint", param(args.local_min_dist_new_point_m)}});
    auto global_mapper_module = std::make_shared<PointDistanceMapperModule>(
      PM::Parameters{{"minDistNewPoint", param(args.global_min_dist_new_point_m)}});

    norlab_icp_mapper::Mapper mapper(args.mapping_config.string(), true, false, false, false);
    TP sensor_to_map = identity();
    TP last_update_pose = identity();
    bool has_map = false;
    DP global_map;
    std::vector<PoseRow> poses;
    Stats stats;

    // baseLinkToHesai() = T_base_link_hesai (maps hesai points INTO base_link frame).
    // sensor_to_filtering = T_filteringFrame_sensor = T_base_link_hesai.
    // filtering_to_sensor = T_sensor_filteringFrame = T_hesai_base_link = inverse.
    const TP sensor_to_filtering = baseLinkToHesai();
    const TP filtering_to_sensor = baseLinkToHesai().inverse();

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = args.bag.string();
    storage_options.storage_id = "mcap";
    rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
    reader.open(storage_options, converter_options);

    std::ofstream log(args.output / "run.log");
    log << "offline_icp_engine started\n";
    log << "bag: " << args.bag << "\n";
    log << "output: " << args.output << "\n";
    log << "mapping_config: " << args.mapping_config << "\n";

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      if (bag_msg->topic_name != args.points_topic) {
        continue;
      }
      ++stats.clouds_seen;
      if (args.max_clouds > 0 && static_cast<int>(stats.clouds_seen) > args.max_clouds) {
        break;
      }

      sensor_msgs::msg::PointCloud2 cloud_msg = deserialize<sensor_msgs::msg::PointCloud2>(bag_msg);
      DP input = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloud_msg);
      ++stats.clouds_processed;

      try {
        input.features = sensor_to_filtering * input.features;
        mapper.applyInputFilters(input);
        input.features = filtering_to_sensor * input.features;
        ensureNormals(input, normal_filter);

        if (!has_map) {
          global_map = transformation->compute(input, sensor_to_map);
          ensureNormals(global_map, normal_filter);
          mapper.setMap(global_map);
          last_update_pose = sensor_to_map;
          has_map = true;
        } else {
          mapper.setIsMapping(false);
          mapper.processInput(
            input,
            sensor_to_map,
            std::chrono::time_point<std::chrono::steady_clock>(
              std::chrono::nanoseconds(cloud_msg.header.stamp.sec * 1000000000LL + cloud_msg.header.stamp.nanosec)));
          sensor_to_map = mapper.getPose();
        }

        const double x = sensor_to_map(0, 3);
        const double y = sensor_to_map(1, 3);
        const double z = sensor_to_map(2, 3);
        const double yaw = yawFromTransform(sensor_to_map);

        double step = 0.0;
        double yaw_step_deg = 0.0;
        if (!poses.empty()) {
          const double dx = x - poses.back().x;
          const double dy = y - poses.back().y;
          step = std::hypot(dx, dy);
          yaw_step_deg = std::abs(wrapToPi(yaw - poses.back().yaw_deg * M_PI / 180.0)) * 180.0 / M_PI;
          stats.trajectory_length_m += step;
          stats.max_step_m = std::max(stats.max_step_m, step);
          stats.max_yaw_step_deg = std::max(stats.max_yaw_step_deg, yaw_step_deg);
        }

        const double update_dist = std::hypot(
          static_cast<double>(sensor_to_map(0, 3) - last_update_pose(0, 3)),
          static_cast<double>(sensor_to_map(1, 3) - last_update_pose(1, 3)));
        const double update_yaw = std::abs(wrapToPi(yaw - yawFromTransform(last_update_pose))) * 180.0 / M_PI;
        if (stats.accepted == 0 ||
            update_dist >= args.min_translation_update_m ||
            update_yaw >= args.min_yaw_update_deg) {
          DP scan_in_map = transformation->compute(input, sensor_to_map);
          DP local_map = mapper.getMap();
          global_mapper_module->inPlaceUpdateMap(scan_in_map, global_map, sensor_to_map);
          local_mapper_module->inPlaceUpdateMap(scan_in_map, local_map, sensor_to_map);
          const Eigen::Vector2f center(sensor_to_map(0, 3), sensor_to_map(1, 3));
          local_map = cropRadius(local_map, center, args.local_map_radius_m);
          ensureNormals(local_map, normal_filter);
          mapper.setMap(local_map);
          last_update_pose = sensor_to_map;
          ++stats.map_updates;
        }

        poses.push_back(PoseRow{
          static_cast<int64_t>(cloud_msg.header.stamp.sec) * 1000000000LL +
            static_cast<int64_t>(cloud_msg.header.stamp.nanosec),
          x, y, z, yaw * 180.0 / M_PI, true, ""});
        ++stats.accepted;
      } catch (const std::exception& e) {
        ++stats.rejected;
        poses.push_back(PoseRow{
          static_cast<int64_t>(cloud_msg.header.stamp.sec) * 1000000000LL +
            static_cast<int64_t>(cloud_msg.header.stamp.nanosec),
          sensor_to_map(0, 3), sensor_to_map(1, 3), sensor_to_map(2, 3),
          yawFromTransform(sensor_to_map) * 180.0 / M_PI, false, e.what()});
        log << "reject cloud=" << stats.clouds_seen << " reason=" << e.what() << "\n";
      }

      if (args.checkpoint_every_scans > 0 &&
          stats.clouds_seen % static_cast<uint64_t>(args.checkpoint_every_scans) == 0 &&
          global_map.getNbPoints() > 0) {
        checkpoint(args, stats, global_map, poses);
      }

      if (stats.clouds_seen % 100 == 0) {
        const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - wall_start).count();
        std::cout << "[direct_icp] clouds=" << stats.clouds_seen
                  << " accepted=" << stats.accepted
                  << " rejected=" << stats.rejected
                  << " map=" << global_map.getNbPoints()
                  << " elapsed=" << elapsed << "s" << std::endl;
      }
    }

    if (global_map.getNbPoints() > 0) {
      global_map.save((args.output / "map.vtk").string());
    }
    trajectoryToDataPoints(poses).save((args.output / "trajectory.vtk").string());
    saveTrajectoryCsv(args.output / "trajectory.csv", poses);
    writeQualityYaml(args.output / "quality.yaml", args, stats, global_map);

    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - wall_start).count();
    std::cout << "[direct_icp] done clouds=" << stats.clouds_seen
              << " accepted=" << stats.accepted
              << " rejected=" << stats.rejected
              << " map=" << global_map.getNbPoints()
              << " elapsed=" << elapsed << "s" << std::endl;
    rclcpp::shutdown();
    return stats.accepted > 1 ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << "offline_icp_engine failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
