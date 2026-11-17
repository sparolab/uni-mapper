#include "open_lmm_online.hpp"

#include "../runtime_adapter/ros_visualization_bridge.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace open_lmm {
namespace fs = std::filesystem;

namespace {

Eigen::Isometry3d ToIsometry(const nav_msgs::msg::Odometry& odometry) {
  const auto& p = odometry.pose.pose.position;
  const auto& q = odometry.pose.pose.orientation;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return pose;
}

std::string JsonString(const std::string& value) {
  std::string out = "\"";
  for (char c : value) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out + "\"";
}

std::string ReplaceOnce(const std::string& text, const std::regex& pattern,
                        const std::string& replacement, const char* what) {
  if (!std::regex_search(text, pattern)) {
    throw std::runtime_error(std::string("config.json has no ") + what);
  }
  return std::regex_replace(text, pattern, replacement,
                            std::regex_constants::format_first_only);
}

}  // namespace

OpenLMMOnline::OpenLMMOnline(const rclcpp::NodeOptions& options)
    : Node("open_lmm_online", options),
      runtime_(std::make_shared<RuntimeClient>()) {
  std::string config = declare_parameter<std::string>("config_path", "config");
  if (!config.empty() && config.front() != '/') {
    config = (fs::path(ament_index_cpp::get_package_share_directory(
                  "open_lmm")) /
              config)
                 .string();
  }
  base_config_ = config;
  if (!fs::exists(base_config_ / "config.json")) {
    throw std::invalid_argument("config_path has no config.json: " +
                                base_config_.string());
  }
  work_dir_ = declare_parameter<std::string>("work_dir", "/tmp/open_lmm_online");
  output_dir_ =
      declare_parameter<std::string>("output_dir", (work_dir_ / "output").string());
  const auto names = declare_parameter<std::vector<std::string>>(
      "agents", std::vector<std::string>{"robot1", "robot2"});
  const auto prefix = declare_parameter<std::string>("topic_prefix", "");
  const auto cloud_topic =
      declare_parameter<std::string>("cloud_topic", "cloud_registered_body");
  const auto odom_topic = declare_parameter<std::string>("odom_topic", "Odometry");
  const double merge_period_s = declare_parameter<double>("merge_period_s", 60.0);
  min_keyframes_per_agent_ = static_cast<std::size_t>(
      declare_parameter<int64_t>("min_keyframes_per_agent", 20));
  min_new_keyframes_ = static_cast<std::size_t>(
      declare_parameter<int64_t>("min_new_keyframes", 10));
  min_agents_ =
      static_cast<std::size_t>(declare_parameter<int64_t>("min_agents", 2));
  max_stamp_gap_s_ = declare_parameter<double>("max_stamp_gap_s", 0.02);
  keep_snapshots_ =
      static_cast<int>(declare_parameter<int64_t>("keep_snapshots", 2));
  live_frame_ = declare_parameter<std::string>("live_frame_id", "map");
  live_voxel_m_ = declare_parameter<double>("live_voxel_m", 0.3);
  const double live_period_s = declare_parameter<double>("live_period_s", 1.0);

  online::KeyframeParams kf;
  kf.min_translation_m = declare_parameter<double>("keyframe_translation_m", 1.0);
  kf.min_rotation_deg = declare_parameter<double>("keyframe_rotation_deg", 10.0);
  kf.scans_per_keyframe = static_cast<std::size_t>(
      declare_parameter<int64_t>("scans_per_keyframe", 5));
  kf.voxel_size_m = declare_parameter<double>("keyframe_voxel_m", 0.1);

  // One fresh recording directory per process start.
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto session =
      std::to_string(std::chrono::duration_cast<std::chrono::seconds>(now).count());
  const fs::path live = work_dir_ / ("session_" + session) / "live";
  work_dir_ = work_dir_ / ("session_" + session);

  const auto qos = rclcpp::SensorDataQoS().keep_last(50);
  for (const auto& name : names) {
    auto agent = std::make_unique<AgentStream>();
    agent->name = name;
    agent->recorder =
        std::make_unique<online::KeyframeRecorder>(live / name, kf);
    const std::string base = prefix + "/" + name + "/";
    AgentStream* raw = agent.get();
    agent->cloud_sub = create_subscription<Cloud2>(
        base + cloud_topic, qos,
        [this, raw](Cloud2::ConstSharedPtr msg) { OnCloud(*raw, std::move(msg)); });
    agent->odom_sub = create_subscription<Odometry>(
        base + odom_topic, qos,
        [this, raw](Odometry::ConstSharedPtr msg) {
          OnOdometry(*raw, std::move(msg));
        });
    const auto latched = rclcpp::QoS(1).transient_local();
    agent->live_points_pub =
        create_publisher<Cloud2>("~/live/" + name + "/points", latched);
    agent->live_path_pub = create_publisher<nav_msgs::msg::Path>(
        "~/live/" + name + "/path", latched);
    agent->live_scan_pub =
        create_publisher<Cloud2>("~/live/" + name + "/scan", rclcpp::QoS(5));
    RCLCPP_INFO(get_logger(), "agent %s: %s%s + %s%s", name.c_str(),
                base.c_str(), cloud_topic.c_str(), base.c_str(),
                odom_topic.c_str());
    agents_.push_back(std::move(agent));
  }

  status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(10).transient_local());
  bridge_ = std::make_unique<RosVisualizationBridge>(*this, runtime_);
  merge_worker_ = std::jthread([this](std::stop_token stop) { MergeLoop(stop); });
  timer_ = create_wall_timer(
      std::chrono::duration<double>(merge_period_s), [this] { OnTimer(); });
  live_timer_ = create_wall_timer(std::chrono::duration<double>(live_period_s),
                                  [this] { PublishLive(); });
  PublishStatus("recording into " + live.string());
}

OpenLMMOnline::~OpenLMMOnline() {
  timer_.reset();
  live_timer_.reset();
  merge_worker_.request_stop();
  merge_wake_.notify_all();
  if (merge_worker_.joinable()) merge_worker_.join();
  subscription_.Reset();
  if (bridge_) bridge_->Stop();
  (void)runtime_->Close();
}

namespace {
double Stamp(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}
}  // namespace

void OpenLMMOnline::OnCloud(AgentStream& agent, Cloud2::ConstSharedPtr cloud) {
  {
    std::lock_guard lock(agent.mutex);
    agent.clouds.push_back(std::move(cloud));
    while (agent.clouds.size() > 20) agent.clouds.pop_front();
  }
  DrainMatches(agent);
}

void OpenLMMOnline::OnOdometry(AgentStream& agent,
                               Odometry::ConstSharedPtr odometry) {
  {
    std::lock_guard lock(agent.mutex);
    agent.odometry.push_back(std::move(odometry));
    while (agent.odometry.size() > 400) agent.odometry.pop_front();
  }
  DrainMatches(agent);
}

void OpenLMMOnline::DrainMatches(AgentStream& agent) {
  for (;;) {
    Cloud2::ConstSharedPtr cloud;
    Odometry::ConstSharedPtr odometry;
    {
      std::lock_guard lock(agent.mutex);
      if (agent.clouds.empty() || agent.odometry.empty()) return;
      const double t = Stamp(agent.clouds.front()->header.stamp);
      // Discard odometry that is strictly older than the oldest cloud.
      while (!agent.odometry.empty() &&
             Stamp(agent.odometry.front()->header.stamp) < t - max_stamp_gap_s_) {
        agent.odometry.pop_front();
      }
      if (agent.odometry.empty()) return;
      const double to = Stamp(agent.odometry.front()->header.stamp);
      if (to > t + max_stamp_gap_s_) {
        agent.clouds.pop_front();  // no odometry for this cloud
        continue;
      }
      cloud = agent.clouds.front();
      odometry = agent.odometry.front();
      agent.clouds.pop_front();
      agent.odometry.pop_front();
    }
    OnMeasurement(agent, *cloud, *odometry);
  }
}

void OpenLMMOnline::OnMeasurement(AgentStream& agent, const Cloud2& cloud,
                                  const Odometry& odometry) {
  online::KeyframeRecorder::Cloud body;
  pcl::fromROSMsg(cloud, body);
  const Eigen::Isometry3d world_T_body = ToIsometry(odometry);
  Eigen::Isometry3d map_T_local;
  {
    std::lock_guard lock(agent.live_mutex);
    agent.live_path.push_back(world_T_body);
    map_T_local = agent.map_T_local;
  }
  if (!body.empty() && agent.live_scan_pub->get_subscription_count() > 0) {
    online::KeyframeRecorder::Cloud scan;
    pcl::transformPointCloud(body, scan,
                             (map_T_local * world_T_body).matrix().cast<float>());
    Cloud2 message;
    pcl::toROSMsg(scan, message);
    message.header.frame_id = live_frame_;
    message.header.stamp = cloud.header.stamp;
    agent.live_scan_pub->publish(message);
  }
  try {
    if (agent.recorder->Add(world_T_body, body)) {
      if (auto keyframe = agent.recorder->LastKeyframeCloud();
          keyframe && !keyframe->empty()) {
        online::KeyframeRecorder::Cloud local;
        pcl::transformPointCloud(*keyframe, local,
                                 world_T_body.matrix().cast<float>());
        const double inv = 1.0 / live_voxel_m_;
        std::lock_guard lock(agent.live_mutex);
        for (const auto& p : local.points) {
          const auto qx = static_cast<std::int64_t>(std::floor(p.x * inv)) & 0x1FFFFF;
          const auto qy = static_cast<std::int64_t>(std::floor(p.y * inv)) & 0x1FFFFF;
          const auto qz = static_cast<std::int64_t>(std::floor(p.z * inv)) & 0x1FFFFF;
          const std::uint64_t key = (static_cast<std::uint64_t>(qx) << 42) |
                                    (static_cast<std::uint64_t>(qy) << 21) |
                                    static_cast<std::uint64_t>(qz);
          if (agent.live_voxels.insert(key).second) agent.live_map.push_back(p);
        }
      }
      const auto count = agent.recorder->KeyframeCount();
      if (count % 50 == 0) {
        RCLCPP_INFO(get_logger(), "agent %s: %zu keyframes",
                    agent.name.c_str(), count);
      }
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "agent %s keyframe write failed: %s",
                 agent.name.c_str(), error.what());
  }
}

void OpenLMMOnline::PublishLive() {
  const auto stamp = now();
  for (auto& agent : agents_) {
    online::KeyframeRecorder::Cloud points;
    nav_msgs::msg::Path path;
    path.header.frame_id = live_frame_;
    path.header.stamp = stamp;
    {
      std::lock_guard lock(agent->live_mutex);
      const Eigen::Isometry3d& T = agent->map_T_local;
      // pcl::transformPointCloud divides by width; skip empty clouds.
      if (!agent->live_map.empty()) {
        pcl::transformPointCloud(agent->live_map, points, T.matrix().cast<float>());
      }
      const std::size_t step = std::max<std::size_t>(1, agent->live_path.size() / 4000);
      path.poses.reserve(agent->live_path.size() / step + 1);
      for (std::size_t i = 0; i < agent->live_path.size(); i += step) {
        const Eigen::Isometry3d pose = T * agent->live_path[i];
        geometry_msgs::msg::PoseStamped out;
        out.header = path.header;
        out.pose.position.x = pose.translation().x();
        out.pose.position.y = pose.translation().y();
        out.pose.position.z = pose.translation().z();
        const Eigen::Quaterniond q(pose.rotation());
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();
        out.pose.orientation.w = q.w();
        path.poses.push_back(std::move(out));
      }
    }
    Cloud2 message;
    pcl::toROSMsg(points, message);
    message.header.frame_id = live_frame_;
    message.header.stamp = stamp;
    agent->live_points_pub->publish(message);
    agent->live_path_pub->publish(path);
  }
}

void OpenLMMOnline::UpdateAlignment(AgentStream& agent, std::size_t keyframes) {
  auto id = AgentId::Parse(agent.name);
  if (!id) return;
  VisualizationQuery query;
  query.agent = id.Value();
  query.include_points = false;
  auto snapshot = runtime_->Visualization(query);
  if (!snapshot || snapshot.Value().poses.empty()) return;
  // Use the latest keyframe the merge saw: merged pose vs recorded pose.
  const VisualizationPose* latest = nullptr;
  for (const auto& pose : snapshot.Value().poses) {
    if (pose.index < 0 || static_cast<std::size_t>(pose.index) >= keyframes) continue;
    if (!latest || pose.index > latest->index) latest = &pose;
  }
  Eigen::Isometry3d local;
  if (!latest || !agent.recorder->KeyframePose(latest->index, local)) return;
  const Eigen::Isometry3d map_T_local = latest->transform.cast<double>() * local.inverse();
  bool first = false;
  {
    std::lock_guard lock(agent.live_mutex);
    first = !agent.aligned;
    agent.map_T_local = map_T_local;
    agent.aligned = true;
  }
  const auto t = map_T_local.translation();
  RCLCPP_INFO(get_logger(), "%s %s: map_T_local t=(%.2f, %.2f, %.2f) yaw=%.1f deg",
              agent.name.c_str(), first ? "aligned" : "realigned", t.x(), t.y(), t.z(),
              std::atan2(map_T_local(1, 0), map_T_local(0, 0)) * 180.0 / M_PI);
}

void OpenLMMOnline::OnTimer() {
  if (merging_.load()) return;
  {
    std::lock_guard lock(merge_mutex_);
    merge_requested_ = true;
  }
  merge_wake_.notify_all();
}

void OpenLMMOnline::MergeLoop(std::stop_token stop) {
  while (!stop.stop_requested()) {
    {
      std::unique_lock lock(merge_mutex_);
      merge_wake_.wait(lock, stop, [this] { return merge_requested_; });
      if (stop.stop_requested()) return;
      merge_requested_ = false;
    }
    merging_ = true;
    try {
      RunMerge();
    } catch (const std::exception& error) {
      PublishStatus(std::string("merge error: ") + error.what());
    }
    merging_ = false;
  }
}

fs::path OpenLMMOnline::WriteSnapshotConfig(
    const fs::path& snapshot_root, const std::vector<std::string>& agents) const {
  const fs::path config_dir = snapshot_root / "config";
  fs::copy(base_config_, config_dir,
           fs::copy_options::recursive | fs::copy_options::overwrite_existing);
  std::ifstream in(config_dir / "config.json");
  std::stringstream buffer;
  buffer << in.rdbuf();
  std::string text = buffer.str();
  in.close();

  std::string list = "[";
  for (std::size_t i = 0; i < agents.size(); ++i) {
    if (i) list += ", ";
    list += JsonString(agents[i]);
  }
  list += "]";
  text = ReplaceOnce(text, std::regex(R"("root_dir_path"\s*:\s*"[^"]*")"),
                     "\"root_dir_path\" : " + JsonString(snapshot_root.string()),
                     "root_dir_path");
  text = ReplaceOnce(text, std::regex(R"("sub_dir_list"\s*:\s*\[[^\]]*\])"),
                     "\"sub_dir_list\" : " + list, "sub_dir_list");
  text = ReplaceOnce(text, std::regex(R"("root_save_dir"\s*:\s*"[^"]*")"),
                     "\"root_save_dir\" : " + JsonString(output_dir_.string()),
                     "root_save_dir");
  std::ofstream out(config_dir / "config.json", std::ios::trunc);
  out << text;
  return config_dir;
}

bool OpenLMMOnline::RunMerge() {
  std::vector<AgentStream*> ready;
  std::size_t fresh = 0;
  std::ostringstream counts;
  for (auto& agent : agents_) {
    const auto count = agent->recorder->KeyframeCount();
    counts << agent->name << '=' << count << ' ';
    if (count >= min_keyframes_per_agent_) {
      ready.push_back(agent.get());
      fresh += count - std::min(count, agent->merged_keyframes);
    }
  }
  if (ready.size() < min_agents_ || fresh < min_new_keyframes_) {
    PublishStatus("waiting: " + counts.str());
    return false;
  }

  const auto started = std::chrono::steady_clock::now();
  std::ostringstream id;
  id << "merge_" << std::setw(4) << std::setfill('0') << ++merge_index_;
  const fs::path root = work_dir_ / "snapshots" / id.str();
  std::vector<std::string> names;
  std::map<AgentStream*, std::size_t> snapped;
  for (auto* agent : ready) {
    snapped[agent] = agent->recorder->Snapshot(root / agent->name);
    names.push_back(agent->name);
  }
  const fs::path config_dir = WriteSnapshotConfig(root, names);
  PublishStatus(id.str() + " started: " + counts.str());

  subscription_.Reset();
  if (auto closed = runtime_->Close(); !closed) {
    PublishStatus("close failed: " + closed.GetError().Message());
    return false;
  }
  auto opened = runtime_->Open(BootstrapRequest{config_dir, "online", output_dir_});
  if (!opened) {
    PublishStatus(id.str() + " open failed: " + opened.GetError().Message());
    return false;
  }
  auto subscribed = runtime_->SubscribeEvents([this](const ExecutionEvent& event) {
    if (event.type != EventType::kJobCompleted || event.error) return;
    ExecutionEvent refresh = event;
    refresh.affected_agents.clear();  // refresh every agent of the new epoch
    bridge_->RequestRefresh(refresh);
  });
  if (subscribed) subscription_ = std::move(subscribed).Value();

  auto job = runtime_->SubmitRunAll();
  if (!job) {
    PublishStatus(id.str() + " submit failed: " + job.GetError().Message());
    return false;
  }
  bridge_->Start();  // no-op after the first merge
  auto waited = runtime_->Wait(job.Value());
  const double seconds = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - started)
                             .count();
  std::string outcome = "succeeded";
  if (!waited) {
    outcome = "failed: " + waited.GetError().Message();
  } else if (auto snapshot = runtime_->Snapshot(); snapshot) {
    const auto& job_state = snapshot.Value().pipeline.job;
    if (job_state && job_state->state != JobState::kSucceeded) {
      outcome = "failed (job state " +
                std::to_string(static_cast<int>(job_state->state)) + ")";
    }
  }
  if (outcome == "succeeded") {
    for (auto& [agent, count] : snapped) {
      agent->merged_keyframes = count;
      UpdateAlignment(*agent, count);
    }
    PublishLive();
  }
  std::ostringstream report;
  report << id.str() << ' ' << outcome << " in " << std::fixed
         << std::setprecision(1) << seconds << " s (" << counts.str() << ")";
  PublishStatus(report.str());

  // Drop old snapshots; the current one stays referenced by the runtime.
  if (keep_snapshots_ > 0 && merge_index_ > static_cast<std::uint64_t>(keep_snapshots_)) {
    std::ostringstream old;
    old << "merge_" << std::setw(4) << std::setfill('0')
        << merge_index_ - keep_snapshots_;
    std::error_code ignored;
    fs::remove_all(work_dir_ / "snapshots" / old.str(), ignored);
  }
  return outcome == "succeeded";
}

void OpenLMMOnline::PublishStatus(const std::string& text) {
  RCLCPP_INFO(get_logger(), "%s", text.c_str());
  std_msgs::msg::String message;
  message.data = text;
  status_pub_->publish(message);
}

}  // namespace open_lmm

RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMOnline)
