#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

// Patchwork++-ROS
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/create_timer_ros.h>

#include "GroundSegmentationServer.hpp"
#include "Utils.hpp"

namespace patchworkpp_ros {

namespace {

inline uint64_t packCell(int32_t ix, int32_t iy) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
         static_cast<uint64_t>(static_cast<uint32_t>(iy));
}

struct CoarseGroundCell {
  double sum_z{0.0};
  uint32_t count{0};
  double mean_z{0.0};
  double plane_a{0.0};
  double plane_b{0.0};
  double plane_c{0.0};
  bool has_plane{false};
};

struct BridgeGroundCell {
  double sum_z{0.0};
  uint32_t count{0};
  double mean_z{0.0};
};

struct VerticalColumn {
  double z_min{std::numeric_limits<double>::max()};
  double z_max{std::numeric_limits<double>::lowest()};
  uint32_t count{0};
};

inline bool solve3x3(const double A[3][3], const double b[3], double x[3]) {
  const double det =
      A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
      A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
      A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (std::abs(det) < 1e-8) {
    return false;
  }
  const double inv_det = 1.0 / det;
  x[0] = inv_det * (b[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                    A[0][1] * (b[1] * A[2][2] - A[1][2] * b[2]) +
                    A[0][2] * (b[1] * A[2][1] - A[1][1] * b[2]));
  x[1] = inv_det * (A[0][0] * (b[1] * A[2][2] - A[1][2] * b[2]) -
                    b[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                    A[0][2] * (A[1][0] * b[2] - b[1] * A[2][0]));
  x[2] = inv_det * (A[0][0] * (A[1][1] * b[2] - b[1] * A[2][1]) -
                    A[0][1] * (A[1][0] * b[2] - b[1] * A[2][0]) +
                    b[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]));
  return std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2]);
}

}  // namespace

// ---------------------------------------------------------------------------
// Parameter loaders
// ---------------------------------------------------------------------------

patchwork::Params GroundSegmentationServer::loadPlusplusParamsFromROS() {
  patchwork::Params params;

  params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
  params.num_iter      = declare_parameter<int>("num_iter", params.num_iter);
  params.num_lpr       = declare_parameter<int>("num_lpr", params.num_lpr);
  params.num_min_pts   = declare_parameter<int>("num_min_pts", params.num_min_pts);
  params.th_seeds      = declare_parameter<double>("th_seeds", params.th_seeds);

  params.th_dist    = declare_parameter<double>("th_dist", params.th_dist);
  params.th_seeds_v = declare_parameter<double>("th_seeds_v", params.th_seeds_v);
  params.th_dist_v  = declare_parameter<double>("th_dist_v", params.th_dist_v);

  params.max_range       = declare_parameter<double>("max_range", params.max_range);
  params.min_range       = declare_parameter<double>("min_range", params.min_range);
  params.uprightness_thr = declare_parameter<double>("uprightness_thr", params.uprightness_thr);

  params.adaptive_seed_selection_margin = declare_parameter<double>(
      "adaptive_seed_selection_margin", params.adaptive_seed_selection_margin);
  params.enable_RVPF = declare_parameter<bool>("enable_RVPF", params.enable_RVPF);
  params.enable_TGR  = declare_parameter<bool>("enable_TGR", params.enable_TGR);

  const std::vector<int64_t> default_sectors(params.num_sectors_each_zone.begin(),
                                             params.num_sectors_each_zone.end());
  const std::vector<int64_t> default_rings(params.num_rings_each_zone.begin(),
                                           params.num_rings_each_zone.end());
  const auto sectors_param =
      declare_parameter<std::vector<int64_t>>("num_sectors_each_zone", default_sectors);
  const auto rings_param =
      declare_parameter<std::vector<int64_t>>("num_rings_each_zone", default_rings);

  if (sectors_param.size() == 4 && rings_param.size() == 4) {
    params.num_sectors_each_zone.clear();
    params.num_rings_each_zone.clear();
    for (size_t i = 0; i < 4; ++i) {
      params.num_sectors_each_zone.push_back(std::max<int>(1, static_cast<int>(sectors_param[i])));
      params.num_rings_each_zone.push_back(std::max<int>(1, static_cast<int>(rings_param[i])));
    }
  } else {
    RCLCPP_WARN(get_logger(),
                "num_sectors_each_zone and num_rings_each_zone must have length 4; using defaults.");
  }

  params.verbose = declare_parameter<bool>("verbose", params.verbose);

  // RNR requires 4th intensity column; keep disabled for XYZ input
  params.enable_RNR = false;

  sensor_height_ = params.sensor_height;
  min_range_     = params.min_range;
  max_range_     = params.max_range;

  return params;
}

patchwork::PatchworkParams GroundSegmentationServer::loadClassicParamsFromROS() {
  patchwork::PatchworkParams params;

  params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
  params.max_range     = declare_parameter<double>("max_range", params.max_range);
  params.min_range     = declare_parameter<double>("min_range", params.min_range);

  params.num_iter    = declare_parameter<int>("num_iter", params.num_iter);
  params.num_lpr     = declare_parameter<int>("num_lpr", params.num_lpr);
  params.num_min_pts = declare_parameter<int>("num_min_pts", params.num_min_pts);
  params.th_seeds    = declare_parameter<double>("th_seeds", params.th_seeds);
  params.th_dist     = declare_parameter<double>("th_dist", params.th_dist);

  params.uprightness_thr = declare_parameter<double>("uprightness_thr", params.uprightness_thr);

  params.adaptive_seed_selection_margin = declare_parameter<double>(
      "adaptive_seed_selection_margin", params.adaptive_seed_selection_margin);

  params.using_global_thr = declare_parameter<bool>("using_global_thr", params.using_global_thr);
  params.global_elevation_thr =
      declare_parameter<double>("global_elevation_thr", params.global_elevation_thr);

  params.ATAT_ON        = declare_parameter<bool>("ATAT_ON", params.ATAT_ON);
  params.max_h_for_ATAT = declare_parameter<double>("max_h_for_ATAT", params.max_h_for_ATAT);
  params.num_sectors_for_ATAT =
      declare_parameter<int>("num_sectors_for_ATAT", params.num_sectors_for_ATAT);
  params.noise_bound = declare_parameter<double>("noise_bound", params.noise_bound);

  params.verbose = declare_parameter<bool>("verbose", params.verbose);

  sensor_height_ = params.sensor_height;
  min_range_     = params.min_range;
  max_range_     = params.max_range;

  return params;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

GroundSegmentationServer::GroundSegmentationServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_node", options) {
  odom_frame_      = declare_parameter<std::string>("odom_frame", odom_frame_);
  base_frame_      = declare_parameter<std::string>("base_frame", base_frame_);
  tf_timeout_      = declare_parameter<double>("tf_timeout", tf_timeout_);
  accum_frames_ =
      std::max(1, static_cast<int>(declare_parameter<int>("accum_frames", accum_frames_)));
  origin_z_offset_ = declare_parameter<double>("origin_z_offset", origin_z_offset_);
  max_rel_z_below_ = declare_parameter<double>("max_rel_z_below", max_rel_z_below_);
  max_rel_z_above_ = declare_parameter<double>("max_rel_z_above", max_rel_z_above_);

  publish_filtered_obstacles_ =
      declare_parameter<bool>("publish_filtered_obstacles", publish_filtered_obstacles_);
  min_clearance_ = declare_parameter<double>("min_clearance", min_clearance_);
  robot_height_  = declare_parameter<double>("robot_height", robot_height_);
  obstacle_max_range_ =
      declare_parameter<double>("obstacle_max_range", obstacle_max_range_);
  obstacle_ground_grid_res_ =
      std::max(0.05, declare_parameter<double>("obstacle_ground_grid_res",
                                               obstacle_ground_grid_res_));
  obstacle_ground_search_radius_ = std::max(
      0,
      static_cast<int>(
          declare_parameter<int>("obstacle_ground_search_radius", obstacle_ground_search_radius_)));
  obstacle_ground_bridge_res_ =
      std::max(0.10, declare_parameter<double>("obstacle_ground_bridge_res",
                                               obstacle_ground_bridge_res_));
  obstacle_ground_bridge_radius_ = std::max(
      0,
      static_cast<int>(
          declare_parameter<int>("obstacle_ground_bridge_radius", obstacle_ground_bridge_radius_)));
  near_unknown_ground_range_ =
      std::max(0.0, declare_parameter<double>("near_unknown_ground_range",
                                              near_unknown_ground_range_));
  min_unknown_ground_z_span_ =
      std::max(0.0, declare_parameter<double>("min_unknown_ground_z_span",
                                              min_unknown_ground_z_span_));

  const std::string algorithm = declare_parameter<std::string>("algorithm", "patchworkpp");

  if (algorithm == "patchwork") {
    patchwork::PatchworkParams classic_params = loadClassicParamsFromROS();
    impl_ = std::make_unique<patchwork::PatchWork>(classic_params);
    RCLCPP_INFO(get_logger(), "Algorithm: patchwork (classic)");
  } else {
    patchwork::Params plusplus_params = loadPlusplusParamsFromROS();
    impl_                             = std::make_unique<patchwork::PatchWorkpp>(plusplus_params);
    RCLCPP_INFO(get_logger(), "Algorithm: patchworkpp (default)");
  }

  // TF2 buffer & listener attached to this node (honors namespace & /tf remappings)
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_topic",
      rclcpp::SensorDataQoS(),
      std::bind(&GroundSegmentationServer::EstimateGround, this, std::placeholders::_1));

  // Real-time volatile QoS compatible with SensorDataQoS() and RViz2 defaults.
  // Relative topic names allow PushRosNamespace to scope multi-robot topics cleanly.
  rclcpp::QoS pub_qos(rclcpp::KeepLast(5));
  pub_qos.reliable().durability_volatile();

  cloud_publisher_     = create_publisher<sensor_msgs::msg::PointCloud2>("patchworkpp/cloud", pub_qos);
  ground_publisher_    = create_publisher<sensor_msgs::msg::PointCloud2>("patchworkpp/ground", pub_qos);
  nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("patchworkpp/nonground", pub_qos);
  obstacle_publisher_  = create_publisher<sensor_msgs::msg::PointCloud2>("patchworkpp/obstacles", pub_qos);

  RCLCPP_INFO(this->get_logger(),
              "Patchwork++ ROS 2 node initialized (odom_frame=%s, base_frame=%s, "
              "origin_z_offset=%.3fm, range=[%.2f, %.2f]m, sensor_height=%.3fm, "
              "accum_frames=%d, clearance=[%.2f, %.2f]m)",
              odom_frame_.c_str(),
              base_frame_.c_str(),
              origin_z_offset_,
              min_range_,
              max_range_,
              sensor_height_,
              accum_frames_,
              min_clearance_,
              robot_height_);
}

// ---------------------------------------------------------------------------
// Callbacks & Clearance Filtering
// ---------------------------------------------------------------------------

void GroundSegmentationServer::EstimateGround(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  if (!msg || msg->data.empty() || msg->width * msg->height == 0) {
    return;
  }

  // 1. Lookup robot position in gravity-aligned odom_frame_
  geometry_msgs::msg::TransformStamped tf_odom_base;
  try {
    tf_odom_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(tf_timeout_));
  } catch (const tf2::TransformException &) {
    try {
      tf_odom_base = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "TF lookup %s -> %s failed: %s",
                           odom_frame_.c_str(),
                           base_frame_.c_str(),
                           ex.what());
      return;
    }
  }

  const Eigen::Vector3f t_base(static_cast<float>(tf_odom_base.transform.translation.x),
                               static_cast<float>(tf_odom_base.transform.translation.y),
                               static_cast<float>(tf_odom_base.transform.translation.z));
  // Virtual gravity-aligned sensor origin above base_link so ground sits at z ~ -sensor_height
  // (required by Patchwork++ heading-outside check and adaptive seed margin).
  const Eigen::Vector3f t_center(
      t_base.x(), t_base.y(), t_base.z() + static_cast<float>(origin_z_offset_));

  // 2. Transform incoming scan to odom_frame_ if needed
  const std::string src_frame = patchworkpp_ros::utils::FixFrameId(msg->header.frame_id);
  bool need_transform         = (src_frame != odom_frame_);
  tf2::Matrix3x3 rot_odom_src;
  tf2::Vector3 trans_odom_src(0.0, 0.0, 0.0);

  if (need_transform) {
    geometry_msgs::msg::TransformStamped tf_odom_src;
    try {
      tf_odom_src = tf_buffer_->lookupTransform(
          odom_frame_, src_frame, msg->header.stamp, rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException &) {
      try {
        tf_odom_src = tf_buffer_->lookupTransform(odom_frame_, src_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             2000,
                             "TF lookup %s -> %s failed: %s",
                             odom_frame_.c_str(),
                             src_frame.c_str(),
                             ex.what());
        return;
      }
    }
    const auto &q = tf_odom_src.transform.rotation;
    const auto &t = tf_odom_src.transform.translation;
    rot_odom_src.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
    trans_odom_src.setValue(t.x, t.y, t.z);
  }

  // Extract finite points into odom_frame_
  const size_t n_in = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  std::vector<Eigen::Vector3f> current_frame;
  current_frame.reserve(n_in);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (size_t i = 0; i < n_in; ++i, ++iter_x, ++iter_y, ++iter_z) {
    const float px = *iter_x;
    const float py = *iter_y;
    const float pz = *iter_z;
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    if (need_transform) {
      const tf2::Vector3 gp = rot_odom_src * tf2::Vector3(px, py, pz) + trans_odom_src;
      current_frame.emplace_back(static_cast<float>(gp.x()),
                                 static_cast<float>(gp.y()),
                                 static_cast<float>(gp.z()));
    } else {
      current_frame.emplace_back(px, py, pz);
    }
  }

  // 3. Maintain sliding window in odom_frame_
  frame_buffer_.push_back(std::move(current_frame));
  while (static_cast<int>(frame_buffer_.size()) > accum_frames_) {
    frame_buffer_.pop_front();
  }

  // 4. Center accumulated points around current robot position (preserving gravity +Z)
  //    and pre-filter self-blind-zone (< min_range) and out-of-range (> max_range) points
  //    so PatchWorkpp::pc2czm does not dump them into cloud_nonground_.
  const float min_r_sq = static_cast<float>(min_range_ * min_range_);
  const float max_r_sq = static_cast<float>(max_range_ * max_range_);
  const float z_lo     = static_cast<float>(max_rel_z_below_);
  const float z_hi     = static_cast<float>(max_rel_z_above_);

  std::vector<Eigen::Vector3f> local_pts;
  size_t total_est = 0;
  for (const auto &f : frame_buffer_) {
    total_est += f.size();
  }
  local_pts.reserve(total_est);

  for (const auto &f : frame_buffer_) {
    for (const auto &p : f) {
      const float rel_z_base = p.z() - t_base.z();
      if (rel_z_base < z_lo || rel_z_base > z_hi) {
        continue;
      }
      const Eigen::Vector3f lp = p - t_center;
      const float r_sq         = lp.x() * lp.x() + lp.y() * lp.y();
      if (r_sq <= min_r_sq || r_sq >= max_r_sq) {
        continue;
      }
      local_pts.push_back(lp);
    }
  }

  std_msgs::msg::Header out_header = msg->header;
  out_header.frame_id              = odom_frame_;

  if (local_pts.empty()) {
    Eigen::MatrixX3f empty3(0, 3);
    Eigen::MatrixX4f empty4(0, 4);
    PublishClouds(empty3, empty3, empty4, out_header);
    return;
  }

  Eigen::MatrixXf local_cloud(static_cast<Eigen::Index>(local_pts.size()), 3);
  for (size_t i = 0; i < local_pts.size(); ++i) {
    local_cloud.row(static_cast<Eigen::Index>(i)) = local_pts[i].transpose();
  }

  // 5. Run Patchwork++ / Patchwork ground segmentation in robot-centered gravity frame
  std::visit([&](auto &impl) { impl->estimateGround(local_cloud); }, impl_);

  Eigen::MatrixX3f local_ground =
      std::visit([](auto &impl) { return impl->getGround(); }, impl_);
  Eigen::MatrixX3f local_nonground =
      std::visit([](auto &impl) { return impl->getNonground(); }, impl_);

  // 6. Optional clearance-filtered obstacle cloud (with height-above-ground as intensity)
  Eigen::MatrixX4f local_obstacles(0, 4);
  if (publish_filtered_obstacles_ && obstacle_publisher_->get_subscription_count() > 0) {
    local_obstacles = FilterClearanceObstacles(local_ground, local_nonground);
  }

  // 7. Shift back to odom_frame_ and publish
  if (cloud_publisher_->get_subscription_count() > 0) {
    Eigen::MatrixX3f odom_cloud = local_cloud.rowwise() + t_center.transpose();
    cloud_publisher_->publish(patchworkpp_ros::utils::EigenMatToPointCloud2(odom_cloud, out_header));
  }

  if (local_ground.rows() > 0) {
    local_ground.rowwise() += t_center.transpose();
  }
  if (local_nonground.rows() > 0) {
    local_nonground.rowwise() += t_center.transpose();
  }
  if (local_obstacles.rows() > 0) {
    local_obstacles.leftCols<3>().rowwise() += t_center.transpose();
  }

  PublishClouds(local_ground, local_nonground, local_obstacles, out_header);
}

Eigen::MatrixX4f GroundSegmentationServer::FilterClearanceObstacles(
    const Eigen::MatrixX3f &local_ground, const Eigen::MatrixX3f &local_nonground) const {
  if (local_nonground.rows() == 0) {
    return Eigen::MatrixX4f(0, 4);
  }

  const double res            = obstacle_ground_grid_res_;
  const double inv_res        = 1.0 / res;
  const double bridge_res     = obstacle_ground_bridge_res_;
  const double inv_bridge_res = 1.0 / bridge_res;
  const double obs_max_r_sq   = obstacle_max_range_ * obstacle_max_range_;
  const bool use_obs_range    = (obstacle_max_range_ > 0.0);
  const double near_r_sq      = near_unknown_ground_range_ * near_unknown_ground_range_;

  // Stage 1 (fine) & Stage 2 (coarse bridge) ground grids
  std::unordered_map<uint64_t, CoarseGroundCell> grid;
  std::unordered_map<uint64_t, BridgeGroundCell> bridge_grid;
  grid.reserve(static_cast<size_t>(local_ground.rows() / 4 + 64));
  bridge_grid.reserve(static_cast<size_t>(local_ground.rows() / 12 + 64));

  for (Eigen::Index i = 0; i < local_ground.rows(); ++i) {
    const double gx = static_cast<double>(local_ground(i, 0));
    const double gy = static_cast<double>(local_ground(i, 1));
    const double gz = static_cast<double>(local_ground(i, 2));

    const int32_t ix = static_cast<int32_t>(std::floor(gx * inv_res));
    const int32_t iy = static_cast<int32_t>(std::floor(gy * inv_res));
    auto &cell       = grid[packCell(ix, iy)];
    cell.sum_z += gz;
    cell.count += 1u;

    const int32_t bx = static_cast<int32_t>(std::floor(gx * inv_bridge_res));
    const int32_t by = static_cast<int32_t>(std::floor(gy * inv_bridge_res));
    auto &bcell      = bridge_grid[packCell(bx, by)];
    bcell.sum_z += gz;
    bcell.count += 1u;
  }

  for (auto &kv : grid) {
    kv.second.mean_z = kv.second.sum_z / static_cast<double>(kv.second.count);
  }
  for (auto &kv : bridge_grid) {
    kv.second.mean_z = kv.second.sum_z / static_cast<double>(kv.second.count);
  }

  // Fit local 3x3 tilted plane z = a*(x-cx) + b*(y-cy) + c for each fine ground cell
  constexpr double kMaxSlopeTan = 0.65;  // ~33 deg safety bound on plane fit
  const double max_z_jump       = res * 1.5 * kMaxSlopeTan + 0.12;

  for (auto &kv : grid) {
    const int32_t ix = static_cast<int32_t>(static_cast<uint32_t>(kv.first >> 32));
    const int32_t iy = static_cast<int32_t>(static_cast<uint32_t>(kv.first & 0xFFFFFFFFu));
    auto &center     = kv.second;
    center.plane_c   = center.mean_z;

    double suu = 0.0, suv = 0.0, svv = 0.0, su = 0.0, sv = 0.0, sw = 0.0;
    double suz = 0.0, svz = 0.0, sz = 0.0;
    int valid_neighbors = 0;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = grid.find(packCell(ix + dx, iy + dy));
        if (it == grid.end()) {
          continue;
        }
        const double nz = it->second.mean_z;
        if (std::abs(nz - center.mean_z) > max_z_jump) {
          continue;
        }
        const double u = static_cast<double>(dx) * res;
        const double v = static_cast<double>(dy) * res;
        const double w = std::min(10.0, static_cast<double>(it->second.count));
        suu += w * u * u;
        suv += w * u * v;
        svv += w * v * v;
        su += w * u;
        sv += w * v;
        sw += w;
        suz += w * u * nz;
        svz += w * v * nz;
        sz += w * nz;
        valid_neighbors++;
      }
    }

    if (valid_neighbors >= 3) {
      const double A[3][3] = {{suu, suv, su}, {suv, svv, sv}, {su, sv, sw}};
      const double b[3]    = {suz, svz, sz};
      double sol[3]        = {0.0, 0.0, center.mean_z};
      if (solve3x3(A, b, sol) && std::hypot(sol[0], sol[1]) <= kMaxSlopeTan) {
        center.plane_a   = sol[0];
        center.plane_b   = sol[1];
        center.plane_c   = sol[2];
        center.has_plane = true;
      }
    }
  }

  // Build 0.20 m vertical column profile over nonground points to distinguish genuine vertical
  // obstacles (walls/pillars) from thin misclassified far ground sheets or thin floating beams.
  constexpr double kColRes    = 0.20;
  constexpr double kInvColRes = 1.0 / kColRes;
  std::unordered_map<uint64_t, VerticalColumn> cols;
  cols.reserve(static_cast<size_t>(local_nonground.rows() / 3 + 64));

  for (Eigen::Index i = 0; i < local_nonground.rows(); ++i) {
    const double px = static_cast<double>(local_nonground(i, 0));
    const double py = static_cast<double>(local_nonground(i, 1));
    if (use_obs_range && (px * px + py * py) > obs_max_r_sq) {
      continue;
    }
    const double pz  = static_cast<double>(local_nonground(i, 2));
    const int32_t cx = static_cast<int32_t>(std::floor(px * kInvColRes));
    const int32_t cy = static_cast<int32_t>(std::floor(py * kInvColRes));
    auto &col        = cols[packCell(cx, cy)];
    col.z_min        = std::min(col.z_min, pz);
    col.z_max        = std::max(col.z_max, pz);
    col.count += 1u;
  }

  // Classify each nonground point via 3-tier ground estimation + vertical structure guard
  std::vector<Eigen::Vector4f> kept;
  kept.reserve(static_cast<size_t>(local_nonground.rows()));
  const int search_r = obstacle_ground_search_radius_;
  const int bridge_r = obstacle_ground_bridge_radius_;

  for (Eigen::Index i = 0; i < local_nonground.rows(); ++i) {
    const double px   = static_cast<double>(local_nonground(i, 0));
    const double py   = static_cast<double>(local_nonground(i, 1));
    const double r_sq = px * px + py * py;
    if (use_obs_range && r_sq > obs_max_r_sq) {
      continue;
    }
    const double pz  = static_cast<double>(local_nonground(i, 2));
    const int32_t ix = static_cast<int32_t>(std::floor(px * inv_res));
    const int32_t iy = static_cast<int32_t>(std::floor(py * inv_res));

    double ground_z   = -sensor_height_;
    bool ground_found = false;
    bool from_fine    = false;

    auto evalPatch = [&](int32_t gx, int32_t gy, const CoarseGroundCell &c) -> double {
      if (!c.has_plane) {
        return c.mean_z;
      }
      const double cx = (static_cast<double>(gx) + 0.5) * res;
      const double cy = (static_cast<double>(gy) + 0.5) * res;
      return c.plane_a * (px - cx) + c.plane_b * (py - cy) + c.plane_c;
    };

    // Tier 1: Fine local tilted ground plane
    auto it = grid.find(packCell(ix, iy));
    if (it != grid.end()) {
      ground_z     = evalPatch(ix, iy, it->second);
      ground_found = true;
      from_fine    = true;
    } else if (search_r > 0) {
      double weighted_z = 0.0;
      double total_w    = 0.0;
      for (int dx = -search_r; dx <= search_r; ++dx) {
        for (int dy = -search_r; dy <= search_r; ++dy) {
          if (dx == 0 && dy == 0) continue;
          auto nit = grid.find(packCell(ix + dx, iy + dy));
          if (nit == grid.end()) continue;
          const double d2 = static_cast<double>(dx * dx + dy * dy);
          const double w  = 1.0 / d2;
          weighted_z += w * evalPatch(ix + dx, iy + dy, nit->second);
          total_w += w;
        }
      }
      if (total_w > 0.0) {
        ground_z     = weighted_z / total_w;
        ground_found = true;
        from_fine    = true;
      }
    }

    // Tier 2: Coarse ground bridging across unobserved tunnel floor / sparse far sectors
    if (!ground_found && bridge_r > 0 && !bridge_grid.empty()) {
      const int32_t bx = static_cast<int32_t>(std::floor(px * inv_bridge_res));
      const int32_t by = static_cast<int32_t>(std::floor(py * inv_bridge_res));
      auto bit         = bridge_grid.find(packCell(bx, by));
      if (bit != bridge_grid.end()) {
        ground_z     = bit->second.mean_z;
        ground_found = true;
      } else {
        double weighted_z = 0.0;
        double total_w    = 0.0;
        for (int dx = -bridge_r; dx <= bridge_r; ++dx) {
          for (int dy = -bridge_r; dy <= bridge_r; ++dy) {
            if (dx == 0 && dy == 0) continue;
            auto nit = bridge_grid.find(packCell(bx + dx, by + dy));
            if (nit == bridge_grid.end()) continue;
            const double d2 = static_cast<double>(dx * dx + dy * dy);
            const double w  = 1.0 / d2;
            weighted_z += w * nit->second.mean_z;
            total_w += w;
          }
        }
        if (total_w > 0.0) {
          ground_z     = weighted_z / total_w;
          ground_found = true;
        }
      }
    }

    auto hasVerticalStructure = [&]() -> bool {
      const int32_t cx = static_cast<int32_t>(std::floor(px * kInvColRes));
      const int32_t cy = static_cast<int32_t>(std::floor(py * kInvColRes));
      auto cit         = cols.find(packCell(cx, cy));
      return (cit != cols.end() && cit->second.count >= 3u &&
              (cit->second.z_max - cit->second.z_min) >= min_unknown_ground_z_span_);
    };

    double out_h   = 0.0;
    bool keep_pt   = false;
    const bool is_near = (r_sq <= near_r_sq);

    if (from_fine) {
      out_h   = pz - ground_z;
      keep_pt = (out_h >= min_clearance_ && out_h <= robot_height_);
    } else if (ground_found) {
      // Bridged ground: overhead tunnel roofs/bars (h > robot_height_) and flat ground
      // (h < min_clearance_) are immediately rejected. At far range, also require a vertical
      // structure so uphill far ramps bridged from downhill ground are never marked as obstacles.
      out_h = pz - ground_z;
      if (out_h >= min_clearance_ && out_h <= robot_height_) {
        keep_pt = is_near || hasVerticalStructure();
      }
    } else if (is_near) {
      // Near robot with occluded floor: fallback to robot feet height, strictly capped at robot_height_
      out_h   = pz - (-sensor_height_);
      keep_pt = (out_h >= min_clearance_ && out_h <= robot_height_);
    } else {
      // Far with no ground within bridge radius: never compare against robot feet height;
      // keep only genuine vertical structures (walls/pillars), rejecting thin far ground sheets.
      const int32_t cx = static_cast<int32_t>(std::floor(px * kInvColRes));
      const int32_t cy = static_cast<int32_t>(std::floor(py * kInvColRes));
      auto cit         = cols.find(packCell(cx, cy));
      if (cit != cols.end() && cit->second.count >= 3u &&
          (cit->second.z_max - cit->second.z_min) >= min_unknown_ground_z_span_) {
        out_h   = pz - cit->second.z_min;
        keep_pt = (out_h >= min_clearance_ && out_h <= robot_height_);
      }
    }

    if (keep_pt) {
      kept.emplace_back(static_cast<float>(px),
                        static_cast<float>(py),
                        static_cast<float>(pz),
                        static_cast<float>(out_h));
    }
  }

  Eigen::MatrixX4f out(static_cast<Eigen::Index>(kept.size()), 4);
  for (size_t i = 0; i < kept.size(); ++i) {
    out.row(static_cast<Eigen::Index>(i)) = kept[i].transpose();
  }
  return out;
}

void GroundSegmentationServer::PublishClouds(const Eigen::MatrixX3f &est_ground,
                                             const Eigen::MatrixX3f &est_nonground,
                                             const Eigen::MatrixX4f &filtered_obstacles,
                                             const std_msgs::msg::Header &header_msg) {
  ground_publisher_->publish(
      patchworkpp_ros::utils::EigenMatToPointCloud2(est_ground, header_msg));
  nonground_publisher_->publish(
      patchworkpp_ros::utils::EigenMatToPointCloud2(est_nonground, header_msg));
  if (publish_filtered_obstacles_ && obstacle_publisher_->get_subscription_count() > 0) {
    obstacle_publisher_->publish(
        patchworkpp_ros::utils::EigenMatToPointCloud2WithIntensity(filtered_obstacles, header_msg));
  }
}

}  // namespace patchworkpp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(patchworkpp_ros::GroundSegmentationServer)
