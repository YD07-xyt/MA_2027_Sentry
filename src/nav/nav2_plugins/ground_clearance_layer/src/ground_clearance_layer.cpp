#include "ground_clearance_layer/ground_clearance_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ground_clearance_layer
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kMaxPendingNongroundPts = 120000;

inline std::string stripLeadingSlash(const std::string & frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

inline bool lookupTransformWithFallback(
  tf2_ros::Buffer * tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  double timeout_sec,
  geometry_msgs::msg::TransformStamped & out_tf)
{
  if (!tf_buffer) {
    return false;
  }
  try {
    out_tf = tf_buffer->lookupTransform(
      target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(timeout_sec));
    return true;
  } catch (const tf2::TransformException &) {
    try {
      out_tf = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
}

inline bool solve3x3(const double A[3][3], const double b[3], double x[3])
{
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

void GroundClearanceLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("GroundClearanceLayer: node expired");
  }

  clock_ = node->get_clock();
  logger_ = node->get_logger();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("ground_points_topic", rclcpp::ParameterValue(std::string("/patchworkpp/ground")));
  declareParameter("nonground_points_topic", rclcpp::ParameterValue(std::string("/patchworkpp/nonground")));
  declareParameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declareParameter("tf_timeout", rclcpp::ParameterValue(0.1));
  declareParameter("combination_method", rclcpp::ParameterValue(1));

  declareParameter("ground_inc", rclcpp::ParameterValue(1.0));
  declareParameter("obstacle_inc", rclcpp::ParameterValue(1.5));
  declareParameter("ground_decay_tau", rclcpp::ParameterValue(0.5));
  declareParameter("obstacle_decay_tau", rclcpp::ParameterValue(0.8));
  declareParameter("obstacle_occ_thresh", rclcpp::ParameterValue(2.0));
  declareParameter("obstacle_prob_thresh", rclcpp::ParameterValue(0.65));
  declareParameter("max_score", rclcpp::ParameterValue(500.0));
  declareParameter("discretize_costs", rclcpp::ParameterValue(true));

  declareParameter("min_clearance", rclcpp::ParameterValue(0.10));
  declareParameter("robot_height", rclcpp::ParameterValue(0.70));
  declareParameter("robot_base_height", rclcpp::ParameterValue(0.0));
  declareParameter("unknown_ground_is_lethal", rclcpp::ParameterValue(true));
  declareParameter("near_unknown_ground_range", rclcpp::ParameterValue(2.5));
  declareParameter("min_unknown_ground_z_span", rclcpp::ParameterValue(0.22));

  declareParameter("ground_patch_res", rclcpp::ParameterValue(0.30));
  declareParameter("ground_neighbor_search_cells", rclcpp::ParameterValue(2));
  declareParameter("ground_bridge_res", rclcpp::ParameterValue(0.60));
  declareParameter("ground_bridge_search_cells", rclcpp::ParameterValue(4));
  declareParameter("ground_height_ema_alpha", rclcpp::ParameterValue(0.35));
  declareParameter("ground_patch_decay_tau", rclcpp::ParameterValue(3.0));
  declareParameter("max_plane_slope_deg", rclcpp::ParameterValue(32.0));

  declareParameter("max_data_range", rclcpp::ParameterValue(6.0));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));

  auto getFloat = [&](const std::string & param, float & out) {
    double tmp = 0.0;
    node->get_parameter(name_ + "." + param, tmp);
    out = static_cast<float>(tmp);
  };

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".ground_points_topic", ground_topic_);
  node->get_parameter(name_ + ".nonground_points_topic", nonground_topic_);
  node->get_parameter(name_ + ".robot_base_frame", robot_base_frame_);
  node->get_parameter(name_ + ".tf_timeout", tf_timeout_);
  node->get_parameter(name_ + ".combination_method", combination_method_);

  getFloat("ground_inc", ground_inc_);
  getFloat("obstacle_inc", obstacle_inc_);
  node->get_parameter(name_ + ".ground_decay_tau", ground_decay_tau_);
  node->get_parameter(name_ + ".obstacle_decay_tau", obstacle_decay_tau_);
  getFloat("obstacle_occ_thresh", obstacle_occ_thresh_);
  getFloat("obstacle_prob_thresh", obstacle_prob_thresh_);
  node->get_parameter(name_ + ".max_score", max_score_);
  node->get_parameter(name_ + ".discretize_costs", discretize_costs_);

  node->get_parameter(name_ + ".min_clearance", min_clearance_);
  node->get_parameter(name_ + ".robot_height", robot_height_);
  node->get_parameter(name_ + ".robot_base_height", robot_base_height_);
  node->get_parameter(name_ + ".unknown_ground_is_lethal", unknown_ground_is_lethal_);
  node->get_parameter(name_ + ".near_unknown_ground_range", near_unknown_ground_range_);
  node->get_parameter(name_ + ".min_unknown_ground_z_span", min_unknown_ground_z_span_);

  node->get_parameter(name_ + ".ground_patch_res", ground_patch_res_);
  node->get_parameter(name_ + ".ground_neighbor_search_cells", ground_neighbor_search_cells_);
  node->get_parameter(name_ + ".ground_bridge_res", ground_bridge_res_);
  node->get_parameter(name_ + ".ground_bridge_search_cells", ground_bridge_search_cells_);
  getFloat("ground_height_ema_alpha", ground_height_ema_alpha_);
  node->get_parameter(name_ + ".ground_patch_decay_tau", ground_patch_decay_tau_);
  node->get_parameter(name_ + ".max_plane_slope_deg", max_plane_slope_deg_);

  node->get_parameter(name_ + ".max_data_range", max_data_range_);
  node->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);

  ground_decay_tau_ = std::max(0.05, ground_decay_tau_);
  obstacle_decay_tau_ = std::max(0.05, obstacle_decay_tau_);
  ground_patch_decay_tau_ = std::max(0.1, ground_patch_decay_tau_);
  ground_patch_res_ = std::max(0.05, ground_patch_res_);
  ground_neighbor_search_cells_ = std::max(0, ground_neighbor_search_cells_);
  ground_bridge_res_ = std::max(0.10, ground_bridge_res_);
  ground_bridge_search_cells_ = std::max(0, ground_bridge_search_cells_);
  near_unknown_ground_range_ = std::max(0.0, near_unknown_ground_range_);
  min_unknown_ground_z_span_ = std::max(0.0, min_unknown_ground_z_span_);
  ground_height_ema_alpha_ = std::clamp(ground_height_ema_alpha_, 0.05f, 1.0f);
  max_plane_slope_deg_ = std::clamp(max_plane_slope_deg_, 5.0, 60.0);
  max_plane_slope_tan_ = std::tan(max_plane_slope_deg_ * kPi / 180.0);

  global_frame_ = stripLeadingSlash(layered_costmap_->getGlobalFrameID());
  robot_base_frame_ = stripLeadingSlash(robot_base_frame_);

  matchSize();

  RCLCPP_INFO(
    logger_,
    "GroundClearanceLayer initialized: global_frame=%s, ground=%s, nonground=%s, "
    "clearance=[%.2f, %.2f]m, range=%.2fm, comb=%d",
    global_frame_.c_str(), ground_topic_.c_str(), nonground_topic_.c_str(),
    min_clearance_, robot_height_, max_data_range_, combination_method_);

  current_ = true;
}

void GroundClearanceLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "GroundClearanceLayer::activate() - node expired");
    return;
  }

  rclcpp::SubscriptionOptions sub_opts;
  if (callback_group_) {
    sub_opts.callback_group = callback_group_;
  }

  auto qos = rclcpp::SensorDataQoS();
  ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    ground_topic_, qos,
    std::bind(&GroundClearanceLayer::groundCloudCallback, this, std::placeholders::_1),
    sub_opts);

  nonground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    nonground_topic_, qos,
    std::bind(&GroundClearanceLayer::nongroundCloudCallback, this, std::placeholders::_1),
    sub_opts);

  RCLCPP_INFO(
    logger_, "GroundClearanceLayer activated: sub to %s and %s",
    ground_topic_.c_str(), nonground_topic_.c_str());
}

void GroundClearanceLayer::deactivate()
{
  ground_sub_.reset();
  nonground_sub_.reset();
}

void GroundClearanceLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
    master->getOriginX(), master->getOriginY());
}

void GroundClearanceLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cells_.clear();
  ground_patches_.clear();
  ground_bridge_patches_.clear();
  pending_nonground_pts_.clear();
  have_new_data_ = false;
  have_new_ground_ = false;
  has_decay_time_ = false;
}

void GroundClearanceLayer::clearArea(
  int start_x, int start_y, int end_x, int end_y, bool invert)
{
  CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert);
  std::lock_guard<std::mutex> lock(mutex_);
  const nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  const double res = master->getResolution();

  for (auto it = cells_.begin(); it != cells_.end(); ) {
    const double wx = (static_cast<double>(unpackX(it->first)) + 0.5) * res;
    const double wy = (static_cast<double>(unpackY(it->first)) + 0.5) * res;
    unsigned int mx = 0;
    unsigned int my = 0;
    const bool in_map = master->worldToMap(wx, wy, mx, my);
    const bool inside = in_map &&
      static_cast<int>(mx) >= start_x && static_cast<int>(mx) <= end_x &&
      static_cast<int>(my) >= start_y && static_cast<int>(my) <= end_y;
    if ((!invert && inside) || (invert && !inside)) {
      it = cells_.erase(it);
    } else {
      ++it;
    }
  }
}

void GroundClearanceLayer::groundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enabled_ || !msg || msg->data.empty() || msg->point_step == 0) {
    return;
  }

  const std::string src_frame = stripLeadingSlash(msg->header.frame_id);
  const bool need_tf = (src_frame != global_frame_);
  tf2::Transform transform;
  transform.setIdentity();

  if (need_tf) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    if (!lookupTransformWithFallback(
          tf_, global_frame_, src_frame, msg->header.stamp, tf_timeout_, tf_stamped))
    {
      RCLCPP_DEBUG(
        logger_, "GroundClearanceLayer: TF lookup failed for ground cloud (%s -> %s)",
        src_frame.c_str(), global_frame_.c_str());
      return;
    }
    tf2::fromMsg(tf_stamped.transform, transform);
  }

  const double res = getResolution();
  const double patch_res = ground_patch_res_;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    std::lock_guard<std::mutex> lock(mutex_);
    const bool use_range = (max_data_range_ > 0.0 && has_robot_pose_);
    const double range_pad = max_data_range_ + std::max(
      patch_res * (ground_neighbor_search_cells_ + 2),
      ground_bridge_res_ * 2.0);
    const double range_pad_sq = range_pad * range_pad;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float rx = *iter_x;
      const float ry = *iter_y;
      const float rz = *iter_z;
      if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)) {
        continue;
      }

      double gx = rx;
      double gy = ry;
      double gz = rz;
      if (need_tf) {
        const tf2::Vector3 gp = transform(tf2::Vector3(rx, ry, rz));
        gx = gp.x();
        gy = gp.y();
        gz = gp.z();
      }

      if (use_range) {
        const double dx = gx - last_robot_x_;
        const double dy = gy - last_robot_y_;
        if (dx * dx + dy * dy > range_pad_sq) {
          continue;
        }
      }

      auto & cell = cells_[worldKey(gx, gy, res)];
      cell.ground_count_frame += 1u;
      cell.ground_z_sum_frame += gz;

      auto & patch = ground_patches_[worldKey(gx, gy, patch_res)];
      patch.count_frame += 1u;
      patch.z_sum_frame += gz;
    }
    have_new_data_ = true;
    have_new_ground_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "GroundClearanceLayer: failed to process ground cloud: %s", e.what());
  }
}

void GroundClearanceLayer::nongroundCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enabled_ || !msg || msg->data.empty() || msg->point_step == 0) {
    return;
  }

  const std::string src_frame = stripLeadingSlash(msg->header.frame_id);
  const bool need_tf = (src_frame != global_frame_);
  tf2::Transform transform;
  transform.setIdentity();

  if (need_tf) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    if (!lookupTransformWithFallback(
          tf_, global_frame_, src_frame, msg->header.stamp, tf_timeout_, tf_stamped))
    {
      RCLCPP_DEBUG(
        logger_, "GroundClearanceLayer: TF lookup failed for nonground cloud (%s -> %s)",
        src_frame.c_str(), global_frame_.c_str());
      return;
    }
    tf2::fromMsg(tf_stamped.transform, transform);
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    std::lock_guard<std::mutex> lock(mutex_);
    const bool use_range = (max_data_range_ > 0.0 && has_robot_pose_);
    const double max_range_sq = max_data_range_ * max_data_range_;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (pending_nonground_pts_.size() >= kMaxPendingNongroundPts) {
        break;
      }

      const float rx = *iter_x;
      const float ry = *iter_y;
      const float rz = *iter_z;
      if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)) {
        continue;
      }

      double gx = rx;
      double gy = ry;
      double gz = rz;
      if (need_tf) {
        const tf2::Vector3 gp = transform(tf2::Vector3(rx, ry, rz));
        gx = gp.x();
        gy = gp.y();
        gz = gp.z();
      }

      if (use_range) {
        const double dx = gx - last_robot_x_;
        const double dy = gy - last_robot_y_;
        if (dx * dx + dy * dy > max_range_sq) {
          continue;
        }
      }

      pending_nonground_pts_.push_back(
        Point3f{static_cast<float>(gx), static_cast<float>(gy), static_cast<float>(gz)});
    }
    have_new_data_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 2000,
      "GroundClearanceLayer: failed to process nonground cloud: %s", e.what());
  }
}

void GroundClearanceLayer::updateGroundPatches(
  double robot_x, double robot_y,
  double win_min_x, double win_min_y, double win_max_x, double win_max_y,
  float patch_decay)
{
  const double patch_res = ground_patch_res_;
  const double margin = std::max(
    patch_res * (ground_neighbor_search_cells_ + 2),
    ground_bridge_res_ * 2.0);
  const bool use_range = (max_data_range_ > 0.0);
  const double max_r_sq = (max_data_range_ + margin) * (max_data_range_ + margin);

  for (auto it = ground_patches_.begin(); it != ground_patches_.end(); ) {
    auto & patch = it->second;
    const double wx = (static_cast<double>(unpackX(it->first)) + 0.5) * patch_res;
    const double wy = (static_cast<double>(unpackY(it->first)) + 0.5) * patch_res;

    if (use_range) {
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      if (dx * dx + dy * dy > max_r_sq) {
        it = ground_patches_.erase(it);
        continue;
      }
    }

    if (wx < win_min_x - margin || wx > win_max_x + margin ||
        wy < win_min_y - margin || wy > win_max_y + margin)
    {
      it = ground_patches_.erase(it);
      continue;
    }

    if (patch.count_frame > 0u) {
      const float frame_z =
        static_cast<float>(patch.z_sum_frame / static_cast<double>(patch.count_frame));
      if (patch.weight <= 1e-4f) {
        patch.ground_z = frame_z;
      } else {
        patch.ground_z =
          (1.0f - ground_height_ema_alpha_) * patch.ground_z +
          ground_height_ema_alpha_ * frame_z;
      }
      patch.weight = std::min(50.0f, patch.weight + static_cast<float>(patch.count_frame));
      patch.count_frame = 0u;
      patch.z_sum_frame = 0.0;
    }

    patch.weight *= patch_decay;
    if (patch.weight < 0.05f) {
      it = ground_patches_.erase(it);
      continue;
    }

    ++it;
  }
}

void GroundClearanceLayer::fitGroundPlanes()
{
  const double patch_res = ground_patch_res_;
  const double max_z_jump = patch_res * 1.5 * max_plane_slope_tan_ + 0.15;

  for (auto & kv : ground_patches_) {
    const int32_t ix = unpackX(kv.first);
    const int32_t iy = unpackY(kv.first);
    auto & center = kv.second;
    center.plane_a = 0.0f;
    center.plane_b = 0.0f;
    center.plane_c = center.ground_z;
    center.has_plane = false;

    double suu = 0.0, suv = 0.0, svv = 0.0, su = 0.0, sv = 0.0, sw = 0.0;
    double suz = 0.0, svz = 0.0, sz = 0.0;
    int valid_neighbors = 0;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        auto nit = ground_patches_.find(packXY(ix + dx, iy + dy));
        if (nit == ground_patches_.end() || nit->second.weight < 0.05f) {
          continue;
        }
        const double nz = static_cast<double>(nit->second.ground_z);
        if (std::abs(nz - static_cast<double>(center.ground_z)) > max_z_jump) {
          continue;
        }
        const double u = static_cast<double>(dx) * patch_res;
        const double v = static_cast<double>(dy) * patch_res;
        const double w = std::clamp(static_cast<double>(nit->second.weight), 0.1, 10.0);
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
      const double b[3] = {suz, svz, sz};
      double sol[3] = {0.0, 0.0, static_cast<double>(center.ground_z)};
      if (solve3x3(A, b, sol) && std::hypot(sol[0], sol[1]) <= max_plane_slope_tan_) {
        center.plane_a = static_cast<float>(sol[0]);
        center.plane_b = static_cast<float>(sol[1]);
        center.plane_c = static_cast<float>(sol[2]);
        center.has_plane = true;
      }
    }
  }
}

void GroundClearanceLayer::rebuildBridgePatches()
{
  ground_bridge_patches_.clear();
  if (ground_patches_.empty() || ground_bridge_search_cells_ <= 0) {
    return;
  }

  const double patch_res = ground_patch_res_;
  const double bridge_res = ground_bridge_res_;
  ground_bridge_patches_.reserve(ground_patches_.size() / 3 + 32);

  for (const auto & kv : ground_patches_) {
    if (kv.second.weight < 0.05f) {
      continue;
    }
    const double wx = (static_cast<double>(unpackX(kv.first)) + 0.5) * patch_res;
    const double wy = (static_cast<double>(unpackY(kv.first)) + 0.5) * patch_res;
    const double w = std::clamp(static_cast<double>(kv.second.weight), 0.1, 10.0);
    auto & bp = ground_bridge_patches_[worldKey(wx, wy, bridge_res)];
    bp.sum_wz += w * static_cast<double>(kv.second.ground_z);
    bp.sum_w += w;
  }

  for (auto & kv : ground_bridge_patches_) {
    if (kv.second.sum_w > 0.0) {
      kv.second.mean_z = kv.second.sum_wz / kv.second.sum_w;
    }
  }
}

bool GroundClearanceLayer::estimateGroundHeight(
  double px, double py, double & out_ground_z, bool & out_from_fine) const
{
  out_from_fine = false;
  const double patch_res = ground_patch_res_;
  const int32_t ix = static_cast<int32_t>(std::floor(px / patch_res));
  const int32_t iy = static_cast<int32_t>(std::floor(py / patch_res));

  auto evalPatch = [&](int32_t gx, int32_t gy, const GroundPatch & p) -> double {
    if (!p.has_plane) {
      return static_cast<double>(p.ground_z);
    }
    const double cx = (static_cast<double>(gx) + 0.5) * patch_res;
    const double cy = (static_cast<double>(gy) + 0.5) * patch_res;
    return static_cast<double>(p.plane_a) * (px - cx) +
           static_cast<double>(p.plane_b) * (py - cy) +
           static_cast<double>(p.plane_c);
  };

  // Tier 1: Direct fine ground patch or local neighbor tilted-plane extrapolation
  auto it = ground_patches_.find(packXY(ix, iy));
  if (it != ground_patches_.end() && it->second.weight >= 0.05f) {
    out_ground_z = evalPatch(ix, iy, it->second);
    out_from_fine = true;
    return true;
  }

  const int r = ground_neighbor_search_cells_;
  if (r > 0) {
    double weighted_z = 0.0;
    double total_w = 0.0;
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        auto nit = ground_patches_.find(packXY(ix + dx, iy + dy));
        if (nit == ground_patches_.end() || nit->second.weight < 0.05f) {
          continue;
        }
        const double d2 = static_cast<double>(dx * dx + dy * dy);
        const double w = 1.0 / d2;
        weighted_z += w * evalPatch(ix + dx, iy + dy, nit->second);
        total_w += w;
      }
    }
    if (total_w > 0.0) {
      out_ground_z = weighted_z / total_w;
      out_from_fine = true;
      return true;
    }
  }

  // Tier 2: Coarse ground bridging across unobserved tunnel floor / sparse far ground
  const int br = ground_bridge_search_cells_;
  if (br > 0 && !ground_bridge_patches_.empty()) {
    const double bridge_res = ground_bridge_res_;
    const int32_t bx = static_cast<int32_t>(std::floor(px / bridge_res));
    const int32_t by = static_cast<int32_t>(std::floor(py / bridge_res));
    auto bit = ground_bridge_patches_.find(packXY(bx, by));
    if (bit != ground_bridge_patches_.end() && bit->second.sum_w > 0.0) {
      out_ground_z = bit->second.mean_z;
      return true;
    }

    double weighted_z = 0.0;
    double total_w = 0.0;
    for (int dx = -br; dx <= br; ++dx) {
      for (int dy = -br; dy <= br; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        auto nit = ground_bridge_patches_.find(packXY(bx + dx, by + dy));
        if (nit == ground_bridge_patches_.end() || nit->second.sum_w <= 0.0) {
          continue;
        }
        const double d2 = static_cast<double>(dx * dx + dy * dy);
        const double w = 1.0 / d2;
        weighted_z += w * nit->second.mean_z;
        total_w += w;
      }
    }
    if (total_w > 0.0) {
      out_ground_z = weighted_z / total_w;
      return true;
    }
  }

  return false;
}

void GroundClearanceLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  // Non-blocking lookup of current robot Z in global_frame_ for Tier 3 fallback
  if (tf_ && !robot_base_frame_.empty()) {
    try {
      const auto tf_robot = tf_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero);
      robot_ground_z_ = tf_robot.transform.translation.z - robot_base_height_;
      has_robot_z_ = true;
    } catch (const tf2::TransformException &) {
      // Keep previous robot_ground_z_ if available
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);

  last_robot_x_ = robot_x;
  last_robot_y_ = robot_y;
  has_robot_pose_ = true;

  useExtraBounds(min_x, min_y, max_x, max_y);
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  const double res = getResolution();
  double local_min_x = std::numeric_limits<double>::max();
  double local_min_y = std::numeric_limits<double>::max();
  double local_max_x = std::numeric_limits<double>::lowest();
  double local_max_y = std::numeric_limits<double>::lowest();

  auto touchLocal = [&](double wx, double wy) {
    local_min_x = std::min(local_min_x, wx);
    local_max_x = std::max(local_max_x, wx);
    local_min_y = std::min(local_min_y, wy);
    local_max_y = std::max(local_max_y, wy);
  };

  // Clear evidence inside robot footprint polygon
  if (footprint_clearing_enabled_ && !transformed_footprint_.empty()) {
    double fp_min_x = std::numeric_limits<double>::max();
    double fp_min_y = std::numeric_limits<double>::max();
    double fp_max_x = std::numeric_limits<double>::lowest();
    double fp_max_y = std::numeric_limits<double>::lowest();
    for (const auto & pt : transformed_footprint_) {
      fp_min_x = std::min(fp_min_x, pt.x);
      fp_min_y = std::min(fp_min_y, pt.y);
      fp_max_x = std::max(fp_max_x, pt.x);
      fp_max_y = std::max(fp_max_y, pt.y);
    }
    const int32_t min_xi = static_cast<int32_t>(std::floor(fp_min_x / res));
    const int32_t min_yi = static_cast<int32_t>(std::floor(fp_min_y / res));
    const int32_t max_xi = static_cast<int32_t>(std::floor(fp_max_x / res));
    const int32_t max_yi = static_cast<int32_t>(std::floor(fp_max_y / res));
    const size_t n = transformed_footprint_.size();

    for (int32_t xi = min_xi; xi <= max_xi; ++xi) {
      for (int32_t yi = min_yi; yi <= max_yi; ++yi) {
        const double px = (static_cast<double>(xi) + 0.5) * res;
        const double py = (static_cast<double>(yi) + 0.5) * res;
        bool inside = false;
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
          const auto & a = transformed_footprint_[i];
          const auto & b = transformed_footprint_[j];
          if ((a.y > py) != (b.y > py) &&
              px < (b.x - a.x) * (py - a.y) / (b.y - a.y) + a.x)
          {
            inside = !inside;
          }
        }
        if (inside) {
          if (cells_.erase(packXY(xi, yi)) > 0) {
            touchLocal(px, py);
          }
        }
      }
    }
  }

  // Real-time dt-based exponential decay (applied only when sensor data has arrived)
  const bool have_new_data = have_new_data_;
  const bool have_new_ground = have_new_ground_;
  have_new_data_ = false;
  have_new_ground_ = false;

  float g_decay = 1.0f;
  float o_decay = 1.0f;
  float p_decay = 1.0f;
  if (have_new_data) {
    const rclcpp::Time now = clock_->now();
    if (has_decay_time_ && now.get_clock_type() == last_decay_time_.get_clock_type()) {
      const double raw_dt = (now - last_decay_time_).seconds();
      if (raw_dt > 0.0) {
        const double dt = std::min(raw_dt, 1.0);
        g_decay = std::exp(-static_cast<float>(dt / ground_decay_tau_));
        o_decay = std::exp(-static_cast<float>(dt / obstacle_decay_tau_));
        p_decay = std::exp(-static_cast<float>(dt / ground_patch_decay_tau_));
      }
    }
    last_decay_time_ = now;
    has_decay_time_ = true;
  }

  // Master costmap window bounds
  const nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  const double win_origin_x = master->getOriginX();
  const double win_origin_y = master->getOriginY();
  const double win_max_x = win_origin_x + master->getSizeInMetersX();
  const double win_max_y = win_origin_y + master->getSizeInMetersY();

  // Step A: Update fine ground patches, fit tilted local planes, and refresh coarse bridge grid
  updateGroundPatches(
    robot_x, robot_y, win_origin_x, win_origin_y, win_max_x, win_max_y, p_decay);
  if (have_new_ground && !ground_patches_.empty()) {
    fitGroundPlanes();
  }
  rebuildBridgePatches();

  // Step B: Classify pending nonground points via 3-tier ground clearance + vertical guard
  if (!pending_nonground_pts_.empty()) {
    std::vector<Point3f> pts;
    pts.swap(pending_nonground_pts_);

    constexpr double kColRes = 0.20;
    std::unordered_map<WorldKey, VerticalColumn> cols;
    cols.reserve(pts.size() / 3 + 64);
    for (const auto & p : pts) {
      auto & col = cols[worldKey(p.x, p.y, kColRes)];
      col.z_min = std::min(col.z_min, static_cast<double>(p.z));
      col.z_max = std::max(col.z_max, static_cast<double>(p.z));
      col.count += 1u;
    }

    const double near_r_sq = near_unknown_ground_range_ * near_unknown_ground_range_;

    for (const auto & p : pts) {
      double ground_z = 0.0;
      bool from_fine = false;
      const bool ground_found = estimateGroundHeight(p.x, p.y, ground_z, from_fine);

      const double dx = static_cast<double>(p.x) - robot_x;
      const double dy = static_cast<double>(p.y) - robot_y;
      const bool is_near = (dx * dx + dy * dy <= near_r_sq);

      auto hasVerticalStructure = [&]() -> bool {
        auto cit = cols.find(worldKey(p.x, p.y, kColRes));
        return cit != cols.end() && cit->second.count >= 3u &&
               (cit->second.z_max - cit->second.z_min) >= min_unknown_ground_z_span_;
      };

      bool is_blocking = false;
      if (from_fine) {
        const double h = static_cast<double>(p.z) - ground_z;
        if (h >= min_clearance_ && h <= robot_height_) {
          is_blocking = true;
        }
      } else if (ground_found) {
        // Tier 2 bridged ground: filters overhead tunnel bars (h > robot_height_) even when
        // floor directly below is unobserved; at far range requires vertical extent to avoid
        // false obstacles on sparse uphill ramps bridged from downhill ground.
        const double h = static_cast<double>(p.z) - ground_z;
        if (h >= min_clearance_ && h <= robot_height_) {
          is_blocking = is_near || hasVerticalStructure();
        }
      } else if (is_near) {
        // Near robot with occluded floor: cap strictly at robot_height_ so overhead bars pass
        if (has_robot_z_) {
          const double h_rel = static_cast<double>(p.z) - robot_ground_z_;
          if (h_rel >= min_clearance_ && h_rel <= robot_height_) {
            is_blocking = true;
          }
        }
      } else if (unknown_ground_is_lethal_) {
        // Far with no ground within bridge radius: require genuine vertical structure
        auto cit = cols.find(worldKey(p.x, p.y, kColRes));
        if (cit != cols.end() && cit->second.count >= 3u &&
            (cit->second.z_max - cit->second.z_min) >= min_unknown_ground_z_span_)
        {
          const double h_local = static_cast<double>(p.z) - cit->second.z_min;
          if (h_local >= min_clearance_ && h_local <= robot_height_) {
            is_blocking = true;
          }
        }
      }

      if (is_blocking) {
        cells_[worldKey(p.x, p.y, res)].obstacle_count_frame += 1u;
      }
    }
  }

  // Step C: Integrate frame counts, decay, compute costs, cull, and touch dirty bounds
  constexpr float kEps = 1e-5f;
  const bool use_range_limit = (max_data_range_ > 0.0);
  const double max_range_sq = max_data_range_ * max_data_range_;

  for (auto it = cells_.begin(); it != cells_.end(); ) {
    auto & cell = it->second;
    const double wx = (static_cast<double>(unpackX(it->first)) + 0.5) * res;
    const double wy = (static_cast<double>(unpackY(it->first)) + 0.5) * res;

    // Cull cells beyond max_data_range_ (touch before erase so static_layer repaints cleanly!)
    if (use_range_limit) {
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      if (dx * dx + dy * dy > max_range_sq) {
        touchLocal(wx, wy);
        it = cells_.erase(it);
        continue;
      }
    }

    // Cull cells outside the master costmap bounds
    if (wx < win_origin_x || wx > win_max_x || wy < win_origin_y || wy > win_max_y) {
      it = cells_.erase(it);
      continue;
    }

    // Integrate ground frame evidence + EMA ground height
    if (cell.ground_count_frame > 0u) {
      const float frame_z = static_cast<float>(
        cell.ground_z_sum_frame / static_cast<double>(cell.ground_count_frame));
      if (!cell.has_ground) {
        cell.ground_z = frame_z;
        cell.has_ground = true;
      } else {
        cell.ground_z =
          (1.0f - ground_height_ema_alpha_) * cell.ground_z +
          ground_height_ema_alpha_ * frame_z;
      }
      const double add = static_cast<double>(cell.ground_count_frame) * ground_inc_;
      cell.ground_score = static_cast<float>(std::min(max_score_, cell.ground_score + add));
      cell.ground_count_frame = 0u;
      cell.ground_z_sum_frame = 0.0;
    }

    // Integrate collision-corridor obstacle evidence
    if (cell.obstacle_count_frame > 0u) {
      const double add = static_cast<double>(cell.obstacle_count_frame) * obstacle_inc_;
      cell.obstacle_score = static_cast<float>(std::min(max_score_, cell.obstacle_score + add));
      cell.obstacle_count_frame = 0u;
    }

    // Exponential decay
    if (have_new_data) {
      cell.ground_score *= g_decay;
      cell.obstacle_score *= o_decay;

      if (cell.ground_score < 1e-3f && cell.obstacle_score < 1e-3f) {
        touchLocal(wx, wy);
        it = cells_.erase(it);
        continue;
      }

      if (cell.ground_score < 1e-3f) {
        cell.ground_score = 0.0f;
      }
      if (cell.obstacle_score < 1e-3f) {
        cell.obstacle_score = 0.0f;
      }
    }

    // Evidence competition: obstacle_score vs ground_score
    const float g = cell.ground_score;
    const float o = cell.obstacle_score;
    const float p_occ = o / (o + g + kEps);

    uint8_t cost = nav2_costmap_2d::FREE_SPACE;
    if (o >= obstacle_occ_thresh_ && p_occ >= obstacle_prob_thresh_) {
      cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else if (!discretize_costs_) {
      cost = static_cast<uint8_t>(std::clamp(p_occ * 252.0f, 0.0f, 252.0f));
    }

    cell.computed_cost = cost;
    touchLocal(wx, wy);
    ++it;
  }

  if (local_min_x <= local_max_x) {
    *min_x = std::min(*min_x, local_min_x - res);
    *min_y = std::min(*min_y, local_min_y - res);
    *max_x = std::max(*max_x, local_max_x + res);
    *max_y = std::max(*max_y, local_max_y + res);
  }
}

void GroundClearanceLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  const std::vector<geometry_msgs::msg::Point> & footprint = getFootprint();
  if (footprint.empty()) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, footprint, transformed_footprint_);

  for (const auto & pt : transformed_footprint_) {
    touch(pt.x, pt.y, min_x, min_y, max_x, max_y);
  }
}

void GroundClearanceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const double res = master_grid.getResolution();

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);

      auto it = cells_.find(worldKey(wx, wy, res));
      if (it == cells_.end()) {
        continue;
      }

      const uint8_t cost = it->second.computed_cost;
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      const unsigned int idx = static_cast<unsigned int>(j) * size_x + static_cast<unsigned int>(i);
      if (combination_method_ == 0) {
        // Overwrite
        master_array[idx] = cost;
      } else {
        // Max (preserves static_layer walls; overwrites NO_INFORMATION)
        const unsigned char old_cost = master_array[idx];
        if (old_cost == nav2_costmap_2d::NO_INFORMATION || cost > old_cost) {
          master_array[idx] = cost;
        }
      }
    }
  }

  if (footprint_clearing_enabled_ && !transformed_footprint_.empty()) {
    master_grid.setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }
}

}  // namespace ground_clearance_layer

PLUGINLIB_EXPORT_CLASS(
  ground_clearance_layer::GroundClearanceLayer,
  nav2_costmap_2d::Layer)
