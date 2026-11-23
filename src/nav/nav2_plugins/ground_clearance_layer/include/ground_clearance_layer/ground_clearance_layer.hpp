#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ground_clearance_layer
{

class GroundClearanceLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  GroundClearanceLayer() = default;
  ~GroundClearanceLayer() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void matchSize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert) override;
  bool isClearable() override { return true; }

  using WorldKey = uint64_t;

private:
  struct Point3f
  {
    float x;
    float y;
    float z;
  };

  struct CellData
  {
    float ground_score{0.0f};
    float obstacle_score{0.0f};
    float ground_z{0.0f};
    bool has_ground{false};
    uint32_t ground_count_frame{0};
    double ground_z_sum_frame{0.0};
    uint32_t obstacle_count_frame{0};
    uint8_t computed_cost{nav2_costmap_2d::NO_INFORMATION};
  };

  struct GroundPatch
  {
    float ground_z{0.0f};
    float weight{0.0f};
    double z_sum_frame{0.0};
    uint32_t count_frame{0};
    float plane_a{0.0f};
    float plane_b{0.0f};
    float plane_c{0.0f};
    bool has_plane{false};
  };

  struct BridgePatch
  {
    double sum_wz{0.0};
    double sum_w{0.0};
    double mean_z{0.0};
  };

  struct VerticalColumn
  {
    double z_min{std::numeric_limits<double>::max()};
    double z_max{std::numeric_limits<double>::lowest()};
    uint32_t count{0};
  };

  static inline WorldKey packXY(int32_t xi, int32_t yi)
  {
    return (static_cast<WorldKey>(static_cast<uint32_t>(xi)) << 32) |
           static_cast<WorldKey>(static_cast<uint32_t>(yi));
  }

  static inline int32_t unpackX(WorldKey k)
  {
    return static_cast<int32_t>(static_cast<uint32_t>(k >> 32));
  }

  static inline int32_t unpackY(WorldKey k)
  {
    return static_cast<int32_t>(static_cast<uint32_t>(k & 0xFFFFFFFFu));
  }

  static inline WorldKey worldKey(double wx, double wy, double res)
  {
    const int32_t xi = static_cast<int32_t>(std::floor(wx / res));
    const int32_t yi = static_cast<int32_t>(std::floor(wy / res));
    return packXY(xi, yi);
  }

  void groundCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void nongroundCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);

  void updateGroundPatches(
    double robot_x, double robot_y,
    double win_min_x, double win_min_y, double win_max_x, double win_max_y,
    float patch_decay);

  void fitGroundPlanes();
  void rebuildBridgePatches();

  bool estimateGroundHeight(
    double px, double py, double & out_ground_z, bool & out_from_fine) const;

  // Topics & frames
  std::string ground_topic_;
  std::string nonground_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double tf_timeout_{0.1};
  int combination_method_{1};  // 0 = Overwrite, 1 = Max

  // Evidence competition & dt-based exponential decay
  float ground_inc_{1.0f};
  float obstacle_inc_{1.5f};
  double ground_decay_tau_{0.5};
  double obstacle_decay_tau_{0.8};
  float obstacle_occ_thresh_{2.0f};
  float obstacle_prob_thresh_{0.65f};
  double max_score_{500.0};
  bool discretize_costs_{true};

  // Collision clearance corridor [min_clearance_, robot_height_]
  double min_clearance_{0.10};
  double robot_height_{0.70};
  double robot_base_height_{0.0};
  bool unknown_ground_is_lethal_{true};
  double near_unknown_ground_range_{2.5};
  double min_unknown_ground_z_span_{0.22};

  // Coarse tilted-plane + bridging ground model
  double ground_patch_res_{0.30};
  int ground_neighbor_search_cells_{2};
  double ground_bridge_res_{0.60};
  int ground_bridge_search_cells_{4};
  float ground_height_ema_alpha_{0.35f};
  double ground_patch_decay_tau_{3.0};
  double max_plane_slope_deg_{32.0};
  double max_plane_slope_tan_{0.625};

  // Range culling & footprint clearing
  double max_data_range_{6.0};
  bool footprint_clearing_enabled_{true};
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;

  // Persistent maps & pending buffers (protected by mutex_)
  std::unordered_map<WorldKey, CellData> cells_;
  std::unordered_map<WorldKey, GroundPatch> ground_patches_;
  std::unordered_map<WorldKey, BridgePatch> ground_bridge_patches_;
  std::vector<Point3f> pending_nonground_pts_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_sub_;

  std::mutex mutex_;
  bool have_new_data_{false};
  bool have_new_ground_{false};
  bool has_decay_time_{false};
  rclcpp::Time last_decay_time_{0, 0, RCL_ROS_TIME};

  bool has_robot_pose_{false};
  double last_robot_x_{0.0};
  double last_robot_y_{0.0};
  bool has_robot_z_{false};
  double robot_ground_z_{0.0};
};

}  // namespace ground_clearance_layer
