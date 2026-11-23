#pragma once

// Patchwork++ and Patchwork classic
#include "patchwork/patchwork.h"
#include "patchwork/patchworkpp.h"

// Standard library
#include <deque>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace patchworkpp_ros {

class GroundSegmentationServer : public rclcpp::Node {
 public:
  /// GroundSegmentationServer constructor
  GroundSegmentationServer() = delete;
  explicit GroundSegmentationServer(const rclcpp::NodeOptions &options);

 private:
  /// Register new frame
  void EstimateGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  /// Filter nonground points against local ground tilted planes to produce
  /// clearance-filtered obstacles with height-above-ground in intensity.
  Eigen::MatrixX4f FilterClearanceObstacles(const Eigen::MatrixX3f &local_ground,
                                            const Eigen::MatrixX3f &local_nonground) const;

  /// Stream the point clouds
  void PublishClouds(const Eigen::MatrixX3f &est_ground,
                     const Eigen::MatrixX3f &est_nonground,
                     const Eigen::MatrixX4f &filtered_obstacles,
                     const std_msgs::msg::Header &header_msg);

  /// Parameter loaders — only the selected algorithm's loader is called
  patchwork::Params loadPlusplusParamsFromROS();
  patchwork::PatchworkParams loadClassicParamsFromROS();

 private:
  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  /// Data publishers.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;

  /// TF2 for gravity-aligned odom <-> base_link centering
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// Algorithm implementation (patchworkpp or patchwork classic)
  using ImplVariant =
      std::variant<std::unique_ptr<patchwork::PatchWorkpp>, std::unique_ptr<patchwork::PatchWork>>;
  ImplVariant impl_;

  /// Sliding frame buffer in odom_frame_ to mitigate sparse single scans
  std::deque<std::vector<Eigen::Vector3f>> frame_buffer_;

  /// Frame & pre-filter parameters
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  double tf_timeout_{0.1};
  int accum_frames_{3};
  double origin_z_offset_{0.573};
  double min_range_{0.4};
  double max_range_{12.0};
  double sensor_height_{0.573};
  double max_rel_z_below_{-1.5};
  double max_rel_z_above_{2.5};

  /// Filtered obstacle output parameters (for SLAM / pointcloud_to_laserscan)
  bool publish_filtered_obstacles_{true};
  double min_clearance_{0.10};
  double robot_height_{0.70};
  double obstacle_max_range_{7.0};
  double obstacle_ground_grid_res_{0.30};
  int obstacle_ground_search_radius_{2};
  double obstacle_ground_bridge_res_{0.60};
  int obstacle_ground_bridge_radius_{4};
  double near_unknown_ground_range_{2.5};
  double min_unknown_ground_z_span_{0.22};
};

}  // namespace patchworkpp_ros
