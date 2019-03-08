#ifndef I_SEE_PEE_I_SEE_PEE_HPP
#define I_SEE_PEE_I_SEE_PEE_HPP

#include <bits/stdc++.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>

#include <Eigen/Dense>

namespace i_see_pee {

using index_t = size_t;
using transform_t = Eigen::Affine2f;

namespace internal {

using transform_t__ = geometry_msgs::TransformStamped;
using pose_t__ = geometry_msgs::Pose;
using quaternion_t__ = geometry_msgs::Quaternion;

inline bool is_valid(const quaternion_t__& _q) noexcept {
  // define a vector in order to check if the length of the quaternion is
  // smaller than 1. Double is required since geometry_msgs uses double as
  // default floating point type.
  using vector_t = Eigen::Matrix<double, 4ul, 1ul>;
  vector_t q(_q.w, _q.x, _q.y, _q.z);
  return q.norm() <= 1.;
}

transform_t to_transform(const transform_t__ &_tf) noexcept;
transform_t to_transform(const pose_t__ & _p) noexcept;
transform_t__ to_transform(const transform_t &_t) noexcept;
transform_t__ to_transform(const transform_t &_t,
                           const std::string &_frame,
                           const std::string &_child) noexcept;

} // namespace internal

namespace map {

using position_t = Eigen::Vector2f;
using coordinate_t = Eigen::Matrix<size_t, 2ul, 1ul>;

struct map_data {

  explicit map_data(const nav_msgs::MapMetaData &m) noexcept;
  map_data(const coordinate_t &_size,
           float _resolution,
           const position_t &_origin) noexcept;

  float resolution;
  coordinate_t size;
  position_t origin;
};

struct converter {

  explicit converter(const map_data &p) noexcept;
  explicit converter(const nav_msgs::MapMetaData &m) noexcept;

  index_t to_index(const position_t &p) const;
  index_t to_index(const coordinate_t &c) const;
  coordinate_t to_coordinate(const position_t &p) const;
  coordinate_t to_coordinate(index_t i) const;
  position_t to_position(index_t _i) const;
  position_t to_position(const coordinate_t& _c) const;

  inline bool valid_position(const position_t &_p) const noexcept {
    const position_t relative = _p - p_.origin;
    if(relative.minCoeff() < 0){
      return false;
    }
    const coordinate_t out = (relative / p_.resolution).cast<size_t>();
    return valid_coordinate(out);
  }

  inline bool valid_coordinate(const coordinate_t& _c) const noexcept {
    return (_c.array() < p_.size.array()).prod() == 1;
  }

  inline bool valid(index_t _i) const noexcept {
    return _i < p_.size.prod();
  }

private:
  const map_data p_;
};

struct knn_t{
  using data_t = std::unordered_map<index_t, std::vector<position_t>>;

  explicit knn_t(const nav_msgs::MapMetaData& _grid, size_t _k) noexcept;
  explicit knn_t(const map_data& _parameter, size_t _k) noexcept;

  data_t knn_;
  size_t k_;
  const converter converter_;
};

using knn_ptr_t  = std::unique_ptr<knn_t>;
using map_value__ = signed char;

constexpr map_value__ occupied = 50;

inline bool is_occupied(map_value__ _val) noexcept {
  return _val > occupied;
}

struct knn_parameter {

  static constexpr size_t k_max = 10;
  static constexpr size_t k_min = 1;
  static constexpr size_t k_def = 5;

  static constexpr size_t radius_max = 10;
  static constexpr size_t radius_min = 0;
  static constexpr size_t radius_def = 5;

  explicit knn_parameter(size_t _k = k_def, size_t _radius = radius_def) noexcept;
  explicit knn_parameter(ros::NodeHandle& _nh);

  size_t k_;
  size_t radius_;

  friend std::ostream &operator<<(std::ostream &_os,
                                  const knn_parameter &_p) noexcept;
};

knn_ptr_t interpret(const nav_msgs::OccupancyGrid &grid,
                    const knn_parameter &_p) noexcept;

struct interface {

  explicit interface(ros::NodeHandle &_nh);
  bool move_data(knn_ptr_t& _knn) noexcept;

private:
  void callback(const nav_msgs::OccupancyGridConstPtr &_msg) noexcept;

  knn_parameter p_;
  knn_ptr_t  knn_;
  ros::Subscriber sub_;
};

} // namespace map

namespace scan {

using scan_t = Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::RowMajor>;
using scan_ptr_t = std::unique_ptr<scan_t>;

using scan_view_t = Eigen::Map<const Eigen::Matrix<float, 1, Eigen::Dynamic>>;

struct parameter {

  explicit parameter(const sensor_msgs::LaserScan &_scan) noexcept;

  bool operator==(const parameter &_other) const noexcept;
  bool operator!=(const parameter &_other) const noexcept;

  const size_t size;
  const float increment, angle_min, angle_max;
};

using parameter_ptr_t = std::unique_ptr<parameter>;

scan_ptr_t cache(const parameter &_p) noexcept;

struct controller_base_ {
  virtual ~controller_base_() = default;
  virtual void update(const scan_t& _scan, const std::string& _scan_frame) {};
};

struct interface_ {

  explicit interface_(controller_base_* _controller = nullptr);
  virtual ~interface_() = default;

protected:
  void callback(const sensor_msgs::LaserScanConstPtr &_msg);

  controller_base_* controller_;
  parameter_ptr_t p_;
  scan_ptr_t cache_;
};

struct interface : public interface_ {
  explicit interface(ros::NodeHandle &_nh, controller_base_* _controller = nullptr);

private:
  ros::Subscriber sub_;
};

} // namespace scan

namespace odom {

struct interface_tf {

  interface_tf();
  transform_t get(const std::string& _from, const std::string& _to);
  void set(const transform_t &_tf,
          const std::string& _from,
          const std::string& _to) noexcept;

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

struct interface_rviz {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static constexpr char topic[] = "/initialpose";

  // todo pass ref to map_frame_ instead of node handle
  interface_rviz(ros::NodeHandle &_nh, transform_t &_map_to_base) noexcept;

private:
  void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& _msg) noexcept;
  transform_t& map_to_base_;
  std::string map_frame_;
  ros::Subscriber sub_;
};

struct frame_handler {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  explicit frame_handler(ros::NodeHandle& _nh);

  transform_t get_map_to_base() noexcept ;
  transform_t get_map_to_odom() const noexcept;
  transform_t get_map_to_odom(const transform_t& _map_to_base) noexcept ;
  transform_t get_base_to_sensor(const std::string& _scan_frame) noexcept;

  void set_map_to_odom(const transform_t& _tf) noexcept;

private:
  void handle_map_to_base();
  void handle_base_to_sensor(const std::string& _scan_frame);

  transform_t map_to_base_, odom_to_base_, base_to_sensor_;
  interface_tf tf_;
  interface_rviz rviz_;

  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
};

} // namespace odom

namespace icp {

struct matches{

  matches(const scan::scan_t& _sensor, const scan::scan_t& _map) noexcept;
  matches(const scan::scan_t& _scan, const map::knn_t& _knn) noexcept;

  scan::scan_t sensor_;
  scan::scan_t map_;
};

using weight_t = Eigen::VectorXf;
weight_t get_weights(const matches& _data);

transform_t point_to_map(matches&& _data, weight_t&& _w);

struct conversion_checker{

  static constexpr float t_norm = 0.01f;
  static constexpr float r_norm = 0.01f;

  explicit conversion_checker(float _t_norm = t_norm,
                              float _r_norm = r_norm) noexcept;
  explicit conversion_checker(ros::NodeHandle &_nh);

  bool operator()(const transform_t &_in) noexcept;

private:
  float t_norm_, r_norm_;
};

struct sampler {

  static constexpr float percentage_def = .3f;
  static constexpr float percentage_min = 0.f;
  static constexpr float percentage_max = 1.f;

  explicit sampler(float _percentage = percentage_def) noexcept;
  explicit sampler(ros::NodeHandle& _nh);

  scan::scan_t operator()(const scan::scan_t& _in) noexcept;

private:
  float percentage_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<float> dist_;
};

struct scan_matcher {

  static constexpr size_t max_iter = 100;
  static constexpr size_t min_iter = 1;

  explicit scan_matcher(ros::NodeHandle& _nh);
  transform_t operator()(const scan::scan_t& _scan);

private:
  conversion_checker is_converged_;
  sampler sample_;
  map::knn_ptr_t knn_;
  map::interface map_;
  size_t iter_;
};

} // namespace icp

struct controller : public scan::controller_base_ {

  explicit controller(ros::NodeHandle& _nh);
  void update(const scan::scan_t& _scan, const std::string& _scan_frame) override;

private:
  icp::scan_matcher icp_;
  scan::interface scan_interface_;
  odom::frame_handler frame_handler_;
};

} // namespace i_see_pee

#endif //I_SEE_PEE_I_SEE_PEE_HPP
