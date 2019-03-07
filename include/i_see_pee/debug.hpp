#ifndef I_SEE_PEE_DEBUG_HPP
#define I_SEE_PEE_DEBUG_HPP

#include <i_see_pee/i_see_pee.hpp>

#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>

#include <ros/ros.h>

namespace i_see_pee {
namespace debug {

using point_cloud_t = sensor_msgs::PointCloud;

inline point_cloud_t to_message(const scan::scan_t& _in){
  point_cloud_t out;
  out.points.resize(_in.cols());
  for(size_t cc = 0; cc <_in.cols(); ++cc) {
    out.points[cc].x = _in.col(cc)(0);
    out.points[cc].y = _in.col(cc)(1);
    out.points[cc].z = 0;
  }
  return std::move(out);
}

inline point_cloud_t to_message(const typename map::knn_t::data_t& _knn){
  point_cloud_t out;
  out.points.resize(_knn.size());
  size_t cc = 0;
  for(const auto& nn : _knn){
    out.points[cc].x = nn.second.front()(0);
    out.points[cc].y = nn.second.front()(1);
    out.points[cc].z = 0;
    ++cc;
  }
  return std::move(out);
}

inline std_msgs::Header to_header(const std::string& _frame){
  std_msgs::Header out;
  out.frame_id = _frame;
  out.stamp = ros::Time::now();
  return out;
}

struct diagnostics {
  static inline void set_node_handle(ros::NodeHandle *_nh) { nh_ = _nh;}
  static inline void publish(const scan::scan_t &_in,
                      const std::string &_topic,
                      const std::string &_frame) {
    if (!nh_) {
      return;
    }

    if (pubs_.find(_topic) == pubs_.end()) {
      pubs_[_topic] = nh_->advertise<point_cloud_t>(_topic, 1);
    }
    point_cloud_t out = to_message(_in);
    out.header = to_header(_frame);
    pubs_[_topic].publish(out);
  }


private:
  static ros::NodeHandle *nh_;
  static std::unordered_map<std::string, ros::Publisher> pubs_;
};

ros::NodeHandle* diagnostics::nh_ = nullptr;
std::unordered_map<std::string, ros::Publisher> diagnostics::pubs_{};

} // namespace debug
} // namespace i_see_pee

#endif //I_SEE_PEE_DEBUG_HPP
