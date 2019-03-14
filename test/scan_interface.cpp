#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
namespace scan {
namespace interface_test {

struct interface_fixture :
        public controller_base_,
        public interface_,
        public ::testing::Test {
  interface_fixture() : interface_(this), has_data_(false){}
  void update(const scan::scan_t &_scan,
              const stamp_t &_stamp,
              const std::string &_scan_frame) override {
    scan_ = _scan;
    scan_frame_ = _scan_frame;
    has_data_ = true;
  }
  scan::scan_t scan_;
  std::string scan_frame_;
  bool has_data_;
};

TEST_F(interface_fixture, empty) {
  // setup the scan message: message is empty and we expect not to have
  // any response to that
  sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan());
  msg->angle_min = 0;
  msg->angle_max = 0;
  msg->angle_increment = 1;
  msg->ranges.clear();

  sensor_msgs::LaserScanConstPtr msg_ptr(std::move(msg));

  // call the callback
  callback(msg_ptr);

  // analyse the result: we should not have any data to move
  scan_ptr_t scan;
  std::string scan_frame;
  EXPECT_FALSE(has_data_);
}

TEST_F(interface_fixture, circle) {
  const size_t size = 180;
  // setup the scan message: message has constant ranges, which should generate
  // a circle message
  sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan());
  msg->angle_min = -M_PI_2;
  msg->angle_max = M_PI_2;
  const float angle = M_PI / (size - 1);
  msg->angle_increment = angle;
  msg->ranges.resize(size);
  msg->header.frame_id = "dummy";
  std::fill(msg->ranges.begin(), msg->ranges.end(), 1.);

  sensor_msgs::LaserScanConstPtr msg_ptr(std::move(msg));

  // call the callback
  callback(msg_ptr);

  // analyze the result: we want a unit circle with 180 elements around zero
  EXPECT_TRUE(has_data_);
  EXPECT_EQ(size, scan_.cols());
  EXPECT_EQ(scan_frame_, msg_ptr->header.frame_id);

  using vector_t = Eigen::Vector2f;
  using matrix_t = Eigen::Matrix2f;

  vector_t curr(0, -1);
  matrix_t rot;
  rot << std::cos(angle), -std::sin(angle),
         std::sin(angle), std::cos(angle);
  for (size_t ii = 0; ii < size; ++ii) {
    const vector_t diff = curr - scan_.col(ii);
    EXPECT_NEAR(diff.norm(), 0, 0.01);
    curr = rot * curr;
  }
}

} // namespace interface_test
} // namespace scan
} // namespace i_see_pee

