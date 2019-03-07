#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
namespace icp {
namespace point_to_map_test{

TEST(point_to_map_test, simple){
  std::srand(std::time(nullptr));

  transform_t tf(Eigen::Rotation2Df(0.25) * Eigen::Translation2f(.1, .2));

  scan::scan_t sensor(scan::scan_t::Random(2, 10));
  scan::scan_t map = tf * sensor;

  icp::matches m(sensor, map);
  icp::weight_t w(icp::weight_t::Ones(10));

  transform_t out = icp::point_to_map(std::move(m), std::move(w));
  const auto diff = (tf.matrix() - out.matrix()).norm();
  EXPECT_NEAR(diff, 0, 0.01);
}

} // namespace point_to_map_test
} // namespace icp
} // namespace i_see_pee
