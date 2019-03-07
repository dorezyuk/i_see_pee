#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
// todo rename and move to internal
namespace odom {
namespace interface_tf_test{

struct parameter {
  double x, y, yaw;
};

struct interface_tf_fixture_conversion : public ::testing::TestWithParam<parameter> {};

INSTANTIATE_TEST_CASE_P(/**/,
        interface_tf_fixture_conversion,
        ::testing::Values(parameter{3, 5, .2},
                          parameter{-3, 5, -.2},
                          parameter{3, -5, .8},
                          parameter{-3, -5, -.8},
                          parameter{5, 3, 1.4},
                          parameter{-5, -3, -1.4}));



TEST_P(interface_tf_fixture_conversion, conversion) {
  internal::transform_t__ in;
  const parameter p = GetParam();
  in.transform.translation.x = p.x;
  in.transform.translation.y = p.y;
  in.transform.translation.z = 0;


  tf2::Quaternion q;
  q.setRPY(0, 0, p.yaw);

  in.transform.rotation.x = q.x();
  in.transform.rotation.y = q.y();
  in.transform.rotation.z = q.z();
  in.transform.rotation.w = q.w();

  // do the forward transformation
  const transform_t out = internal::to_transform(in);
  const Eigen::Vector2f t = out.translation();
  const Eigen::Matrix2f r = out.rotation();
  EXPECT_NEAR(t(0), p.x, 0.01);
  EXPECT_NEAR(t(1), p.y, 0.01);
  EXPECT_NEAR(std::atan2(r(1, 0), r(0, 0)), p.yaw, 0.01);

  const internal::transform_t__ backward = internal::to_transform(out);
  EXPECT_NEAR(in.transform.translation.x, backward.transform.translation.x, 0.01);
  EXPECT_NEAR(in.transform.translation.y, backward.transform.translation.y, 0.01);
  EXPECT_NEAR(in.transform.translation.z, backward.transform.translation.z, 0.01);
  EXPECT_NEAR(in.transform.rotation.x, backward.transform.rotation.x, 0.01);
  EXPECT_NEAR(in.transform.rotation.y, backward.transform.rotation.y, 0.01);
  EXPECT_NEAR(in.transform.rotation.z, backward.transform.rotation.z, 0.01);
  EXPECT_NEAR(in.transform.rotation.w, backward.transform.rotation.w, 0.01);
}

} // namespace interface_tf_test
} // namespace odom
} // namespace i_see_pee
