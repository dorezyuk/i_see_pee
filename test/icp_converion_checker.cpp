#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
namespace icp {
namespace conversion_checker_test {

TEST(conversion_checker_test, all){
  conversion_checker checker(0.1, 0.2);

  const transform_t in = Eigen::Translation2f(-0.02f, -0.02f) * Eigen::Rotation2Df(-0.19f);
  {
    SCOPED_TRACE("failed for converged transform");

    ASSERT_TRUE(checker(in));
  }

  {
    SCOPED_TRACE("failed for too large translation");

    transform_t out = in;
    out.translation() = Eigen::Vector2f(0.8f, 0.8f);
    EXPECT_FALSE(checker(out));
  }

  {
    SCOPED_TRACE("failed for too rotation translation");

    transform_t out = Eigen::Translation2f(-0.02f, -0.02f) * Eigen::Rotation2Df(0.21f);
    EXPECT_FALSE(checker(out));
  }
}


} // conversion_checker_test
} // namespace icp
} // namespace i_see_pee