#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
namespace internal {
namespace stamp_test {

using parameter_t = std::tuple<stamp_rep_t__, stamp_rep_t__ >;
struct stamp_fixture : public ::testing::TestWithParam<parameter_t>{};

// define [min, mid, max] values for the seconds
constexpr stamp_rep_t__ sec_min = std::numeric_limits<stamp_rep_t__>::min();
constexpr stamp_rep_t__ sec_max = std::numeric_limits<stamp_rep_t__>::max();
constexpr stamp_rep_t__ sec_mid = sec_max / 2;

// define [min, mid1, mid2, max] values for the nanoseconds
constexpr stamp_rep_t__ nsec_min = 0;
constexpr stamp_rep_t__ nsec_max = 999999999;
constexpr stamp_rep_t__ nsec_mid1 = 123456789;
constexpr stamp_rep_t__ nsec_mid2 = 987654321;

INSTANTIATE_TEST_CASE_P(/**/, stamp_fixture,
                              ::testing::Combine(
                                      ::testing::Values(sec_min, sec_max, sec_mid),
                                      ::testing::Values(nsec_min, nsec_max, nsec_mid1, nsec_mid2)));


TEST_P(stamp_fixture, all){
  // get the seconds and nanoseconds from the parameter
  const auto sec = std::get<0>(GetParam());
  const auto nsec = std::get<1>(GetParam());

  // build the input stamp and check that the double conversion is not
  // changing it.
  const stamp_t__ in{sec, nsec};
  const auto out = to_stamp(to_stamp(in));
  EXPECT_EQ(in, out);
}

} // namespace stamp_test
} // namespace internal
} // namespace i_see_pee
