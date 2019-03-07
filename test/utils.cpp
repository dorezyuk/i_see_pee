#include <i_see_pee/i_see_pee.hpp>
#include <i_see_pee/utils.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
namespace maybe_insert_test {

TEST(maybe_insert_test, on_double){
  using vector_t  = std::vector<double>;
  vector_t data;
  data.reserve(10);

  auto comparator = [](double l, double r) { return l < r;};

  {
    SCOPED_TRACE("simple insert case failed");

    // generate expected data
    vector_t expected(data.capacity());
    size_t n = 0;
    std::generate(expected.begin(), expected.end(),
                  [&n]() { return static_cast<double>(++n); });

    // run maybe_insert on the data
    for (size_t ii = data.capacity(); ii > 0; --ii) {
      maybe_insert(data, static_cast<double>(ii), comparator);
    }
    EXPECT_EQ(expected, data);
    EXPECT_EQ(data.size(), data.capacity());
  }

  {
    SCOPED_TRACE("out_of_bounds test case failed");

    // try to add numbers which are larger then the largest.
    // the resulting vector shall remain the same
    const auto max = data.back();
    const auto expected = data;
    for(size_t ii = 0; ii < 10; ++ii){
      maybe_insert(data, max + ii, comparator);
    }

    EXPECT_EQ(expected, data);
  }

  {
    SCOPED_TRACE("smart insert case failed");

    // try to insert now numbers smaller than the min
    // generate expected data [-5, 5)
    vector_t expected(data.capacity());
    long int n = (data.capacity()) / 2; n = -n; --n;
    std::generate(expected.begin(), expected.end(),
                  [&n]() { return static_cast<double>(++n); });


    // now insert the negative side into the data
    for (size_t ii = 0; ii <= data.capacity() / 2; ++ii) {
      maybe_insert(data, -static_cast<double>(ii), comparator);
    }

    EXPECT_EQ(expected, data);
  }
}

} // namespace maybe_insert_test

namespace cast_to_range_test {

template <typename T>
struct parameter {
  T min, max, in, out;
};

template <typename T>
struct cast_to_range_fixture : public ::testing::TestWithParam<parameter<T>> {};

namespace cast_to_range_double {

using parameter_t = parameter<double>;
using cast_to_range_double_t = cast_to_range_fixture<double>;

INSTANTIATE_TEST_CASE_P(/**/,
        cast_to_range_double_t,
        ::testing::Values(parameter_t{0, 1, 0.5, 0.5}, // positive in
                          parameter_t{0, 1, 1.2, 1},   // positive above
                          parameter_t{0, 1, -1, 0},    // positive below
                          parameter_t{-2, -1, -1.5, -1.5},
                          parameter_t{-2, -1, 1, -1},
                          parameter_t{-2, -1, -3, -2}));

TEST_P(cast_to_range_double_t, all){
  const auto p = GetParam();
  EXPECT_EQ(cast_to_range(p.in, p.min, p.max), p.out);
}

} // cast_to_range_double

namespace cast_to_range_int {

using parameter_t = parameter<int>;
using cast_to_range_int_t = cast_to_range_fixture<int>;

INSTANTIATE_TEST_CASE_P(/**/,
        cast_to_range_int_t,
        ::testing::Values(parameter_t{0, 2, 1, 1}, // positive in
                          parameter_t{0, 2, 3, 2},   // positive above
                          parameter_t{0, 2, -1, 0},    // positive below
                          parameter_t{-3, -1, -2, -2},
                          parameter_t{-3, -1, 1, -1},
                          parameter_t{-3, -1, -4, -3}));

TEST_P(cast_to_range_int_t, all){
  const auto p = GetParam();
  EXPECT_EQ(cast_to_range(p.in, p.min, p.max), p.out);
}

} // cast_to_range_int
} // cast_to_range_test
} // namespace i_see_pee
