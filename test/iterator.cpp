#include <i_see_pee/iterator.hpp>

#include <gtest/gtest.h>

namespace i_see_pee{
namespace centered_submap_iterator_double{

using iterator_t = centered_submap_iterator<double>;
using data_t = iterator_t::data_t;
using bound_t = iterator_t::bound_t;

// expect that no iterator access shall be made when at least one
// dimension of the array is zero
TEST(centered_submap_iterator_double, zero){
  {
    iterator_t iter(bound_t(0, 0));
    EXPECT_TRUE(iter.past_end());
  }

  {
    iterator_t iter(bound_t(10, 0));
    EXPECT_TRUE(iter.past_end());
  }

  {
    iterator_t iter(bound_t(0, 10));
    EXPECT_TRUE(iter.past_end());
  }

}

// expect that we get only one valid iterator access for (1, 1)
TEST(centered_submap_iterator_double, one){
  iterator_t iter(bound_t(1, 1));
  size_t counter = 0;
  data_t expected = data_t::Zero();
  for(; !iter.past_end(); ++iter, ++counter){
    EXPECT_EQ(expected, *iter);
  }
  EXPECT_EQ(counter, 1ul);
}

// expect that swapping the dimensions is handled correctly
TEST(centered_submap_iterator_double, two){
  {
    // vertical case
    iterator_t iter(bound_t(1, 2));
    size_t counter = 0;
    std::array<data_t, 2> expected = {data_t(0, 0), data_t(0, 1)};
    for(; !iter.past_end(); ++iter, ++counter){
      EXPECT_EQ(expected[counter], *iter);
    }
    EXPECT_EQ(counter, expected.size());
  }

  {
    // horizontal case
    iterator_t iter(bound_t(2, 1));
    size_t counter = 0;
    std::array<data_t, 2> expected = {data_t(0, 0), data_t(1, 0)};
    for(; !iter.past_end(); ++iter, ++counter){
      EXPECT_EQ(expected[counter], *iter);
    }
    EXPECT_EQ(counter, expected.size());
  }
}

// check if the iteration goes proper
TEST(centered_submap_iterator_double, multiple){
  iterator_t iter(bound_t(4, 3));
  size_t counter = 0;
  std::array<data_t, 12> expected = {data_t(0, 0), data_t(0, 1), data_t(0, 2),
                                     data_t(1, 0), data_t(1, 1), data_t(1, 2),
                                     data_t(2, 0), data_t(2, 1), data_t(2, 2),
                                     data_t(3, 0), data_t(3, 1), data_t(3, 2)};

  for(; !iter.past_end(); ++iter, ++counter){
    EXPECT_EQ(expected[counter], *iter);
  }
  EXPECT_EQ(counter, expected.size());
}

} // namespace centered_submap_iterator_double

namespace centered_submap_iterator_int{

using iterator_t = centered_submap_iterator<int>;
using data_t = iterator_t::data_t;
using bound_t = iterator_t::bound_t;

// check if the iteration goes proper for int (similar case to the double)
TEST(centered_submap_iterator_int, multiple){
  iterator_t iter(bound_t(4, 3));
  size_t counter = 0;
  std::array<data_t, 12> expected = {data_t(0, 0), data_t(0, 1), data_t(0, 2),
                                     data_t(1, 0), data_t(1, 1), data_t(1, 2),
                                     data_t(2, 0), data_t(2, 1), data_t(2, 2),
                                     data_t(3, 0), data_t(3, 1), data_t(3, 2)};

  for(; !iter.past_end(); ++iter, ++counter){
    EXPECT_EQ(expected[counter], *iter);
  }
  EXPECT_EQ(counter, expected.size());
}

} // namespace centered_submap_iterator_int

namespace submap_iterator_double{

using iterator_t = submap_iterator<double>;
using data_t = typename iterator_t::data_t;
using bound_t = typename iterator_t::bound_t;

const std::array<data_t, 12> expected = {data_t(0, 0), data_t(0, 1), data_t(0, 2),
                                          data_t(1, 0), data_t(1, 1), data_t(1, 2),
                                          data_t(2, 0), data_t(2, 1), data_t(2, 2),
                                          data_t(3, 0), data_t(3, 1), data_t(3, 2)};


TEST(submap_iterator_double, positive_shift){
  // prepare expected data to be shifted
  data_t origin(10, 20);
  auto shifted = expected;
  for (auto &s : shifted) {
    s += origin;
  }

  // setup the iterator
  iterator_t iter(origin, bound_t(4, 3));
  size_t counter = 0;

  // compare the generated values
  for(; !iter.past_end(); ++iter, ++counter){
    EXPECT_EQ(shifted[counter], *iter);
  }
  EXPECT_EQ(counter, shifted.size());
}

} // namespace submap_iterator_double

namespace submap_iterator_int {

using iterator_t = submap_iterator<int>;
using data_t = typename iterator_t::data_t;
using bound_t = typename iterator_t::bound_t;

const std::array<data_t, 12> expected = {data_t(0, 0), data_t(0, 1), data_t(0, 2),
                                         data_t(1, 0), data_t(1, 1), data_t(1, 2),
                                         data_t(2, 0), data_t(2, 1), data_t(2, 2),
                                         data_t(3, 0), data_t(3, 1), data_t(3, 2)};


TEST(submap_iterator_int, negative_shift){
  // prepare expected data to be shifted
  data_t origin(-30, -10);
  auto shifted = expected;
  for (auto &s : shifted) {
    s += origin;
  }

  // setup the iterator
  iterator_t iter(origin, bound_t(4, 3));
  size_t counter = 0;

  // compare the generated values
  for(; !iter.past_end(); ++iter, ++counter){
    EXPECT_EQ(shifted[counter], *iter);
  }
  EXPECT_EQ(counter, shifted.size());
}

} // namespace submap_iterator_int

namespace circle_iterator_double {

using iterator_t = circle_iterator<int>;
using data_t = typename iterator_t::data_t;
using bound_t = typename iterator_t::bound_t;

TEST(circle_iterator_double, first) {
  iterator_t iter(data_t(10, 10), 7);
  using matrix_t = Eigen::Matrix<int, 20, 20>;
  matrix_t map = matrix_t::Zero();

  for(; !iter.past_end(); ++iter){
    ++map((*iter)(0), (*iter)(1));
  }
  using namespace std;
  cout << map << endl;
}

} // namespace circle_iterator_double

} // namespace i_see_pee


//int main(int argc, char **argv){
//  testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();
//}
//

