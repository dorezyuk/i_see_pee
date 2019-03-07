#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
namespace map {
namespace interpret_one {

struct interpret_fixture : public ::testing::Test {
  interpret_fixture() {
    nav_msgs::MapMetaData meta_data;
    meta_data.height = 10;
    meta_data.width = 10;
    meta_data.resolution = 0.05;
    meta_data.origin.position.x = 0;
    meta_data.origin.position.y = 0;
    grid.info = meta_data;
    grid.data.resize(meta_data.height * meta_data.width);
    std::fill(grid.data.begin(), grid.data.end(), 0);
  }
  nav_msgs::OccupancyGrid grid;
};

struct interpret_fixture_one :
        public interpret_fixture,
        public ::testing::WithParamInterface<coordinate_t>{};

INSTANTIATE_TEST_CASE_P(/**/,
        interpret_fixture_one,
        ::testing::Values(
                coordinate_t(5, 5),
                coordinate_t(0, 0),
                coordinate_t(9, 9)));

TEST_P(interpret_fixture_one, one) {
  converter c(grid.info);

  // get the index of the middle element and mark it in the data
  const knn_parameter p {3, 3};
  const index_t index = c.to_index(GetParam());
  grid.data[index] = 99;


  // get the knn_
  auto knn_full = interpret(grid, p);
  auto& knn = knn_full->knn_;

  // check the map not to be empty
  EXPECT_FALSE(knn.empty());

  // prepare vars for checks
  using data_t = Eigen::Matrix<long int, 2ul, 1ul>;
  const data_t center = c.to_coordinate(index).cast<long int>();
  const std::vector<position_t> expected = {c.to_position(index)};

  // iterate over the entire grid and check if the neighbors have been marked
  for(size_t ii = 0; ii < grid.data.size(); ++ii) {
    const data_t curr_p = c.to_coordinate(ii).cast<long int>();
    const double norm = (curr_p - center).norm();
    if(norm <= p.radius_) {
      // if we are inside the radius, make sure the index points to the center
      ASSERT_EQ(knn[ii], expected);
      knn.erase(ii);
    }
    else {
      // if we are outside the radius, make sure the element is missing
      EXPECT_EQ(knn.find(ii), knn.end());
    }
  }
  // make sure the knn_ did not contain any other values
  EXPECT_TRUE(knn.empty());
}

TEST_F(interpret_fixture, radius_zero) {
  // setup the variables
  const knn_parameter p{1, 0};
  const coordinate_t center(0,0);
  const converter c(grid.info);
  const index_t index = c.to_index(center);
  grid.data[index] = 99;

  // run the interpret function
  auto knn_full = interpret(grid, p);
  auto& knn = knn_full->knn_;

  // check if the knn_ is empty
  EXPECT_FALSE(knn.empty());

  // iterate over the entire grid.
  // all but the center index must be misses
  for(size_t ii = 0; ii < grid.data.size(); ++ii) {
    if(ii == index) {
      EXPECT_NE(knn.find(ii), knn.end());
      knn.erase(ii);
    }
    else {
      EXPECT_EQ(knn.find(ii), knn.end());
    }
  }
  EXPECT_TRUE(knn.empty());
}

TEST_F(interpret_fixture, radius_one) {
  // setup the variables
  const knn_parameter p{1, 1};
  const coordinate_t center(5, 5);
  const converter c(grid.info);
  const index_t index = c.to_index(center);
  grid.data[index] = 99;

  // run the interpret function
  auto knn_full = interpret(grid, p);
  auto& knn = knn_full->knn_;

  // check if the knn_ is empty
  EXPECT_FALSE(knn.empty());

  // iterate over the entire grid.
  // we expect to find exactly 9 hits (center + moore neighbors)
  size_t counter = 0;
  for(size_t ii = 0; ii < grid.data.size(); ++ii) {
    if(knn.find(ii) != knn.end()) {
      knn.erase(ii);
      ++counter;
    }
  }
  EXPECT_EQ(counter, 9);
  EXPECT_TRUE(knn.empty());
}

TEST_F(interpret_fixture, radius_all) {
  // setup the variables
  const knn_parameter p{1, 20};
  const coordinate_t center(5, 5);
  const converter c(grid.info);
  const index_t index = c.to_index(center);
  grid.data[index] = 99;

  // run the interpret function
  auto knn_full = interpret(grid, p);
  auto& knn = knn_full->knn_;

  // check if the knn_ is empty
  EXPECT_FALSE(knn.empty());

  // iterate over the entire grid.
  // we expect that every cell is marked
  size_t counter = 0;
  for(size_t ii = 0; ii < grid.data.size(); ++ii) {
    if(knn.find(ii) != knn.end()) {
      knn.erase(ii);
      ++counter;
    }
  }
  map_data m(grid.info);
  EXPECT_EQ(counter, m.size.prod());
  EXPECT_TRUE(knn.empty());
}

TEST_F(interpret_fixture, multiple) {
  using c_vector_t = std::vector<coordinate_t>;
  using p_vector_t = std::vector<position_t>;

  // setup the variables
  const knn_parameter p{3, 1};
  const c_vector_t marked = {{5, 5}, {5, 6}, {5, 7}};
  const converter c(grid.info );
  for(const auto& m : marked) {
    grid.data[c.to_index(m)] = 99;
  }

  // run the interpret function
  auto knn_full = interpret(grid, p);
  auto& knn = knn_full->knn_;

  // check if the knn_ is empty
  EXPECT_FALSE(knn.empty());

  // check the left index
  {
    SCOPED_TRACE("left marked failed");
    const index_t index = c.to_index(marked[0]);
    EXPECT_NE(knn.find(index), knn.end());
    // setup the expected value, sorted
    const p_vector_t expected = {c.to_position(marked[0]),
                                 c.to_position(marked[1])};
    const p_vector_t result = knn[index];

    EXPECT_EQ(expected, result);
  }

  // check the middle index
  {
    SCOPED_TRACE("middle marked failed");
    const index_t index = c.to_index(marked[1]);
    EXPECT_NE(knn.find(index), knn.end());
    // setup the expected value, sorted
    const p_vector_t expected = {c.to_position(marked[1]),
                                 c.to_position(marked[2]),
                                 c.to_position(marked[0])};
    const p_vector_t result = knn[index];

    EXPECT_EQ(expected, result);
  }

  // check the right index
  {
    SCOPED_TRACE("right marked failed");
    const index_t index = c.to_index(marked[2]);
    EXPECT_NE(knn.find(index), knn.end());
    // setup the expected value
    const p_vector_t expected = {c.to_position(marked[2]),
                                 c.to_position(marked[1])};
    const p_vector_t result = knn[index];

    EXPECT_EQ(expected, result);
  }
}

} // namespace interpret_one


} // namespace map
} // namespace i_see_pee
