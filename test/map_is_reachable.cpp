#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

namespace i_see_pee {
namespace map {
namespace is_reachable_test {

struct is_reachable_fixture : public ::testing::Test {
  is_reachable_fixture() noexcept {
    grid_.info.width = 3;
    grid_.info.height = 3;
    grid_.data.resize(grid_.info.width * grid_.info.height);
  }
  nav_msgs::OccupancyGrid grid_;
};

TEST_F(is_reachable_fixture, full) {
  // set all cells to occupied
  std::fill(grid_.data.begin(), grid_.data.end(), occupied);

  // setup the structure under test
  is_reachable tested(grid_);

  // every cell must be not reachable
  for(size_t ii = 0; ii < grid_.data.size(); ++ii) {
    EXPECT_FALSE(tested(ii));
  }
}

TEST_F(is_reachable_fixture, empty) {
  // set all cells to free
  std::fill(grid_.data.begin(), grid_.data.end(), occupied - 1);

  // setup the structure under test
  is_reachable tested(grid_);

  // every cell must be reachable
  for(size_t ii = 0; ii < grid_.data.size(); ++ii) {
    EXPECT_TRUE(tested(ii));
  }
}

TEST_F(is_reachable_fixture, invalid_center) {
  // set all cells to free
  std::fill(grid_.data.begin(), grid_.data.end(), occupied - 1);

  // setup the structure under test
  is_reachable tested(grid_);

  // every cell must be not reachable, since it is not on the grid
  const auto offset = grid_.info.height * grid_.info.width;
  for(size_t ii = 0; ii < grid_.data.size(); ++ii) {
    EXPECT_FALSE(tested(offset + ii));
  }
}

TEST_F(is_reachable_fixture, checker) {
  // set grid to following checker pattern:
  // o x o
  // x o x
  // o x o
  std::fill(grid_.data.begin(), grid_.data.end(), occupied - 1);
  std::array<size_t, 4> occupied_i = {1, 3, 5, 7};
  for(const auto ii : occupied_i) {
    grid_.data[ii] = occupied;
  }

  // setup the structure under test
  is_reachable tested(grid_);

  // every cell must be reachable
  for(size_t ii = 0; ii < grid_.data.size(); ++ii) {
    EXPECT_TRUE(tested(ii));
  }
}

} // namespace is_reachable_test
} // namespace map
} // namespace i_see_pee