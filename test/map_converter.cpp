#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

// optional compliance test
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/GridMap.hpp>

namespace i_see_pee{
namespace map{

namespace converter_integrity{

// Tests first against internal integrity
/*         index  pos  cor
 * index |       |x   |x  |
 * pos   |       |    |   |
 * cor   |       |x   |   |
 */

struct converter_fixture {
  converter_fixture() : p(coordinate_t(10, 20), 0.05, position_t(1, 2)), c(p) {}
  const map_data p;
  const converter c;
};

struct converter_fixture_i2c : public converter_fixture, public testing::TestWithParam<index_t>{};

INSTANTIATE_TEST_CASE_P(/*prefix*/, converter_fixture_i2c, ::testing::Values(0, 19, 20, 99, 100, 199));

TEST_P(converter_fixture_i2c, index_to_cor) {
   const index_t index = GetParam();
   const coordinate_t cor = c.to_coordinate(index);
   EXPECT_EQ(c.to_index(cor), index);
}

struct converter_fixture_i2p : public converter_fixture, public testing::TestWithParam<index_t>{};

INSTANTIATE_TEST_CASE_P(/*prefix*/, converter_fixture_i2p, ::testing::Values(0, 19, 20, 99, 100, 199));

TEST_P(converter_fixture_i2p, index_to_pos){
   const index_t index = GetParam();
   const position_t cor = c.to_position(index);
   EXPECT_EQ(c.to_index(cor), index);
}

struct converter_fixture_c2p : public converter_fixture, ::testing::TestWithParam<coordinate_t>{};

INSTANTIATE_TEST_CASE_P(/*prefix*/,
        converter_fixture_c2p,
        ::testing::Values(coordinate_t(0, 0),
                          coordinate_t(0, 19),
                          coordinate_t(1, 0),
                          coordinate_t(4, 19),
                          coordinate_t(5, 0),
                          coordinate_t(9, 19)));


TEST_P(converter_fixture_c2p, cor_to_pos) {
   const coordinate_t cor = GetParam();
   const position_t pos = c.to_position(cor);
   EXPECT_EQ(c.to_coordinate(pos), cor);
}

} // namespace converter_integrity

namespace converter_death {

// Test covers the case, where an input is out of bounds.
// Four tests: index, coordinate, position to high, position to low
TEST(converter_death, out_of_bounds) {
   map_data p(coordinate_t(20, 50), 1, position_t(0, 0));
   converter c(p);

   {
      SCOPED_TRACE("out_of_bounds test failed for index");

      // get the last index
      const index_t index = p.size.prod() -1;
      EXPECT_TRUE(c.valid(index));
      EXPECT_FALSE(c.valid(index + 1));
   }

   {
      SCOPED_TRACE("out_of_bounds failed for coordinate");

      // get the last coordinate
      const coordinate_t cor(19, 49);
      EXPECT_TRUE(c.valid_coordinate(cor));
      EXPECT_FALSE(c.valid_coordinate(cor + coordinate_t::Ones()));
   }

   {
      SCOPED_TRACE("out_of_bounds failed for positive position");

      // get the last valid position
      const position_t pos(19, 49);
      EXPECT_TRUE(c.valid_position(pos));
      EXPECT_FALSE(c.valid_position(pos + position_t::Ones()));
   }


   {
      SCOPED_TRACE("out_of_bounds failed for negative position");

      // get the last valid position
      const position_t pos(0, 0);
      EXPECT_TRUE(c.valid_position(pos));
      EXPECT_FALSE(c.valid_position(pos - position_t::Ones()));
   }
}

} // namespace converter_death

//namespace converter_compliance {
//
//struct converter_compliance_fixture : public ::testing::TestWithParam<grid_map::Index> {
//
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//
//  converter_compliance_fixture() {
//     // setup the map parameter
//     nav_msgs::MapMetaData meta_data;
//     meta_data.origin.position.x = 10;
//     meta_data.origin.position.y = -15;
//     meta_data.resolution = 0.05;
//     meta_data.width = 10;
//     meta_data.height = 50;
//
//     // setup the occupancy grid map
//     og_map.info = meta_data;
//     og_map.header.frame_id = "map";
//     og_map.data.resize(meta_data.width * meta_data.height);
//     int8_t n = 0;
//     std::generate(og_map.data.begin(), og_map.data.end(), [&n](){return std::abs(++n);});
//
//     // setup the grid_map
//     grid_map::GridMapRosConverter::fromOccupancyGrid(og_map, "main", gm_map);
//
//     // setup a own map
//     using matrix_t = Eigen::Matrix<double, 10, 50>;
//     double *p = &icp_map(0, 0);
//     std::copy(og_map.data.begin(), og_map.data.end(), p);
//  }
//  nav_msgs::OccupancyGrid og_map;
//  grid_map::GridMap gm_map;
//  Eigen::Matrix<double, 10, 50> icp_map;
//};
//
//INSTANTIATE_TEST_CASE_P(/**/,
//        converter_compliance_fixture,
//        ::testing::Values(grid_map::Index(6, 40),
//                          grid_map::Index(0, 0),
//                          grid_map::Index(9, 49)));
//
//TEST_P(converter_compliance_fixture, DISABLED){
//   converter icp_converter(og_map.info);
//
//   // get the grid map position and coordinate
//   const grid_map::Index input = GetParam();
//   grid_map::Position gm_pos;
//
//   gm_map.getPosition(input, gm_pos);
//
//   // get own coordinate from position
//   const position_t icp_pos = gm_pos.cast<float>();
//   const coordinate_t icp_cor = icp_converter.to_coordinate(icp_pos);
//
//   // assert that the mapping of both is same
//   EXPECT_EQ(gm_map.at("main", input), icp_map(icp_cor(0), icp_cor(1)));
//}
//
//} // namespace converter_compliance
} // namespace map
} // namespace i_see_pee

