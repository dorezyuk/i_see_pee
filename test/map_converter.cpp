#include <i_see_pee/i_see_pee.hpp>
#include <gtest/gtest.h>

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
} // namespace map
} // namespace i_see_pee

