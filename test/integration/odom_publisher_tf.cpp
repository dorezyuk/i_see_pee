#include <i_see_pee/i_see_pee.hpp>
#include <i_see_pee/macros.hpp>

#include <ros/ros.h>

using namespace i_see_pee::odom;
using namespace i_see_pee;

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle nh;

  frame_handler controller_(nh);

  ros::Rate r(50);
  while(ros::ok()){
    r.sleep();
    const transform_t unused = controller_.get_map_to_base();
    const transform_t map_to_odom = controller_.get_map_to_odom();
    const auto stamp = internal::to_stamp(ros::Time::now());
    controller_.set_map_to_odom(map_to_odom, stamp);
  }
}
