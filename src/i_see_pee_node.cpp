#include <i_see_pee/i_see_pee.hpp>
#include <i_see_pee/debug.hpp>
#include <ros/ros.h>

using namespace i_see_pee;

int main(int argc, char** argv){
  ros::init(argc, argv, "i_see_pee_node");
  ros::NodeHandle nh("~");

  debug::diagnostics::set_node_handle(&nh);

  controller c(nh);
  ros::spin();
  return 0;
}