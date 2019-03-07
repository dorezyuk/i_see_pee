#include <i_see_pee/i_see_pee.hpp>
#include <i_see_pee/debug.hpp>
#include <i_see_pee/macros.hpp>

namespace i_see_pee {

struct knn_publisher {
  explicit knn_publisher(ros::NodeHandle& _nh) : interface_(_nh), knn_(nullptr){
    pub_ = _nh.advertise<debug::point_cloud_t>("knn", 1);
  } 
  
  void run(){
    interface_.move_data(knn_);
    if(knn_){
      auto knn = debug::to_message(knn_->knn_);
      knn.header = debug::to_header("map");
      pub_.publish(knn);
    }
  }
private:
  ros::Publisher pub_;
  map::interface interface_;
  map::knn_ptr_t knn_;
};

} // namespace i_see_pee

using namespace i_see_pee;

int main(int argc, char** argv){
  ros::init(argc, argv, "knn");
  ros::NodeHandle nh("~");
  
  knn_publisher tested(nh);
  
  ros::Rate r(20);
  while(ros::ok()){
    tested.run();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

