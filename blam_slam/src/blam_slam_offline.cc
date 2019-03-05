
#include <ros/ros.h>
#include <blam_slam/BlamSlamOffline.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "blam_slam");
  ros::NodeHandle n("~");

  BlamSlamOffline bso;
  if (!bso.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize BLAM SLAM offline processor.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
