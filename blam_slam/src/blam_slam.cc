#include <ros/ros.h>
#include <blam_slam/BlamSlam.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "blam_slam");
  ros::NodeHandle n("~");

  BlamSlam bs;
  if (!bs.Initialize(n, false /* online processing */)) {
    ROS_ERROR("%s: Failed to initialize BLAM SLAM.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
