#include <ros/ros.h>
#include <point_cloud_odometry/PointCloudOdometry.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_odometry");
  ros::NodeHandle n("~");

  PointCloudOdometry pco;
  if (!pco.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud odometry.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
