
#ifndef POINT_CLOUD_ODOMETRY_H
#define POINT_CLOUD_ODOMETRY_H

#include <ros/ros.h>
#include <geometry_utils/Transform3.h>

#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>

class PointCloudOdometry {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  PointCloudOdometry();
  ~PointCloudOdometry();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

  // Align incoming point cloud with previous point cloud, updating odometry.
  bool UpdateEstimate(const PointCloud& points);

  // Get pose estimates.
  const geometry_utils::Transform3& GetIncrementalEstimate() const;
  const geometry_utils::Transform3& GetIntegratedEstimate() const;

  // Get the most recent point cloud that we processed. Return false and warn if
  // not initialized.
  bool GetLastPointCloud(PointCloud::Ptr& out) const;

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Use ICP between a query and reference point cloud to estimate pose.
  bool UpdateICP();

  // Publish reference and query point clouds.
  void PublishPoints(const PointCloud::Ptr& points,
                     const ros::Publisher& pub);

  // Publish incremental and integrated pose estimates.
  void PublishPose(const geometry_utils::Transform3& pose,
                   const ros::Publisher& pub);

  // The node's name.
  std::string name_;

  // Pose estimates.
  geometry_utils::Transform3 integrated_estimate_;
  geometry_utils::Transform3 incremental_estimate_;

  // Publishers.
  ros::Publisher reference_pub_;
  ros::Publisher query_pub_;
  ros::Publisher incremental_estimate_pub_;
  ros::Publisher integrated_estimate_pub_;

  // Most recent point cloud time stamp for publishers.
  ros::Time stamp_;

  // Coordinate frames.
  std::string fixed_frame_id_;
  std::string odometry_frame_id_;

  // Transform broadcasting to other nodes.
  tf2_ros::TransformBroadcaster tfbr_;

  // For initialization.
  bool initialized_;

  // Point cloud containers.
  PointCloud::Ptr query_;
  PointCloud::Ptr reference_;

  // Parameters for filtering, and ICP.
  struct Parameters {
    // Stop ICP if the transformation from the last iteration was this small.
    double icp_tf_epsilon;

    // During ICP, two points won't be considered a correspondence if they are
    // at least this far from one another.
    double icp_corr_dist;

    // Iterate ICP this many times.
    unsigned int icp_iterations;
  } params_;

  // Maximum acceptable translation and rotation tolerances. If
  // transform_thresholding_ is set to false, neither of these thresholds are
  // considered.
  bool transform_thresholding_;
  double max_translation_;
  double max_rotation_;
};

#endif
