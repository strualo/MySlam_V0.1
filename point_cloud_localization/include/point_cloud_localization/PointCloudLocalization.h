#ifndef POINT_CLOUD_LOCALIZATION_H
#define POINT_CLOUD_LOCALIZATION_H

#include <ros/ros.h>
#include <geometry_utils/Transform3.h>

#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>

class PointCloudLocalization {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  PointCloudLocalization();
  ~PointCloudLocalization();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

  // Transform a point cloud from the sensor frame into the fixed frame using
  // the current best position estimate.
  bool TransformPointsToFixedFrame(const PointCloud& points,
                                   PointCloud* points_transformed) const;

  // Transform a point cloud from the fixed frame into the sensor frame using
  // the current best position estimate.
  bool TransformPointsToSensorFrame(const PointCloud& points,
                                    PointCloud* points_transformed) const;

  // Store incremental estimate from odometry.
  bool MotionUpdate(const geometry_utils::Transform3& incremental_odom);

  // Align incoming point cloud with a reference point cloud from the map.
  // Output the query scan aligned in the localization frame.
  bool MeasurementUpdate(const PointCloud::Ptr& query,
                         const PointCloud::Ptr& reference,
                         PointCloud* aligned_query);

  // Get pose estimates.
  const geometry_utils::Transform3& GetIncrementalEstimate() const;
  const geometry_utils::Transform3& GetIntegratedEstimate() const;

  // Set integrated estimate. Useful for graph SLAM whenever the pose graph is
  // updated and the map is regenerated.
  void SetIntegratedEstimate(
      const geometry_utils::Transform3& integrated_estimate);

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Publish reference, query, and aligned query point clouds.
  void PublishPoints(const PointCloud& points,
                     const ros::Publisher& pub) const;

  // Publish incremental and integrated pose estimates.
  void PublishPose(const geometry_utils::Transform3& pose,
                   const ros::Publisher& pub) const;

  // The node's name.
  std::string name_;

  // Pose estimate.
  geometry_utils::Transform3 incremental_estimate_;
  geometry_utils::Transform3 integrated_estimate_;

  // Publishers.
  ros::Publisher reference_pub_;
  ros::Publisher query_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher incremental_estimate_pub_;
  ros::Publisher integrated_estimate_pub_;

  // Most recent point cloud time stamp for publishers.
  ros::Time stamp_;

  // Coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  // Transform broadcasting to other nodes.
  tf2_ros::TransformBroadcaster tfbr_;

  // Parameters for filtering and ICP.
  struct Parameters {
    // Stop ICP if the transformation from the last iteration was this small.
    double tf_epsilon;

    // During ICP, two points won't be considered a correspondence if they are
    // at least this far from one another.
    double corr_dist;

    // Iterate ICP this many times.
    unsigned int iterations;
  } params_;

  // Maximum acceptable translation and rotation tolerances. If
  // transform_thresholding_ is set to false, neither of these thresholds are
  // considered.
  bool transform_thresholding_;
  double max_translation_;
  double max_rotation_;
};

#endif
