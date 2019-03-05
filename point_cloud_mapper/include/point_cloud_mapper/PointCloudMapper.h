#ifndef POINT_CLOUD_MAPPER_H
#define POINT_CLOUD_MAPPER_H

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <mutex>
#include <thread>

class PointCloudMapper {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

  PointCloudMapper();
  ~PointCloudMapper();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

  // Resets the octree and stored points to an empty map. This is used when
  // closing loops or otherwise regenerating the map from scratch.
  void Reset();

  // Adds a set of points to the octree. Only inserts points if one does not
  // already exist in the corresponding voxel. Output the points from the input
  // that ended up being inserted into the octree.
  bool InsertPoints(const PointCloud::ConstPtr& points,
                    PointCloud* incremental_points);

  // Returns the approximate nearest neighbor for every point in the input point
  // cloud. Localization to the map can be performed by doing ICP between the
  // input and output. Returns true if at least one of the inputs points had a
  // nearest neighbor.
  bool ApproxNearestNeighbors(const PointCloud& points, PointCloud* neighbors);

  // Publish map for visualization. This can be expensive so it is not called
  // from inside, as opposed to PublishMapUpdate().
  void PublishMap();

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Threaded version to avoid blocking SLAM when the map gets big.
  void PublishMapThread();

  // Publish map updates for visualization.
  void PublishMapUpdate(const PointCloud& incremental_points);

  // The node's name.
  std::string name_;

  // Frame id for publishing map.
  std::string fixed_frame_id_;

  // Boolean initialization flag that is set after success from LoadParameters.
  bool initialized_;

  // Boolean to only publish the map if it has been updated recently.
  bool map_updated_;

  // When a loop closure occurs, this flag enables a user to unsubscribe from
  // and resubscribe to the incremental map topic in order to re-draw the map.
  bool incremental_unsubscribed_;

  // Containers storing the map and its structure.
  PointCloud::Ptr map_data_;
  Octree::Ptr map_octree_;

  // Map parameters.
  double octree_resolution_;

  // Map publisher.
  ros::Publisher map_pub_;
  ros::Publisher incremental_map_pub_;
  std::thread publish_thread_;
  mutable std::mutex map_mutex_;
};

#endif
