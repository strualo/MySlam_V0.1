#ifndef POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
#define POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

#include "nodelet/nodelet.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"


namespace pointcloud_to_laserscan
{
  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;
/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
  class PointCloudToLaserScanNodelet : public nodelet::Nodelet
  {

  public:
    PointCloudToLaserScanNodelet();

  private:
    virtual void onInit();

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
        tf2_ros::filter_failure_reasons::FilterFailureReason reason);

    void connectCb();

    void disconnectCb();

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher pub_;
    boost::mutex connect_mutex_;

    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
    boost::shared_ptr<MessageFilter> message_filter_;

    // ROS Parameters
    unsigned int input_queue_size_;
    std::string target_frame_;
    double tolerance_;
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
    bool use_inf_;
    double inf_epsilon_;
  };

}  // pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
