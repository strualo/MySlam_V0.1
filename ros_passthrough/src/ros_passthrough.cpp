#include <iostream>
#include <ros/ros.h>  
#include <sensor_msgs/PointCloud2.h>  
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>  

ros::Publisher pcl_pub;


 //基于RANSAC
void laserCLoudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*laserCloudMsg,*cloudIn); 

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloudIn);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1.7,-0.4);
  pass.setFilterLimitsNegative (false);
  //pass.setNegative(true);
  pass.filter(*cloud_filtered);

  //再rviz上显示所以要转换回PointCloud2
  sensor_msgs::PointCloud2 cloud_filteredMsg;  
  pcl::toROSMsg(*cloud_filtered,cloud_filteredMsg);
  cloud_filteredMsg.header.stamp=laserCloudMsg->header.stamp;
  cloud_filteredMsg.header.frame_id="/world";
  pcl_pub.publish (cloud_filteredMsg);
}


int main (int argc, char **argv)  
{  
  ros::init (argc, argv, "ros_passthrough");  
  ros::NodeHandle nh;  
  ros::Subscriber subLaserCloud=nh.subscribe<sensor_msgs::PointCloud2>("/blam/blam_slam/octree_map",2,laserCLoudHandler);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/passthrough", 2);  //参数缓冲区大小　２
  //ros::Rate loop_rate(0.5);　　//数据发送频率
  ros::spin();  
  return 0;  
}  

