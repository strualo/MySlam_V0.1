# MySlam_V0.1
# MySlam_V0.1
总：　roslaunch blam_example run.launch

过滤原始雷达数据的地面：
plane_ground_filter   rlaunch plane_ground_filter plane_ground_filter.launch  

滤除建图结果指定高度：
ros_passthrough   roslaunch ros_passthrough  passthrough.launch   nh.subscribe<sensor_msgs::PointCloud2>("/blam/blam_slam/octree_map",2,laserCLoudHandler);　发布/passthrough

自己写的　将三维图转换成二维占据栅格地图：
cloud_to_map   roslaunch cloud_to_map cloud_to_map.launch    ros::Subscriber sub = nh.subscribe<PointCloud>("/passthrough", 1, callback); 

保存地图：
rosbag record -o out /blam/blam_slam/octree_map
rosrun pcl_ros bag_to_pcd out_2018-09-28-22-16-26.bag /blam/blam_slam/octree_map pcd

16线转单线：
    roslaunch  pointcloud_to_laserscan pointcloud_to_laserscan.launch     <remap from="cloud_in" to="/velodyne_points"/>   nh_.advertise<sensor_msgs::LaserScan>("scan", 10,
https://www.ncnynl.com/archives/201807/2554.html
https://www.ncnynl.com/archives/201810/2766.html





＆＆＆－－－　Ｖ０．１　－－－＆＆＆
建图并且生成二维占据栅格地图由多个程序包组成。
１．blam：　建图主要程序，得到初始点云地图。对地图的处理，可以选择使用（平整的地面情况下）（３）在激光雷达原始数据滤除地面，也可以选择使用（４）处理建图后的结果。
２．cloud_to_map：　处理点云地图得到　occupancy grid map　地图的包，有很多参数可以修改，决定最后地图的效果。
３．plane_fit_ground_filter：　处理激光雷达原始数据，滤除地面，用以建图。如果使用这个包，在ｂｌａｍ的ｌａｕｎｃｈ文件要修改订阅的雷达数据。
４．ros_passthrough：　处理建图后的结果，直通滤波，适合建图结果比较正的情况。如果使用这个包，需要修改（２）源代码中的订阅点云地图节点。
５．octomap_server.launch : 　octomap_server调用文件，可以生成Occypancygrid(三维占据栅格地图)和Occupancymap(二维占据栅格地图)！！！！！　cd blam:roslaunch octomap_server.launch 





