// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud2.h>

// using namespace sensor_msgs;
// using namespace message_filters;

// ros::Publisher image_pub;
// ros::Publisher lidar_pub;


// //void callback(const ImageConstPtr& image, const ImuConstPtr& imu, const PointCloud2ConstPtr& PointCloud2)
// void callback(const ImageConstPtr& image, const PointCloud2ConstPtr& PointCloud2)
// {
//   printf("---------下面两个时间戳的image和lidar同步对齐起来------------\n");
  
//   ros::Time image_time = image->header.stamp;// 获取图像消息的时间戳
//   ros::Time lidar_time = PointCloud2->header.stamp;// 获取点云消息的时间戳
  
//   printf("Image timestamp: %f \n", image_time.toSec());// 打印图像消息的时间戳
//   printf("PointCloud2 timestamp: %f \n", lidar_time.toSec());// 打印点云消息的时间戳

//   image_pub.publish(image);
//   lidar_pub.publish(PointCloud2);
//   //发布频率只有10hz,正常的,因为livox的频率就是10hz,image是40hz，只有同时接受到livox和image后,才能执行这个回调函数.
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "sync_camera_lidar_node");//初始化一个新节点。
//   ros::NodeHandle nh;

//   image_pub = nh.advertise<Image>("/synced_Image", 1);// 创建图像发布者
//   lidar_pub = nh.advertise<PointCloud2>("/synced_PointCloud2", 1); // 创建点云发布者

//   message_filters::Subscriber<Image> image_sub(nh, "/left_camera/image", 1);//  订阅left_camera/image 话题的类型
//   //message_filters::Subscriber<Imu> imu_sub(nh, "/livox/imu", 1);//  /livox/imu 话题的类型
//   message_filters::Subscriber<PointCloud2> PointCloud2_sub(nh, "/livox/lidar", 1);//订阅livox/lidar的话题

//   //typedef sync_policies::ApproximateTime<Image, Imu, PointCloud2> MySyncPolicy;
//   typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;

//   //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, imu_sub,PointCloud2_sub);
//   Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub,PointCloud2_sub);//20

//   //sync.registerCallback(boost::bind(&callback, _1, _2, _3));
//   sync.registerCallback(boost::bind(&callback, _1, _2));

//   ros::spin();
//   return 0;
// }


//同时订阅两个话题，并打印各自的时间戳。

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace message_filters;
void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    // 获取图像消息的时间戳
    ros::Time image_time = image_msg->header.stamp;
    // 打印图像消息的时间戳
    ROS_INFO("Image timestamp: %f", image_time.toSec());
}
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg)
{
    // 获取点云消息的时间戳
    ros::Time lidar_time = lidar_msg->header.stamp;
    // 打印点云消息的时间戳
    ROS_INFO("PointCloud2 timestamp: %f", lidar_time.toSec());
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // 获取imu消息的时间戳
    ros::Time imu_time = imu_msg->header.stamp;
    // 打印imu消息的时间戳
    ROS_INFO("imu timestamp: %f", imu_time.toSec());
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_camera_lidar_node");//初始化一个新节点。
  ros::NodeHandle nh;

  // 订阅图像话题
  ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/left_camera/image", 5, imageCallback);
  // 订阅点云话题
  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 5, lidarCallback);
  //订阅imu话题
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 5, imuCallback);
  
  ros::spin();
  return 0;
}




//精确时间同步。不适合我们的情况。
/*
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
using namespace sensor_msgs;
using namespace message_filters;
void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{
  printf("callback function!\n");
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  typedef sync_policies::ExactTime<Image, CameraInfo> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}
*/
