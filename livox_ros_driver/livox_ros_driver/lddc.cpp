//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"

#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "lds_lidar.h"
#include "lds_lvx.h"
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

// time_stamp ;
struct time_stamp {
  int64_t high;
  int64_t low;
};
extern struct time_stamp *pointt;

namespace livox_ros {

#define ERR_EXIT(m)                                                            \
  do {                                                                         \
    perror(m);                                                                 \
    exit(EXIT_FAILURE);                                                        \
  } while (0)

/** Lidar Data Distribute Control-----激光雷达数据分布控制---------------------------------------*/
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id, bool lidar_bag, bool imu_bag)
    : transfer_format_(format), use_multi_topic_(multi_topic),
      data_src_(data_src), output_type_(output_type), publish_frq_(frq),
      frame_id_(frame_id), enable_lidar_bag_(lidar_bag),
      enable_imu_bag_(imu_bag) {

  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
  memset(private_pub_, 0, sizeof(private_pub_));
  memset(private_imu_pub_, 0, sizeof(private_imu_pub_));
  global_pub_ = nullptr;
  global_imu_pub_ = nullptr;
  cur_node_ = nullptr;
  bag_ = nullptr;
};

Lddc::~Lddc() {//析构函数,释放相关资源
  if (global_pub_) {
    delete global_pub_;
  }
  if (global_imu_pub_) {
    delete global_imu_pub_;
  }
  if (lds_) {
    lds_->PrepareExit();
  }
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_pub_[i]) {
      delete private_pub_[i];
    }
  }
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_imu_pub_[i]) {
      delete private_imu_pub_[i];
    }
  }
}

//获取即将要发布的数据的起始时间。
//通过计算时间戳与周期的余数来确定发布数据的起始时间，并通过跳过数据包来实现对齐周期边界。
int32_t Lddc::GetPublishStartTime(LidarDevice *lidar, LidarDataQueue *queue,
                                  uint64_t *start_time,
                                  StoragePacket *storage_packet) {
  QueuePrePop(queue, storage_packet);//从队列中取出一段lidar数据并放入 storage_packet 对象中。

  //获取这段lidar数据的时间戳
  uint64_t timestamp = GetStoragePacketTimestamp(storage_packet, lidar->data_src);

  uint32_t remaining_time = timestamp % publish_period_ns_;
  uint32_t diff_time = publish_period_ns_ - remaining_time;

  /** Get start time, down to the period boundary */
  if (diff_time > (publish_period_ns_ / 4)) {
    // ROS_INFO("0 : %u", diff_time);
    *start_time = timestamp - remaining_time;
    return 0;
  } else if (diff_time <= lidar->packet_interval_max) {
    *start_time = timestamp;
    return 0;
  } else {
    /** Skip some packets up to the period boundary 跳过一些数据包直到周期边界 */
    // ROS_INFO("2 : %u", diff_time);
    do {
      if (QueueIsEmpty(queue)) {
        break;
      }
      QueuePopUpdate(queue); /* skip packet */
      QueuePrePop(queue, storage_packet);
      uint32_t last_remaning_time = remaining_time;
      timestamp = GetStoragePacketTimestamp(storage_packet, lidar->data_src);
      remaining_time = timestamp % publish_period_ns_;
      /** Flip to another period */
      if (last_remaning_time > remaining_time) {
        // ROS_INFO("Flip to another period, exit");
        break;
      }
      diff_time = publish_period_ns_ - remaining_time;
    } while (diff_time > lidar->packet_interval);

    /* the remaning packets in queue maybe not enough after skip 跳过后，队列中的剩余数据包可能不够 */
    return -1;
  }
}

void Lddc::InitPointcloud2MsgHeader(sensor_msgs::PointCloud2 &cloud) {
  cloud.header.frame_id.assign(frame_id_);//指定点云消息的坐标系。
  cloud.height = 1;//表示该点云消息只有一行数据。
  cloud.width = 0;//表示该点云消息的宽度尚未确定。
  cloud.fields.resize(6);//容纳 6 个字段的描述信息。

  cloud.fields[0].offset = 0;//在点云数据中的字节偏移量（FLOAT32有4字节，UINT8有1字节）
  cloud.fields[0].name = "x";//字段名称
  cloud.fields[0].count = 1;//字段的数量
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;//字段的数据类型

  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;

  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;

  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = sensor_msgs::PointField::UINT8;

  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = sensor_msgs::PointField::UINT8;

  cloud.point_step = sizeof(LivoxPointXyzrtl);//每个点的字节大小。
}

uint32_t Lddc::PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num, uint8_t handle) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;//已发布的数据包数量

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip  跳过后，队列中的剩余数据包可能不够 */
    return 0;
  }

  sensor_msgs::PointCloud2 cloud;
  InitPointcloud2MsgHeader(cloud);
  cloud.data.resize(packet_num * kMaxPointPerEthPacket *
                    sizeof(LivoxPointXyzrtl));
  cloud.point_step = sizeof(LivoxPointXyzrtl);

  uint8_t *point_base = cloud.data.data();
  uint8_t data_source = lidar->data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
  uint32_t is_zero_packet = 0;

  // 在一个循环中，函数从队列中取出数据包，计算数据包的时间间隔（packet_gap）并判断是否需要进行时间间隔补偿。
  while ((published_packet < packet_num) && !QueueIsEmpty(queue)) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    
    //printf("timestamp: %lu\n", timestamp);//打印出livox的时间戳

    int64_t packet_gap = timestamp - last_timestamp;
    if ((packet_gap > lidar->packet_interval_max) &&
        lidar->data_is_pubulished) {
      // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle,
      //     packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + lidar->packet_interval;
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
      }
    }
    /** Use the first packet timestamp as pointcloud2 msg timestamp 
    使用第一个数据包时间戳作为pointcloud2 msg时间戳*/
    if (!published_packet) {

      // //处理lidar时间回退
      // if (timestamp_last_lidar_ > timestamp) {        
      //   diff_t_lidar_ = timestamp_last_lidar_ - timestamp;
      //   if (diff_t_lidar_ > uint64_t((30000000000))) {
      //     printf("WOC Lidar!!%f\n", (float)(diff_t_lidar_ / 1e9));
      //     if (diff_t_lidar_ > 39809238968) {
      //       cnt_lidar_ = ((int)((float)diff_t_lidar_ / 40000000000.0));
      //       printf("Lidar drop!!%d\n", (int)cnt_lidar_);
      //       timestamp = timestamp + (cnt_lidar_ + 1) * 40000000000;
      //     } else {
      //       timestamp = timestamp;
      //     } 
      //   }
      // }

        if (timestamp_last_lidar_ > timestamp) {
          diff_t_lidar_ = timestamp_last_lidar_ - timestamp;
          if (diff_t_lidar_ > uint64_t((5000000))) {//回退超过5ms（20hz）
              printf("WOC Lidar!!%f\n", (float)(diff_t_lidar_ / 1e9));
              cnt_lidar_ = ((int)((float)diff_t_lidar_ / 5000000.0));
              timestamp = timestamp + cnt_lidar_  * 5000000;
            }
          }

      cloud.header.stamp = ros::Time(timestamp / 1000000000.0);
      timestamp_last_lidar_ = timestamp;
    }


    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(lidar->raw_data_type);
      if (pf_point_convert) {
        point_base = pf_point_convert(point_base, raw_packet,
                                      lidar->extrinsic_parameter, line_num);
      } else {
        /** Skip the packet */
        ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      point_base = LivoxPointToPxyzrtl(point_base, raw_packet,
                                       lidar->extrinsic_parameter, line_num);
    }

    if (!is_zero_packet) {
      QueuePopUpdate(queue);
    } else {
      is_zero_packet = 0;
    }
    cloud.width += single_point_num;
    ++published_packet;
    last_timestamp = timestamp;
  }
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step); /** Adjust to the real size */
  ros::Publisher *p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
                  cloud);
    }
  }
  if (!lidar->data_is_pubulished) {
    lidar->data_is_pubulished = true;
  }
  return published_packet;
}

//将 Livox 点云数据转换成 PCL 点云数据
void Lddc::FillPointsToPclMsg(PointCloud::Ptr &pcl_msg,
                              LivoxPointXyzrtl *src_point, uint32_t num) {
  LivoxPointXyzrtl *point_xyzrtl = (LivoxPointXyzrtl *)src_point;
  for (uint32_t i = 0; i < num; i++) {
    pcl::PointXYZI point;
    point.x = point_xyzrtl->x;
    point.y = point_xyzrtl->y;
    point.z = point_xyzrtl->z;
    point.intensity = point_xyzrtl->reflectivity;
    ++point_xyzrtl;
    pcl_msg->points.push_back(point);
  }
}

/* for pcl::pxyzi */
// TODO pcl
uint32_t Lddc::PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                     uint8_t handle) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id.assign(frame_id_);
  cloud->height = 1;
  cloud->width = 0;

  uint8_t point_buf[2048];
  uint32_t is_zero_packet = 0;
  uint8_t data_source = lidar->data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
  while ((published_packet < packet_num) && !QueueIsEmpty(queue)) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if ((packet_gap > lidar->packet_interval_max) &&
        lidar->data_is_pubulished) {
      // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle,
      // packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + lidar->packet_interval;
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
      }
    }
    if (!published_packet) {
      cloud->header.stamp = timestamp / 1000.0; // to pcl ros time stamp
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(lidar->raw_data_type);
      if (pf_point_convert) {
        pf_point_convert(point_buf, raw_packet, lidar->extrinsic_parameter,
                         line_num);
      } else {
        /* Skip the packet */
        ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      LivoxPointToPxyzrtl(point_buf, raw_packet, lidar->extrinsic_parameter,
                          line_num);
    }
    LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
    FillPointsToPclMsg(cloud, dst_point, single_point_num);
    if (!is_zero_packet) {
      QueuePopUpdate(queue);
    } else {
      is_zero_packet = 0;
    }
    cloud->width += single_point_num;
    ++published_packet;
    last_timestamp = timestamp;
  }

  ros::Publisher *p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
                  cloud);
    }
  }
  if (!lidar->data_is_pubulished) {
    lidar->data_is_pubulished = true;
  }
  return published_packet;
}

void Lddc::FillPointsToCustomMsg(livox_ros_driver::CustomMsg &livox_msg,
                                 LivoxPointXyzrtl *src_point, uint32_t num,
                                 uint32_t offset_time, uint32_t point_interval,
                                 uint32_t echo_num) {
  LivoxPointXyzrtl *point_xyzrtl = (LivoxPointXyzrtl *)src_point;
  for (uint32_t i = 0; i < num; i++) {
    livox_ros_driver::CustomPoint point;
    if (echo_num > 1) { /** dual return mode */
      point.offset_time = offset_time + (i / echo_num) * point_interval;
    } else {
      point.offset_time = offset_time + i * point_interval;
    }
    point.x = point_xyzrtl->x;
    point.y = point_xyzrtl->y;
    point.z = point_xyzrtl->z;
    point.reflectivity = point_xyzrtl->reflectivity;
    point.tag = point_xyzrtl->tag;
    point.line = point_xyzrtl->line;
    ++point_xyzrtl;
    livox_msg.points.push_back(point);
  }
}

uint64_t cur_lidar_time = -1;
double cur_lidar_time_sec = -1;
//packet_num：每次发布的数据包数量
//函数的主要功能是从队列中获取数据包，并将数据包转换为自定义消息格式，然后发布出去。
uint32_t Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint32_t packet_num, uint8_t handle) {
  //printf("\n enter : PublishCustomPointcloud()\n");//进入了。
  static uint32_t msg_seq = 0;
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  //获取即将要发布的数据（storage_packet）的起始时间（last_timestamp）。
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    //跳过后，队列中的剩余数据包可能不够
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  livox_ros_driver::CustomMsg livox_msg;
  livox_msg.header.frame_id.assign(frame_id_);
  livox_msg.header.seq = msg_seq;
  ++msg_seq;
  livox_msg.timebase = 0;
  livox_msg.point_num = 0;
  livox_msg.lidar_id = handle;

  uint8_t point_buf[2048];
  uint8_t data_source = lds_->lidars_[handle].data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
  uint32_t point_interval = GetPointInterval(lidar->info.type);
  uint32_t published_packet = 0;
  uint32_t packet_offset_time = 0; /** uint:ns */
  uint32_t is_zero_packet = 0;

  // 在一个循环中，函数从队列中取出数据包，计算数据包的时间间隔（packet_gap）并判断是否需要进行时间间隔补偿。
  while (published_packet < packet_num) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if ((packet_gap > lidar->packet_interval_max) && lidar->data_is_pubulished) {
      // ROS_INFO("Lidar[%d] packet time interval is %ldns", handle, packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + lidar->packet_interval;
        ZeroPointDataOfStoragePacket(&storage_packet);//将存储数据包（StoragePacket）中的点云数据清零
        is_zero_packet = 1;
      }
    }

    /** first packet */
    //在处理第一个数据包时，函数设置消息的时间基准（timebase）和时间戳（timestamp），并更新其他相关变量。
    if (!published_packet) {
      livox_msg.timebase = timestamp;
      packet_offset_time = 0;
      /** convert to ros time stamp */
      // timestamp_last
      // 如果出现回退，那就计算一下 回退时间
      // TODO CustomPointcloud
      if (timestamp_last_lidar_ > timestamp) {
        diff_t_lidar_ = timestamp_last_lidar_ - timestamp;
        if (diff_t_lidar_ > uint64_t((30000000000))) {//30S
          printf("WOC Lidar!!%f\n", (float)(diff_t_lidar_ / 1e9));

          // float counter = (float)(((float)diff_t_lidar_) / (40 * 1e9));
          // printf("Lidar drop!!%f\n", counter);
          //
        }
      }

      if (diff_t_lidar_ > 39809238968) {
        cnt_lidar_ = ((int)((float)diff_t_lidar_ / 40000000000.0));
        printf("Lidar drop!!%d\n", (int)cnt_lidar_);
        timestamp = timestamp + (cnt_lidar_ + 1) * 40000000000;
      } else {
        timestamp = timestamp;
      }
      
      //pointt->low = timestamp;//这里写入共享内存文件！
      //printf("livox 写入共享内存文件的timestamp:  %ld \n",timestamp);

      //
      // timestamp = timestamp + (counter + 1) * 40000000000;

      /** convert to ros time stamp */
      cur_lidar_time = timestamp;
      cur_lidar_time_sec = timestamp / 1000000000.0;
      time_offset_ = time_pc_ - cur_lidar_time_sec;

      livox_msg.header.stamp =
          ros::Time(timestamp / 1000000000.0); // + time_offset_);

      // livox_msg.header.stamp = ros::Time(timestamp / 1000000000.0);
      timestamp_last_lidar_ = timestamp;
    } else {
      packet_offset_time = (uint32_t)(timestamp - livox_msg.timebase);
    }

    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(lidar->raw_data_type);
      if (pf_point_convert) {
        pf_point_convert(point_buf, raw_packet, lidar->extrinsic_parameter,
                         line_num);
      } else {
        /* Skip the packet */
        ROS_INFO("Lidar[%d] unkown packet type[%d]", handle,
                 lidar->raw_data_type);
        break;
      }
    } else {
      LivoxPointToPxyzrtl(point_buf, raw_packet, lidar->extrinsic_parameter, line_num);
    }

    LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
    FillPointsToCustomMsg(livox_msg, dst_point, single_point_num,
                          packet_offset_time, point_interval, echo_num);

    if (!is_zero_packet) {
      QueuePopUpdate(queue);
    } else {
      is_zero_packet = 0;
    }

    livox_msg.point_num += single_point_num;
    last_timestamp = timestamp;
    ++published_packet;
  }

  ros::Publisher *p_publisher = Lddc::GetCurrentPublisher(handle);
  if (kOutputToRos == output_type_) {
    p_publisher->publish(livox_msg);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
                  livox_msg);
    }
  }

  if (!lidar->data_is_pubulished) {
    lidar->data_is_pubulished = true;
  }
  return published_packet;
}

// 发布IMU数据

uint32_t Lddc::PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle) {
  uint64_t timestamp = 0;
  uint32_t published_packet = 0;
  sensor_msgs::Imu imu_data;
  imu_data.header.frame_id = "livox_frame";
  static bool skip_this_imu = false;
  static uint64_t timestamp_last_imu_ = 0;
  uint8_t data_source = lds_->lidars_[handle].data_src;
  StoragePacket storage_packet;
  QueuePrePop(queue, &storage_packet);
  LivoxEthPacket *raw_packet = reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
  timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
  if (timestamp >= 0) {
    if(raw_packet->timestamp_type == 3){
  	  if((timestamp - timestamp_last_imu_) / 1000000.0 < 4.5){//当前帧与上一帧的时间差值小于4.5ms，就是多出来的一帧，剔除掉。(正常应该是5ms)
        skip_this_imu = true;//跳过这帧数据。
      }
      else if (timestamp_last_imu_ > timestamp) {
        printf("------------发生时间回退!----------------\n");//现在不会出现时间回退了！
        printf("回退时间为： %f\n", (timestamp_last_imu_ - timestamp)/1000000000.0);
	    }           
      imu_data.header.stamp = ros::Time(timestamp / 1000000000.0); //秒
    }
  }

  uint8_t point_buf[2048];
  LivoxImuDataProcess(point_buf, raw_packet);
  LivoxImuPoint *imu = (LivoxImuPoint *)point_buf;
  imu_data.angular_velocity.x = imu->gyro_x;
  imu_data.angular_velocity.y = imu->gyro_y;
  imu_data.angular_velocity.z = imu->gyro_z;
  imu_data.linear_acceleration.x = imu->acc_x;
  imu_data.linear_acceleration.y = imu->acc_y;
  imu_data.linear_acceleration.z = imu->acc_z;
  QueuePopUpdate(queue);
  ++published_packet;
  ros::Publisher *p_publisher = Lddc::GetCurrentImuPublisher(handle);

  if (kOutputToRos == output_type_) {
    if(skip_this_imu == false){//不要跳过当前帧，就发布出去。
      timestamp_last_imu_ = timestamp;
      p_publisher->publish(imu_data);
    }else{//跳过当前帧imu数据。
      published_packet--;
      skip_this_imu = false;
    }
  } else {
    if (bag_ && enable_imu_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
          imu_data);
    }
  }
  return published_packet;
}

/*
uint32_t Lddc::PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle) {
  volatile uint64_t timestamp = 0;
  uint32_t published_packet = 0;
  static uint64_t timestamp_last_imu_ = 0;
  static bool skip_this_imu = false;
  bool need_imu_interpolation = false;//判断是否需要插值
  sensor_msgs::Imu imu_data;
  imu_data.header.frame_id = "livox_frame";
  uint8_t data_source = lds_->lidars_[handle].data_src;
  StoragePacket storage_packet;
  QueuePrePop(queue, &storage_packet);
  LivoxEthPacket *raw_packet = reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
  
  // static auto start1 = std::chrono::high_resolution_clock::now();
  // LdsStamp timestamp_temp;
  // memcpy(timestamp_temp.stamp_bytes, raw_packet->timestamp, sizeof(timestamp_temp));
  // static uint64_t last_timestamp_stamp_word_high = timestamp_temp.stamp_word.high;
  // if(timestamp_temp.stamp_word.high - last_timestamp_stamp_word_high >= 1000000){
  //   last_timestamp_stamp_word_high = timestamp_temp.stamp_word.high;
  //   printf("!!!!!!!!!!!!!livox内部计时已经达到1秒了!!!!!!!!!!!!!\n");
  //   //用电脑时间验证，livox内部计时确实是准确的1s。（误差不到半ms）。所以，是stm32的计时不准确。
  //   auto end1 = std::chrono::high_resolution_clock::now();
  //   std::chrono::duration<double> duration1 = end1 - start1;
  //   printf("PC计时器是否也是一秒?(希望是1s): %f s\n",duration1.count());
  //   start1 = std::chrono::high_resolution_clock::now();//重新开始计时
  // }


  timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
  //当前帧imu的时间戳，这里是用livox内部时钟计算的时间(纳秒部分)
  //收到pps信号后，这里的timestamp就从0开始计时。

    if (timestamp >= 0) {
      if(raw_packet->timestamp_type == 3){//如果使用pps+gps同步的话，才使用这里的时间戳修正。//注意：刚开始运行的时候是仅pps，过两秒才会变为pps+gps的状态。
        if((timestamp - timestamp_last_imu_) / 1000000.0 < 4.5){//当前帧与上一帧的时间差值dt小于4.5ms，就是多出来的一帧，剔除掉。(正常应该是5ms)
          skip_this_imu = true;//跳过这帧数据。
        }
        else if((timestamp - timestamp_last_imu_) / 1000000.0 > 5.5){
          //这里产生了6ms或9ms的帧，用这个6ms的帧和上一帧做插值，得到5ms处的帧。更改这帧的陀螺仪/加速度计和时间戳。
          need_imu_interpolation = true;       
        }
        if (timestamp_last_imu_ > timestamp) {//时间回退了问题已经解决了。
          //如果系统时间不稳定或者发生了突然的时间调整，可能会导致 IMU 数据的时间戳出现异常。
          printf("------------发生时间回退!----------------\n");
          printf("回退的时间为 :  %f s \n", (timestamp_last_imu_ - timestamp)/1000000000.0);
          //printf("time = %lu\n", timestamp);
          //每隔20s就会发生一次，然后这样设置时间戳就会导致每隔20s就有1s的时间戳几乎重叠。
          // diff_t_imu_ = timestamp_last_imu_ - timestamp;
          // if (diff_t_imu_ > uint64_t((5000000))) {//回退超过5ms（20hz）
          //     printf("WOC IMU!!%f\n", (float)(diff_t_imu_ / 1e9));
          //     cnt_imu_ = ((int)((float)diff_t_imu_ / 5000000.0));
          //     timestamp = timestamp + (cnt_imu_+1)  * 5000000;
          //   }
          }
          // if(timestamp - timestamp_last_imu_ < 2000000){//当前imu帧的时间戳与上一帧的时间差小于2ms，就把当前帧丢掉。？
          //   printf("timestamp %lu ,timestamp_last_imu_ = %lu, dt<2ms\n",timestamp,timestamp_last_imu_ );
          // }
        //过几秒后，在接受到一个gprmc的时候，就发生一次时间回退，退了1秒。再接受到gprmc消息后，生成的时间戳会重叠。
        // if (timestamp_last_imu_ > timestamp) {
        //   diff_t_imu_ = timestamp_last_imu_ - timestamp;
        //   if (diff_t_imu_ > uint64_t((30000000000))) {
        //     printf("WOC IMU!!%f\n", (float)(diff_t_imu_ / 1e9));
        //     if (diff_t_imu_ > 39809238968) {
        //       cnt_imu_ = ((int)((float)diff_t_imu_ / 40000000000.0));
        //       printf("IMU drop!!%d\n", (int)cnt_imu_);
        //       timestamp = timestamp + (cnt_imu_ + 1) * 40000000000;
        //     }
        //   }
        // }
        // timestamp_last_imu_ = timestamp;
  }

  uint8_t point_buf[2048];
  LivoxImuDataProcess(point_buf, raw_packet);//从包中获取当前帧的imu数值
  LivoxImuPoint *current_imu = (LivoxImuPoint *)point_buf;
  static LivoxImuPoint last_imu = *current_imu; // 进行结构体的深拷贝(创建一个新的结构体last_imu_value，存放上一帧imu数值）
  bool need_pub_imu_data_2 = false;  
  uint64_t imu_data_2_timestamp = 0;
  sensor_msgs::Imu imu_data_2;
  if(need_imu_interpolation) {//需要imu插值
    if((timestamp - timestamp_last_imu_) / 1000000.0 > 8.0){//理论是10ms
      //这里是考虑到rqt中连续两帧的时间差异dt为6ms和9ms的时候，要把9ms处的那个数据也发布出去。实验结果：dt几乎全部为5ms，但是这个方案不可行。因为imu数据分布不规律。
      need_pub_imu_data_2 = true;
      imu_data_2_timestamp = timestamp;
      //要把当前帧也发布出去。并更新timestamp_last_imu_为当前帧的时间戳。
      imu_data_2.header.frame_id = "livox_frame";
      imu_data_2.header.stamp = ros::Time(timestamp / 1000000000.0); //  seconds
      imu_data_2.angular_velocity.x = current_imu->gyro_x;
      imu_data_2.angular_velocity.y = current_imu->gyro_y;
      imu_data_2.angular_velocity.z = current_imu->gyro_z;
      imu_data_2.linear_acceleration.x = current_imu->acc_x;
      imu_data_2.linear_acceleration.y = current_imu->acc_y;
      imu_data_2.linear_acceleration.z = current_imu->acc_z;
    }
    uint64_t real_timestamp = timestamp;//这帧imu的实际时间戳（与上一帧的dt>5.5ms）
    uint64_t after_interpolation_timestamp = timestamp_last_imu_   + 5000000.0;//插值后的时间戳
    float lamda = 1.0 * (after_interpolation_timestamp - timestamp_last_imu_) / (real_timestamp - timestamp_last_imu_);
    current_imu->gyro_x = (1-lamda) * last_imu.gyro_x  + lamda * current_imu->gyro_x ;
    current_imu->gyro_y = (1-lamda) * last_imu.gyro_y  + lamda * current_imu->gyro_y ;
    current_imu->gyro_z = (1-lamda) * last_imu.gyro_z  + lamda * current_imu->gyro_z ;
    current_imu->acc_x = (1-lamda) * last_imu.acc_x  + lamda * current_imu->acc_x ;
    current_imu->acc_y = (1-lamda) * last_imu.acc_y  + lamda * current_imu->acc_y ;
    current_imu->acc_z = (1-lamda) * last_imu.acc_z  + lamda * current_imu->acc_z ;
    timestamp = after_interpolation_timestamp;//插值之后的时间戳，后面作为插值帧的时间戳发布出去
    need_imu_interpolation = false;//更新标志位
  }
    imu_data.header.stamp = ros::Time(timestamp / 1000000000.0); //  seconds
    imu_data.angular_velocity.x = current_imu->gyro_x;
    imu_data.angular_velocity.y = current_imu->gyro_y;
    imu_data.angular_velocity.z = current_imu->gyro_z;
    imu_data.linear_acceleration.x = current_imu->acc_x;
    imu_data.linear_acceleration.y = current_imu->acc_y;
    imu_data.linear_acceleration.z = current_imu->acc_z;
    last_imu = *current_imu;//更新last_imu（插值之后的imu数值），用于下一轮插值
  QueuePopUpdate(queue);//读取的位置++，发布一次数据就要++一次。
  ++published_packet;
  ros::Publisher *p_publisher = Lddc::GetCurrentImuPublisher(handle);
  if (kOutputToRos == output_type_){
    if(skip_this_imu == false){//不要跳过当前帧，就发布出去。
        p_publisher->publish(imu_data);
        if(need_pub_imu_data_2 == true){
          need_pub_imu_data_2 = false;
          timestamp_last_imu_ = imu_data_2_timestamp;
          p_publisher->publish(imu_data_2);
        }else{
          timestamp_last_imu_ = timestamp;
        }
    }else{//跳过当前帧imu数据。
      published_packet--;
      skip_this_imu = false;
    }
  } else {
    if (bag_ && enable_imu_bag_) {
      bag_->write(p_publisher->getTopic(), ros::Time(timestamp / 1000000000.0),
                  imu_data);
    }
  }
  //printf("return published_packet: %d\n", published_packet);//打印的都是1
  return published_packet;
}
*/

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

//轮询激光雷达的点云数据，并根据指定的传输格式（transfer_format_）进行发布
void Lddc::PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;

  //如果数据队列中没有存储数据包，则直接返回
  if (p_queue->storage_packet == nullptr) {
    return;
  }

  //只要数据队列中的未使用数据包数量大于等于每次发布的数据包数量（onetime_publish_packets），就会进入数据转换和发布的流程。
  while (!QueueIsEmpty(p_queue)) {
    uint32_t used_size = QueueUsedSize(p_queue);
    uint32_t onetime_publish_packets = lidar->onetime_publish_packets;
    if (used_size < onetime_publish_packets) {
      break;
    }

    //根据指定的传输格式 transfer_format_ ，分别调用不同的函数来发布点云数据。
//   typedef enum {
//   kPointCloud2Msg = 0,
//   kLivoxCustomMsg = 1,
//   kPclPxyziMsg
// } TransferType;

    if (kPointCloud2Msg == transfer_format_) {//发布PointCloud2类型消息，就调用这个
      PublishPointcloud2(p_queue, onetime_publish_packets, handle);
    } else if (kLivoxCustomMsg == transfer_format_) {//自定义的话，就调用这个
      PublishCustomPointcloud(p_queue, onetime_publish_packets, handle);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPointcloudData(p_queue, onetime_publish_packets, handle);
    }
  }
}

//轮询激光雷达的 IMU 数据，并将其发布到 ROS 中。
void Lddc::PollingLidarImuData(uint8_t handle, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->imu_data;
  if (p_queue->storage_packet == nullptr) {
    return;
  }

  while (!QueueIsEmpty(p_queue)) {
    PublishImuData(p_queue, 1, handle);//将队列中的 IMU 数据包发布到 ROS 中
  }
}

//通过轮询各个激光雷达的数据队列，并将数据转换为指定的传输格式，最终发布到ROS中。
void Lddc::DistributeLidarData(void) {
  //livox_ros_driver.cpp 一直while循环这个函数
  //   while (ros::ok()) {
  //   lddc->DistributeLidarData();
  // }

  
  if (lds_ == nullptr) {//激光雷达设备结构体 
    return;
  }
  lds_->semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    
    //如果当前激光雷达的连接状态不是采样状态或数据队列为空，则继续处理下一个激光雷达。
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }

    //计算（自纪元以来）以秒为单位的时间戳 time_pc_
    auto time_pc_clk = std::chrono::high_resolution_clock::now();
    time_pc_ = uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(time_pc_clk.time_since_epoch()).count()) / 1000000000.0;

    //轮询激光雷达的点云数据，并根据指定的传输格式（transfer_format_）进行发布
    PollingLidarPointCloudData(lidar_id, lidar);

    //轮询激光雷达的 IMU 数据，并将其发布到 ROS 中。
    PollingLidarImuData(lidar_id, lidar);

  }

  if (lds_->IsRequestExit()) {
    PrepareExit();
  }
}

ros::Publisher *Lddc::GetCurrentPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               lds_->lidars_[handle].info.broadcast_code);
      ROS_INFO("Support multi topics.");
    } else {
      ROS_INFO("Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/lidar");
    }

    *pub = new ros::Publisher;
    if (kPointCloud2Msg == transfer_format_) {
      **pub =
          cur_node_->advertise<sensor_msgs::PointCloud2>(name_str, queue_size);
      ROS_INFO(
          "%s publish use PointCloud2 format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kLivoxCustomMsg == transfer_format_) {
      **pub = cur_node_->advertise<livox_ros_driver::CustomMsg>(name_str,
                                                                queue_size);
      ROS_INFO(
          "%s publish use livox custom format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kPclPxyziMsg == transfer_format_) {
      **pub = cur_node_->advertise<PointCloud>(name_str, queue_size);
      ROS_INFO("%s publish use pcl PointXYZI format, set ROS publisher queue "
               "size %d",
               name_str, queue_size);
    }
  }

  return *pub;
}

ros::Publisher *Lddc::GetCurrentImuPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_imu_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_imu_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      ROS_INFO("Support multi topics.");
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
               lds_->lidars_[handle].info.broadcast_code);
    } else {
      ROS_INFO("Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/imu");
    }

    *pub = new ros::Publisher;
    **pub = cur_node_->advertise<sensor_msgs::Imu>(name_str, queue_size);
    ROS_INFO("%s publish imu data, set ROS publisher queue size %d", name_str,
             queue_size);
  }

  return *pub;
}

void Lddc::CreateBagFile(const std::string &file_name) {
  if (!bag_) {
    bag_ = new rosbag::Bag;
    bag_->open(file_name, rosbag::bagmode::Write);
    ROS_INFO("Create bag file :%s!", file_name.c_str());
  }
}

void Lddc::PrepareExit(void) {
  if (bag_) {
    ROS_INFO("Waiting to save the bag file!");
    bag_->close();
    ROS_INFO("Save the bag file successfully!");
    bag_ = nullptr;
  }
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

} // namespace livox_ros
