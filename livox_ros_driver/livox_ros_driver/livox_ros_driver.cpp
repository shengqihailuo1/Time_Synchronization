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

#include "include/livox_ros_driver.h"

#include <chrono>
#include <vector>
#include <csignal>

#include <ros/ros.h>
#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"

using namespace livox_ros;

const int32_t kSdkVersionMajorLimit = 2;

inline void SignalHandler(int signum) {
  printf("livox ros driver will exit\r\n");
  ros::shutdown();
  exit(signum);
}

struct time_stamp {
  int64_t high;
  int64_t low;
};
struct time_stamp *pointt;

#define ERR_EXIT(m)                                                            \
  do {                                                                         \
    perror(m);                                                                 \
    exit(EXIT_FAILURE);                                                        \
  } while (0)

#include <boost/asio.hpp>

int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle livox_node;

  // ros::NodeHandle image_time_reference_node;  // 创建 ROS 节点句柄,用于发布数据到相机。
  // ros::Publisher time_reference_pub = image_time_reference_node.advertise<sensor_msgs::TimeReference>("/mvs_camera_trigger/image_time_reference", 1);


  // //设置相机采样频率可控
  // int camera_publish_freq = 40;
  // livox_node.getParam("camera_publish_freq", camera_publish_freq);
  // printf(" camera_publish_freq: %d \n",camera_publish_freq);
  // //把相机采样频率 camera_publish_freq 通过串口发送给stm32


  ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);
  signal(SIGINT, SignalHandler);
  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
             _sdkversion.minor, _sdkversion.patch);
    return 0;
  }

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;//这个是默认的，我改成：kLivoxCustomMsg，这样才能执行“把时间戳写到共享文件中“
  //int xfer_format = kLivoxCustomMsg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;
  //std::string path_for_time_stamp = "/home/aa/test4";//原来的
  std::string path_for_time_stamp = "/home/sqhl/Desktops/fast_livo_catkin_ws/src/time_stamp_file";
  livox_node.getParam("xfer_format", xfer_format);
  livox_node.getParam("multi_topic", multi_topic);
  livox_node.getParam("data_src", data_src);
  livox_node.getParam("publish_freq", publish_freq);
  livox_node.getParam("output_data_type", output_type);
  livox_node.getParam("frame_id", frame_id);
  livox_node.getParam("enable_lidar_bag", lidar_bag);
  livox_node.getParam("enable_imu_bag", imu_bag);
  livox_node.getParam("path_for_time_stamp", path_for_time_stamp);
  const char *shared_file_name = path_for_time_stamp.c_str();


  //方案二用不到共享内存。所以注释掉。
  // printf("start open file : %s\n",shared_file_name);
  // int fd = open(shared_file_name, O_CREAT | O_RDWR | O_TRUNC, 0666);
  // if (fd == -1) {
  //   ERR_EXIT("open");
  // } else {
  //   printf("open code: %d\n", fd);
  // }
  
  // lseek(fd, sizeof(time_stamp) * 1, SEEK_SET);//将文件描述符 fd 偏移量设置为 sizeof(time_stamp) 的大小。
  // write(fd, "", 1);//将一个字节的空数据写入文件中。这是为了确保在 mmap 函数调用之前文件有足够的空间。
  // //mmap将文件映射到进程的内存空间中。这样可以方便地通过指针 pointt 访问文件的内容
  // pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp) * 1, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);


  if (publish_freq > 100.0) {//限制发布频率
    publish_freq = 100.0;
  } else if (publish_freq < 0.1) {
    publish_freq = 0.1;
  } else {
    publish_freq = publish_freq;
  }

  /** Lidar data distribute control and lidar data source set */
  /**激光雷达数据分发控制和激光雷达数据源集*/
  Lddc *lddc = new Lddc(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  lddc->SetRosNode(&livox_node);

  int ret = 0;
  if (data_src == kSourceRawLidar) {//我的设备跑这里的代码，后面的kSourceRawHub是hub设备的，最后的是lvx文件。
    ROS_INFO("Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    ROS_INFO("Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.getParam("cmdline_str", cmdline_bd_code);//就是从launch文件中读取到的广播码
    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    //实例化LdsLidar类对象。形参：发布数据的时间间隔（毫秒）
    LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);

    //将 read_lidar 对象转换为 Lds 类型并将其注册到 Lddc 类中，以便在后续数据处理过程中能够使用该对象获取激光雷达数据。
    lddc->RegisterLds(static_cast<Lds *>(read_lidar));

    ret = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());//里面解析livox_lidar_config.json
    if (!ret) {
      ROS_INFO("Init lds lidar success!");//跑到这里了。
    } else {
      ROS_ERROR("Init lds lidar fail!");
    }
  } else if (data_src == kSourceRawHub) {
    ROS_INFO("Data Source is hub.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    ROS_INFO("Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.getParam("cmdline_str", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_hub));
    ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
    if (!ret) {
      ROS_INFO("Init lds hub success!");
    } else {
      ROS_ERROR("Init lds hub fail!");
    }
  } else {
    ROS_INFO("Data Source is lvx file.");

    std::string cmdline_file_path;
    livox_node.getParam("cmdline_file_path", cmdline_file_path);

    do {
      if (!IsFilePathValid(cmdline_file_path.c_str())) {
        ROS_ERROR("File path invalid : %s !", cmdline_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = cmdline_file_path.find_last_of('.');
      rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
      lddc->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc->CreateBagFile(rosbag_file_path);
      int ret = read_lvx->InitLdsLvx(cmdline_file_path.c_str());
      if (!ret) {
        ROS_INFO("Init lds lvx file success!");
      } else {
        ROS_ERROR("Init lds lvx file fail!");
      }
    } while (0);
  }

  ros::Time::init();
  while (ros::ok()) {
    lddc->DistributeLidarData();
  }
  munmap(pointt, sizeof(time_stamp) * 5);
  return 0;
}
