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


#ifndef SYNCHRO_H_
#define SYNCHRO_H_

#include <stdint.h>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>

#ifdef WIN32
#include <Windows.h>
#endif

namespace livox_ros {

typedef void (*FnReceiveSyncTimeCb)(const char *rmc, uint32_t rmc_length,
                                    void *client_data);
typedef void (*UtcTimerCallback)(const char* rmc, uint16_t rmc_length);

# define READ_BUF (256)

enum Parity {
  P_8N1,    /* No parity (8N1)	*/     
  P_7E1,    /* Even parity (7E1)*/
  P_7O1,    /* Odd parity (7O1)	*/
  P_7S1     /* Space parity is setup the same as no parity (7S1)	*/
};

enum BaudRate {
  BR2400,
  BR4800,
  BR9600,
  BR19200,
  BR38400,
  BR57600,
  BR115200,
  BR230400,
  BR460800,
  BR500000,
  BR576000,
  BR921600,
  BR1152000,
  BR1500000,
  BR2000000,
  BR2500000,
  BR3000000,
  BR3500000,
  BR4000000,
};


class Synchro {

public:
  Synchro();
  ~Synchro();

  static Synchro& GetInstance() {
    static Synchro synchro;
    return synchro;
  }
  
  bool Start();
  void Stop();
  void SetBaudRate(BaudRate baudrate);
  void SetPortName(std::string port_name);
  void SetParity(Parity parity);
  void SetSyncTimerCallback(FnReceiveSyncTimeCb cb, void *data);
  void Set_camera_publish_freq(int freq);



private:
  int Setup(enum BaudRate baud, enum Parity parity);
  void Close();
  bool Open();
  void IOLoop();
  size_t Read(uint8_t *buffer, size_t size);
  void Decode(uint8_t* buff, size_t size);
  bool ParseGps(uint8_t in_byte);
  void Clear();

  bool Send_Image_Time(const char* cam_str);

  bool validate_checksum(const char *nmea_sentence);
  int calculate_checksum(const char *data);




#ifdef WIN32
  HANDLE  hfile_;
#else
  int fd_;
#endif

  enum BaudRate baudrate_;
  enum Parity parity_;
  std::string port_name_;
  int camera_publish_freq_;

  FnReceiveSyncTimeCb sync_timer_cb_;
  void *client_data_;

  bool is_quite_;
  std::shared_ptr<std::thread> listener_;

  std::vector<char> rmc_buff_;//gprmc数据的缓存区
  std::vector<char> cam_buff_;//cam相机数据的缓存区


  ros::NodeHandle nh_image_time_reference;  // 创建 ROS 节点句柄,用于发布数据到相机。
  ros::Publisher time_reference_pub = nh_image_time_reference.advertise<sensor_msgs::TimeReference>("/mvs_camera_trigger/image_time_reference", 1);



  uint16_t cur_len_ = 0;//zcw，添加“= 0“

  int year, month, day, hour, minute, second, ms;
  int num = 0;
  uint64_t time_epoch,total_seconds,total_nanoseconds;

};

#endif
}
