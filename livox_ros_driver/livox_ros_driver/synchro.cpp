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
#include <iostream>

#include <algorithm> 
#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>


#include "synchro.h"
#include <stdio.h>
#include <string.h>

#include <boost/asio.hpp>

#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <fcntl.h>
#endif

#include <ctime>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

namespace livox_ros {

struct my_t {
  int64_t gps_tt;
  //std::chrono::high_resolution_clock::time_point pc_tt;
  long pc_tt;//自1970年到当前接受到gps时刻所经过的ns数量。
};
static my_t *pointt_clk; // 共享内存指针,用于实现时间戳同步
static int fd;

#ifdef WIN32
Synchro::Synchro() {
  hfile_ = INVALID_HANDLE_VALUE;
  is_quite_ = false;
  rmc_buff_.resize(128,0);
  cam_buff_.resize(128,0);
}
#else
Synchro::Synchro() {
  fd_ = 0; 
  is_quite_ = false;
  rmc_buff_.resize(128,0);
  cam_buff_.resize(128,0);
}
#endif

Synchro::~Synchro() {
  Stop();
}

void Synchro::Set_camera_publish_freq(int freq)
{
  camera_publish_freq_ = freq;
}

void Synchro::SetBaudRate(BaudRate baudrate) {
  baudrate_ = baudrate;
}

void Synchro::SetParity(Parity parity) {
  parity_ = parity;
}

void Synchro::SetPortName(std::string port_name) {
  port_name_ = port_name;
}

bool Synchro::Start() {//开始执行函数

  if (!Open()) {
    return false;
  }
  is_quite_ = false;//退出 false
  //创建 std::shared_ptr 对象，可以确保线程对象在不再需要时能够正确地释放资源
  //&Synchro::IOLoop 表示要在新线程中执行的成员函数，this 则表示当前的 Synchro 对象，确保在新线程中可以访问当前对象的成员变量和方法。
  listener_ = std::make_shared<std::thread>(&Synchro::IOLoop, this);
  return true;
}

void Synchro::Stop() {
  is_quite_ = true;
  if (listener_) {
    listener_->join();
    listener_ = nullptr;
  }
  Close();
}

void Synchro::IOLoop() {//线程里面一直执行这函数
  uint8_t buff[READ_BUF];
  size_t size = 0;
  while (!is_quite_) {//没有退出就一直循环
    //从串口中读取数据，将读取到的数据存储到 buff 中，读取的最大字节数为 READ_BUF
    if (0 != (size = Read(buff, READ_BUF))) {//实际读取到的字节数size
      Decode(buff, size);
    }
  }
}

// 计算校验和
int Synchro::calculate_checksum(const char *data) {
    // 提取数据部分（不包括起始符 '$' 和校验和部分 '*23'）
    const char *start = strchr(data, '$');
    const char *end = strchr(data, '*');
    if (start == NULL || end == NULL) {
        return -1;  // 错误：找不到起始符或结束符
    }
    int checksum = 0;
    for (const char *ptr = start + 1; ptr < end; ptr++) {
        checksum ^= *ptr;
    }
    return checksum;
}

// 验证校验和
bool Synchro::validate_checksum(const char *nmea_sentence) {
    // 提取校验和部分
    const char *checksum_str = strchr(nmea_sentence, '*');
    if (checksum_str == NULL) {
        return 0;  // 格式错误：找不到校验和部分
    }
    int received_checksum = strtol(checksum_str + 1, NULL, 16);
    // 计算数据部分的校验和
    int calculated_checksum = calculate_checksum(nmea_sentence);
    // 比较校验和
    if (received_checksum == calculated_checksum) {
        return 1;  // 校验通过
    } else {
        return 0;  // 校验未通过
    }
}

/*
    该方法用于逐字节解析 GPS 数据，将输入的字节存储在 rmc_buff_ 缓冲区中，并检查是否接收完整的 $GPRMC 或 $GNRMC 消息。
    如果接收的消息完整，且校验通过，则返回 true，表示成功解析出一条完整的 GPS 消息；否则返回 false。
*/
/*
bool Synchro::ParseGps(uint8_t in_byte) {
  std::string str_to_find = "$cam";
  if (cur_len_ < strlen("$GPRMC")) {
    rmc_buff_[0] = rmc_buff_[1];
    rmc_buff_[1] = rmc_buff_[2];
    rmc_buff_[2] = rmc_buff_[3];
    rmc_buff_[3] = rmc_buff_[4];
    rmc_buff_[4] = rmc_buff_[5];
    rmc_buff_[5] = in_byte;
    cur_len_++;
    // if (cur_len_ == strlen("$GPRMC")) {//当前接收到了6个字符
    //   //strncmp：比较两个字符串的前 strlen("$GPRMC") 个字符是否相同，如果相同则返回 0；如果不同，则返回一个非零值。
    //   if (strncmp((const char*)&rmc_buff_[0], "$GPRMC", strlen("$GPRMC")) != 0 &&  \
    //       strncmp((const char*)&rmc_buff_[0], "$GNRMC", strlen("$GNRMC")) != 0) {
    //     //接受到的前6个字符不是"$GPRMC"，也不是"$GNRMC"，有误差
    //     cur_len_--;
    //   } 
    // }
    return false;//还没接受到完整的信息
  }

  //到这里就完成接受到消息段头"$GPRMC"了。

  //if (cur_len_ >= rmc_buff_.size()) {//128。一条完整的gprmc数据是75/80个字符
  if (cur_len_ >= 128){//81
    // printf("cur_len_: %d ,rmc_buff_.size(): %d \n", cur_len_,rmc_buff_.size());
    // printf("\n cur_len_ >= rmc_buff_.size() \n");
    Clear();//清除缓存区域的数据
    return false;
  }

  rmc_buff_[cur_len_] = in_byte;//把新接受到的字符存放到缓存rmc_buff_里
  
    
    if(rmc_buff_[cur_len_] == '#'){//接受到完整的cam数据了。
    printf("---222---\n");
    if (!rmc_buff_.empty() && rmc_buff_[0] == '\n') {
        rmc_buff_.erase(rmc_buff_.begin());
        cur_len_--;
    }

    for (const char& c : rmc_buff_) {
    printf("%c", c);
    }
    printf("\n");

        //rmc_buff_.erase(rmc_buff_.begin()); // 删除第一个元素（回车）
        //rmc_buff_.push_back('\0');
        if(Send_Image_Time(&rmc_buff_.at(0))){
          printf("-----发送image时间戳成功!------\n");
        }
        else{
          printf("-----发送image时间戳失败。-----\n");
        }
        Clear();//数据用完了，要清空掉。
        return false;
  }

  //if (rmc_buff_[cur_len_ - 3] == '.' && std::search(rmc_buff_.begin(), rmc_buff_.end(), str_to_find.begin(), str_to_find.end()) != rmc_buff_.end()) {
  //   std::string str(rmc_buff_.begin(), rmc_buff_.end());
  //   size_t pos = str.find("$cam");
  //   std::string subString = str.substr(pos);
  //   // printf("livox驱动收到来自stm32的相机时间戳数据:");
  //   // for (char c : subString) {
  //   // printf("%c" ,c);//$cam,001803.125
  //   // }
  //   // printf("\n");
  //   //ros::Time 的构造函数接受两个参数，分别是秒数和纳秒数
  //   int hour, minute, second,millisecond;
  //   sscanf(subString.c_str(), "$cam,%2d%2d%2d.%3d", &hour, &minute, &second, &millisecond);
  //   int total_seconds = hour * 3600 + minute * 60 + second;
  //   int total_nanoseconds = millisecond * 1000000;
  //   ros::Time camera_timestamp(total_seconds, total_nanoseconds);//格式：秒、纳秒
  //   ros::Time current_time = ros::Time::now();  // 当前系统时间
  //   sensor_msgs::TimeReference time_reference_msg;
  //   time_reference_msg.header.stamp = current_time;
  //   time_reference_msg.source = "HIK_camera";// 相机名称 
  //   time_reference_msg.time_ref = camera_timestamp;//来自该外部源的相应时间

  //   // 打印 time_reference_msg 的数据
  //   // ROS_INFO("Header Stamp: %ld.%09d", time_reference_msg.header.stamp.sec, time_reference_msg.header.stamp.nsec);
  //   // ROS_INFO("Source: %s", time_reference_msg.source.c_str());
  //   // ROS_INFO("Time Reference: %ld.%09d", time_reference_msg.time_ref.sec, time_reference_msg.time_ref.nsec);

  //   time_reference_pub.publish(time_reference_msg);  // 发布消息到话题 //频率是40hz
  //   Clear();
  //   return false;
  // }
  
  //  校验  Checksum $GPRMC/$GNRMC. 
  //   首先，在接收到的数据中，最后两个字节表示校验和，以 * 开头，后跟两个十六进制字符（0-9 和 A-F）。
  //   紧接着，在解析过程中，代码会遍历除去起始符号 $ 和校验和之外的所有数据字节，计算它们的异或值（XOR）。
  //   然后，将计算得到的异或值与校验和进行比较。如果两者相等，则说明数据没有被篡改，校验通过；否则，校验失败。
  //   这样做的目的是确保接收到的数据完整且未被损坏或篡改。通过校验和，可以简单地检查数据在传输过程中是否发生了错误，提高数据的可靠性和稳定性。
  

  //格式： $GPRMC,000185.00,A,2237.496474,N,11356.089515,E,0.0,225.5,310518,2.3,W,A*23

  // const char* data_ptr = rmc_buff_.data();
  // std::string temp = "$GPRMC";
  // if(std::search(rmc_buff_.begin(), rmc_buff_.end(), temp.begin(), temp.end()) != rmc_buff_.end())
  // { 
  //   printf("rmc_buff_:%s\n", data_ptr);
  //   printf("cur_len_:%d\n",cur_len_);//75
  // }

  //if (cur_len_ == 80 && rmc_buff_[cur_len_ - 2] == '*') {//注意信息长度，之前发送的是75，现在是80
  if (rmc_buff_[cur_len_ - 2] == '*') {//注意信息长度，之前发送的是75，现在是80

    rmc_buff_.erase(rmc_buff_.begin()); // 删除第一个元素（回车）
    cur_len_--;
    //rmc_buff_.push_back('\0');

    printf("接受到的gprmc数据:");
    for (char c : rmc_buff_) {
    printf("%c" ,c);
    }
    printf("\n");
    //printf("cur_len_ == %d\n",cur_len_);//75就是一条完整gprmc的数据

    //源代码需要做这一步的校验
     //遍历 $GPRMC 数据中从第二个字符到倒数第三个字符之间的所有字符，对它们进行异或运算，并将结果存储在 result 中
    //  uint8_t result = 0;
    //  for (int i = 1; i < cur_len_ - 2; i++) {
    //    result ^= rmc_buff_[i];
    //  }
    //  char crc[3] = {0};
    //  uint32_t crc_num = 0;
    //  strncpy(crc, &rmc_buff_[cur_len_ - 1], 2);//将 $GPRMC 数据中倒数第二个字符和倒数第一个字符（即校验位）复制到 crc 中。
    //  sscanf(crc, "%x", &crc_num);//转换为十六进制数
    //  result ^= (uint8_t)crc_num;//将 crc_num 强制转换为 uint8_t 类型，并与 result 进行异或运算。
    //  if (result == 0) {//如果等于 0，则校验通过，返回 true。
    //    printf("校验通过!\n");
    //    return true;
    //  }
    //  else{
    //   printf("校验没通过！\n");
    //   printf("----------------------------------------\n");
    //  }
    

    //验证校验和
    const char* rmc_buff_data = rmc_buff_.data();
    if (validate_checksum(rmc_buff_data)) {
        printf("校验通过！\n");
        return true;
    } else {
        printf("校验未通过！\n");
        printf("----------------------------------------\n");
        Clear();
        return false;
    }
    //return true;//默认校验全部通过，因为我们不是要真正的gprmc数据，只是要里面的时间段即可。
  }

  cur_len_++;
  return false; 
}
*/


//原来的解析函数。现在要接受真正的gps信息。上面那个代码是符合32发送的虚假的gprmc数据解析。
bool Synchro::ParseGps(uint8_t in_byte) {
    
  if (cur_len_ < strlen("$GPRMC")) {
    rmc_buff_[0] = rmc_buff_[1];
    rmc_buff_[1] = rmc_buff_[2];
    rmc_buff_[2] = rmc_buff_[3];
    rmc_buff_[3] = rmc_buff_[4];
    rmc_buff_[4] = rmc_buff_[5];
    rmc_buff_[5] = in_byte;
    cur_len_++;
    // if (cur_len_ == strlen("$GPRMC")) {
    //   if (strncmp((const char*)&rmc_buff_[0], "$GPRMC", strlen("$GPRMC")) != 0 &&  \
    //       strncmp((const char*)&rmc_buff_[0], "$GNRMC", strlen("$GNRMC")) != 0) {
    //     cur_len_--;
    //   } 
    // }
    return false;
  }

  if (cur_len_ >= rmc_buff_.size()) {
    Clear();
    return false;
  }

  rmc_buff_[cur_len_] = in_byte;

  if(rmc_buff_[cur_len_] == '#'){//接受到完整的cam数据了。
    
        // static auto start2 = std::chrono::high_resolution_clock::now();
        // auto end2 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration2 = end2 - start2;
        // std::cout << "pc accept $cam data time（ is 0.05(20hz) or 0.025(40hz) s ?）: " << duration2.count() << " s" << std::endl;
        // start2 = std::chrono::high_resolution_clock::now();//重新开始计时

    Send_Image_Time(&rmc_buff_.at(0));
    // if(Send_Image_Time(&rmc_buff_.at(0))){
    //   printf("-----发送image时间戳成功!------\n");
    // }
    // else{
    //   printf("-----发送image时间戳失败。-----\n");
    // }

    Clear();//数据用完了，要清空掉。
    return false;
  }
  else if (rmc_buff_[cur_len_ - 2] == '*') {//接受到完整的gpemc数据了。
     
    //  uint8_t result = 0;
    //  for (int i = 1; i < cur_len_ - 2; i++) {
    //    result ^= rmc_buff_[i];
    //  }
    //  char crc[3] = {0};
    //  uint32_t crc_num = 0;
    //  strncpy(crc, &rmc_buff_[cur_len_ - 1], 2);
    //  sscanf(crc, "%x", &crc_num);
    //  result ^= (uint8_t)crc_num;
    //  if (result == 0) {
    //    return true;
    //  }
    //  else{
    //   printf("-----校验没通过！-----\n");
    //  }
    return true;

  }
  
  cur_len_++;
  return false; 
}



//接受到完整的$cam时间信息后，调用这条函数
bool Synchro:: Send_Image_Time(const char* cam_str){
  //const char* cam_str = "$cam,2018.03.12,00:10:20:500#";//接受到的数据
  printf("接受到的$cam格式数据:%s\n", cam_str); 

  ros::Time current_time = ros::Time::now();  // 当前系统时间，一般精度在几毫秒到微秒级别之间。
  double time_sec = current_time.toSec(); // 将 ROS 时间转换为秒
  current_time = ros::Time().fromSec((time_sec-0.00225)); // 将秒转换回 ROS 时间对象

    
  // std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
  // long long current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();// 将当前时间转换为从纳秒到秒的计数
  // // 将当前时间减去 2.25ms（就是波特率115200的32发送字符串到串口的时间。），转换为 ROS 时间
  // ros::Time current_time(current_time_sec -0.00225);


  const char* cam_begin = strstr(cam_str, "$cam,");
  if (cam_begin == NULL) {
    return false;
  }
  num = sscanf(cam_begin, "$cam,%4d.%2d.%2d,%2d:%2d:%2d:%3d#",&year,&month,&day,&hour,&minute,&second,&ms);
  if (num != 7) {
    return false;
  }
  
  struct tm cam_utc_time;
  //memset(&cam_utc_time, 0, sizeof(cam_utc_time));
  cam_utc_time.tm_isdst = 0;
  cam_utc_time.tm_year = year -1900;
  cam_utc_time.tm_mon = month -1;
  cam_utc_time.tm_mday = day ;
  cam_utc_time.tm_hour = hour;
  cam_utc_time.tm_min = 0;
  cam_utc_time.tm_sec = 0;

  //time_epoch = mktime(&cam_utc_time);
  time_epoch = timegm(&cam_utc_time);  // no timezone 没有显式地指定任何特定的时区信息，而是默认使用本地时区来计算时间戳。
  //得到 $cam里面的“年月日时”转化为秒数。

  //time_epoch = time_epoch * 1000000 + (minute*60*1000000 + second*1000000 + ms*1000);//把$cam里面的“分秒毫秒”转化为微秒，再和gprmc里面的“年月日时”转化为微秒，相加。得到总微秒数量。
  //time_epoch =time_epoch * 1000;//转化为总纳秒

  total_seconds =time_epoch + minute*60 + second;//总秒数
  total_nanoseconds = ms * 1000000;//纳秒部分

  

  ros::Time camera_timestamp(total_seconds, total_nanoseconds);


  //ros::Time current_time = std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1000000000.0;// 用这个精度更高。可以达到纳秒级别甚至更高
  sensor_msgs::TimeReference time_reference_msg;
  time_reference_msg.header.stamp = current_time;//系统时间
  time_reference_msg.source = "HIK_camera";// 相机名称 
  time_reference_msg.time_ref = camera_timestamp;//来自该外部源的触发时间 utc时间

  // 打印 time_reference_msg 的数据
  // ROS_INFO("Header Stamp: %ld.%09d", time_reference_msg.header.stamp.sec, time_reference_msg.header.stamp.nsec);
  // ROS_INFO("Source: %s", time_reference_msg.source.c_str());
  // ROS_INFO("The timestamp sent to the camera: %d.%09d", time_reference_msg.time_ref.sec, time_reference_msg.time_ref.nsec);
  time_reference_pub.publish(time_reference_msg);  // 发布消息到话题

  return true;
}

//该方法用于设置同步定时器的回调函数和相关数据。当解析到完整的 GPS 消息时，会调用此回调函数。
void Synchro::SetSyncTimerCallback(FnReceiveSyncTimeCb cb, void *data) {
    if ((cb != nullptr) || (data != nullptr)) {
      sync_timer_cb_ = cb;
      client_data_ = data;
    }
}

//该方法用于处理接收到的原始 GPS 数据流，循环调用 ParseGps 方法解析每个字节，并在解析完成后检查是否触发了同步定时器的回调函数。
//读取缓存buff、实际读取到的字节数size
void Synchro::Decode(uint8_t * buff, size_t size) {
  for (size_t i = 0; i < size; i++) {
    if (ParseGps(buff[i])) {
      //解析完一条数据了
      if (sync_timer_cb_) {

        // //串口传输消息发生通信延迟，已解决。
        // static auto start = std::chrono::high_resolution_clock::now();
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;
        // std::cout << "pc accept $GPRMC data time（ is 1 s ?）: " << duration.count() << " s" << std::endl;
        // start = std::chrono::high_resolution_clock::now();
        // //20hz的时候，这里接受到的是1s+50ms，正常应该是1s，多了50ms。
        // //40hz的时候，这里接受到的是1s+299ms，正常应该是1s，多了299ms。


        //在这里把当前系统时间减去gprmc传输延迟的时间，发送到共享内存中。
        auto pc_tt = std::chrono::high_resolution_clock::now();   
        long gprmc_start_send_time = std::chrono::duration_cast<std::chrono::nanoseconds>(pc_tt.time_since_epoch()).count();       
        gprmc_start_send_time -= static_cast<long>(6.6 * 1000000); //6.6ms是gprmc串口传输的耗时
        //pointt_clk->pc_tt = std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(gprmc_start_send_time));
        pointt_clk->pc_tt = gprmc_start_send_time;

        //这里的sync_timer_cb_回调函数是LdsLidar::ReceiveSyncTimeCallback()
        //在解析完成某条 GPS 消息后，会调用 LdsLidar 类中的 ReceiveSyncTimeCallback 方法来处理同步定时器事件。
        sync_timer_cb_(&rmc_buff_.at(0), cur_len_, client_data_);
        Clear();//数据用完了，要清空掉。
      }
    }
  }
}

void Synchro::Clear() {
   cur_len_ = 0;
   std::fill(rmc_buff_.begin(), rmc_buff_.end(), 0);
}

#ifdef WIN32
bool Synchro::Open() {
  hfile_ = CreateFile(port_name_.c_str(),
                      GENERIC_READ,
                      0,
                      NULL,
                      OPEN_EXISTING,
                      FILE_ATTRIBUTE_NORMAL,
                      NULL);

  if (hfile_ == INVALID_HANDLE_VALUE) {
    printf("Open %s serials fail!\n", port_name_.c_str());
    return false;
  }

 /* Set baudrate and parity,etc. */
  Setup(baudrate_, parity_);
  printf("Set baudrate success.\n");
  return true;
}

void Synchro::Close() {
  if (hfile_ != INVALID_HANDLE_VALUE) {
    PurgeComm(hfile_, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT); 
    CloseHandle(hfile_);
  }
  hfile_ = INVALID_HANDLE_VALUE;
}

/* Sets up the port parameters */
int Synchro::Setup(enum BaudRate baud, enum Parity parity) {
  static uint32_t baud_map[19] = {\
                                   2400, 4800, 9600, 19200, 38400, 57600,115200, 230400,\
                                   460800, 500000, 576000,921600,1152000, 1500000, 2000000,\
                                   2500000, 3000000, 3500000, 4000000\
                                 };
  DCB dcb;
  GetCommState(hfile_, &dcb);
  dcb.BaudRate = DWORD(baud_map[baud]);
  switch (parity) {
    case P_8N1:
      /* No parity (8N1)  */
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    case P_7E1:
      /* Even parity (7E1) */
      dcb.ByteSize = 7;
      dcb.Parity = EVENPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    case P_7O1:
      /* Odd parity (7O1) */
      dcb.ByteSize = 7;
      dcb.Parity = ODDPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    case P_7S1:
      /* Space parity is setup the same as no parity (7S1)  */
      dcb.ByteSize = 7;
      dcb.Parity = SPACEPARITY;
      dcb.StopBits = ONESTOPBIT;
    break;
    default:
      return -1;
  }
  SetCommState(hfile_, &dcb);
  PurgeComm(hfile_, PURGE_RXCLEAR|PURGE_TXCLEAR|PURGE_RXABORT|PURGE_TXABORT);
  return 0;
}

size_t Synchro::Read(uint8_t *buffer, size_t size) {
  DWORD len = 0;
  if (hfile_ !=  INVALID_HANDLE_VALUE) {
    ReadFile(hfile_, buffer, size, &len, NULL);
    return len;
  } else {
    return 0;
  }
}

#else

bool Synchro::Open() {
  fd_ = open(port_name_.c_str(), O_RDONLY | O_NOCTTY);
  if (fd_ < 0) {
    printf("Open %s serials fail!\n", port_name_.c_str());
    return false;
  }
  else{
    
    //成功打开串口，现在发送自定义的相机采样频率给stm32。
    //printf("成功打开串口 %s ,现在发送自定义的相机采样频率给stm32!\n", port_name_.c_str());

    boost::asio::io_service io;  // 创建一个IO服务
    boost::asio::serial_port serial(io, port_name_);  // 替换成您的串口号
    if(baudrate_ == BR9600){
      serial.set_option(boost::asio::serial_port_base::baud_rate(9600));  // 设置波特率  9600
    }else if(baudrate_ == BR115200){
      serial.set_option(boost::asio::serial_port_base::baud_rate(115200));  // 设置波特率  115200
    }

    std::string freq_str = std::to_string(camera_publish_freq_) + "\r\n";  // 将频率转换为字符串并添加换行符
    //注意：linux发送"\r\n"，到stm32解析的时候就是“\n”。
    
    boost::system::error_code ec;
    boost::asio::write(serial, boost::asio::buffer(freq_str), ec);  // 将数据写入串口
    
    if (ec) {
        std::cerr << "发送自定义的相机采样频率给stm32错误: " << ec.message() << std::endl;  // 发生错误时输出错误信息
    }else{
      printf("成功设置的相机采样频率为:%s\n", freq_str.c_str());
    }


    Setup(baudrate_, parity_);

    printf(" Set baudrate success.\n");
  
  }

  const char *shared_file_name = "/home/sqhl/Desktops/fast_livo_catkin_ws/src/time_stamp_file";//"/home/sqhl/Desktops/new_sync_method_ws/src/time_stamp_file";
  fd = open(shared_file_name, O_CREAT | O_RDWR, 0777);
  if (fd == -1) {
    printf("time_stamp_file 打开失败！\n");
    exit(-1);
  } else {
    printf("time_stamp_file 打开成功！\n");
  }
  write(fd, "", sizeof(my_t));
  pointt_clk = (my_t *)mmap(NULL, sizeof(my_t) * 1, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  return true;
}

void Synchro::Close() {
  if (fd_ > 0) {
    /* Flush the port */
    tcflush(fd_,TCOFLUSH); 
    tcflush(fd_,TCIFLUSH); 

    close(fd_);

    munmap(pointt_clk, sizeof(my_t) * 1);

  }
  fd_ = 0;
}

/* Sets up the port parameters  设置端口参数（波特率和parity）*/
int Synchro::Setup(enum BaudRate baud, enum Parity parity) {
  static uint32_t baud_map[19] = {\
                                   B2400, B4800, B9600, B19200, B38400, B57600,B115200, B230400,\
                                   B460800, B500000, B576000,B921600,B1152000, B1500000, B2000000,\
                                   B2500000, B3000000, B3500000, B4000000\
                                 };
  tcflag_t baudrate;
  struct termios options;

  /* Clear old setting completely,must add here for A3 CDC serial */
  tcgetattr(fd_, &options);
  memset(&options, 0, sizeof(options));
  tcflush(fd_, TCIOFLUSH);
  tcsetattr(fd_, TCSANOW, &options);
  usleep(10000);

  /* Minimum number of characters to read */
  options.c_cc[VMIN] = 0;  
   
  /* Time to wait for data */
  options.c_cc[VTIME] = 100; 

  /* Enable the receiver and set local mode... */
  options.c_cflag |= (CLOCAL | CREAD);

  /* Set boadrate */
  options.c_cflag &= ~CBAUD;
  baudrate = baud_map[baud];
  options.c_cflag |= baudrate;
  printf("[Baudrate]: %d %d\r\n", baud, baudrate);

  switch (parity) {
    case P_8N1:
      /* No parity (8N1)  */
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
    break;
    case P_7E1:
      /* Even parity (7E1) */
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
    break;
    case P_7O1:
      /* Odd parity (7O1) */
      options.c_cflag |= PARENB;
      options.c_cflag |= PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
    break;
    case P_7S1:
      /* Space parity is setup the same as no parity (7S1)  */
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
    break;
    default:
      return -1;
  }    

  /* flush the port */
  tcflush(fd_, TCIOFLUSH);
    
  /* send new config to the port */
  tcsetattr(fd_, TCSANOW, &options);

  return 0;
}

//从串口中读取数据，将读取到的数据存储到 buffer 中，读取的最大字节数为 size。
size_t Synchro::Read(uint8_t *buffer, size_t size) {
  if (fd_ > 0) {//fd_是open的文件标识符（已经打开的串口文件）
    return read(fd_, buffer, size);//返回实际读取到的字节数
  } else {
    return 0;
  }
}
#endif


}

