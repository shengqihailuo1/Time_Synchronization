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

#include "timesync.h"

#include <stdint.h>
#include <string.h>
#include <chrono>
#include <cstdio>
#include <functional>
#include <thread>
#include <stdio.h>
namespace livox_ros {
using namespace std;

TimeSync::TimeSync()
      //这些标志位在多线程编程中用于控制线程的行为
    : exit_poll_state_(false),//控制“轮询设备状态“的线程是否退出。
      start_poll_state_(false),//控制“轮询设备状态“的线程是否启动。
      exit_poll_data_(false),//控制“轮询设备数据“的线程是否退出。
      start_poll_data_(false) {//控制“轮询设备数据“的线程是否启动。
  fsm_state_ = kOpenDev;//初始化状态为“kOpenDev”（打开设备）
  uart_ = nullptr;
  comm_ = nullptr;
  fn_cb_ = nullptr;//注意：这里的回调函数fn_cb_是  LdsLidar::ReceiveSyncTimeCallback() 函数
  client_data_ = nullptr;
  rx_bytes_ = 0;//Rx接受的字节数
}

TimeSync::~TimeSync() { DeInitTimeSync(); }

//形参是 livox_lidar_config.json 文件中的 timesync_config
int32_t TimeSync::InitTimeSync(const TimeSyncConfig &config) {
  config_ = config;

  if (config_.dev_config.type == kCommDevUart) {
    uint8_t baudrate_index = config_.dev_config.config.uart.baudrate;
    uint8_t parity_index = config_.dev_config.config.uart.parity;

    if ((baudrate_index < BRUnkown) && (parity_index < ParityUnkown)) {
      uart_ = new UserUart(baudrate_index, parity_index);
    } else {
      printf("Uart parameter error, please check the configuration file!\n");
      return -1;
    }
  } else {
    printf("Device type not supported, now only uart is supported!\n");
    return -1;
  }

  config_.protocol_config.type = kGps;//gps协议
  comm_ = new CommProtocol(config_.protocol_config);

  //开启两个线程，用于分别轮询设备状态和数据。
  t_poll_state_ = std::make_shared<std::thread>(std::bind(&TimeSync::PollStateLoop, this));
  t_poll_data_ = std::make_shared<std::thread>(std::bind(&TimeSync::PollDataLoop, this));

  return 0;
}

int32_t TimeSync::DeInitTimeSync() {
  StopTimesync();

  if (uart_) delete uart_;
  if (comm_) delete comm_;

  //将回调函数(fn_cb_)和客户端数据(client_data_)重置为nullptr
  fn_cb_ = nullptr;
  client_data_ = nullptr;
  return 0;
}

void TimeSync::StopTimesync() {
  start_poll_state_ = false;
  start_poll_data_ = false;
  exit_poll_state_ = true;
  exit_poll_data_ = true;
  if (t_poll_state_) {
    //等待线程结束并释放资源。
    t_poll_state_->join();
    t_poll_state_ = nullptr;
  }

  if (t_poll_data_) {
    //等待线程结束并释放资源。
    t_poll_data_->join();
    t_poll_data_ = nullptr;
  }
}

//线程函数1
void TimeSync::PollStateLoop() {
  while (!start_poll_state_) {//等待
    /* waiting to start */ 
  }
  while (!exit_poll_state_) {
    //根据当前状态机的状态执行相应的操作。
    if (fsm_state_ == kOpenDev) {
      FsmOpenDev();
    } else if (fsm_state_ == kPrepareDev) {
      FsmPrepareDev();
    } else if (fsm_state_ == kCheckDevState) {
      FsmCheckDevState();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));//暂停50毫秒，周期性地轮询设备状态
  }
}

//线程函数2
void TimeSync::PollDataLoop() {

  printf("进入:PollDataLoop()\n");

  while (!start_poll_data_) {//等待指令
    /* waiting to start */
  }

  while (!exit_poll_data_) {
    if (uart_->IsOpen()) {//判断串口是否已经打开
    //尝试从UART设备读取数据，并将数据存储到缓冲区中
      uint32_t get_buf_size;
      //获取缓存中的空闲空间大小get_buf_size，并返回指向可写的位置指针cache_buf
      uint8_t *cache_buf = comm_->FetchCacheFreeSpace(&get_buf_size);

      if (get_buf_size) {
        //从 UART 设备中读取数据并存储到指定的缓冲区中。
        
        uint32_t read_data_size;
        //读取不多于 get_buf_size 字节的数据，并将其存储到 cache_buf 指向的内存区域中。返回实际读取到的数据大小。
        read_data_size = uart_->Read((char *)cache_buf, get_buf_size);

        if (read_data_size) {
          comm_->UpdateCacheWrIdx(read_data_size);//更新缓存的写入指针和接收字节数。
          rx_bytes_ += read_data_size;//RX接受的字节数

          CommPacket packet;
          memset(&packet, 0, sizeof(packet));//初始化为0。

          // 解析 $GPRMC
          while ((kParseSuccess == comm_->ParseCommStream(&packet))) {
            
            //在每次成功解析一条数据后，会检查回调函数(fn_cb_)和客户端数据(client_data_)是否存在
            if (((fn_cb_ != nullptr) || (client_data_ != nullptr))) {
              
              //判断解析出来的数据是否是"GPRMC"或"GNRMC"类型的数据
              if ((strstr((const char *)packet.data, "$GPRMC")) || (strstr((const char *)packet.data , "$GNRMC"))){
              
                // printf((const char *)packet.data); // 这里把GPRMC包的结果打印出来了

                //调用回调函数(fn_cb_)，将解析出来的数据传递给回调函数进行处理。
                //注意：这里的回调函数fn_cb_是  LdsLidar::ReceiveSyncTimeCallback() 函数
                fn_cb_((const char *)packet.data, packet.data_len, client_data_);
                
                // printf((const char *)client_data_);
                printf("RMC data parse success RMC数据解析成功 !.\n");
              }
            }
          }
        }
      }
    } else {
      //printf("TimeSync::PollDataLoop() : uart_ 串口未打开！\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}


/*
把新状态保存到 fsm_state_ 变量中，并记录状态转换时间 transfer_time_。
主要包括以下几个状态和函数：
    kFsmDevUndef：设备未定义状态。
    kOpenDev：设备已打开状态。
    kPrepareDev：准备设备状态。
    kCheckDevState：检查设备状态。
*/
void TimeSync::FsmTransferState(uint8_t new_state) {
  if (new_state < kFsmDevUndef) {
    fsm_state_ = new_state;
  }
  transfer_time_ = chrono::steady_clock::now();
}


//下面的三个函数是PollStateLoop（）线程1运行用到的函数


void TimeSync::FsmOpenDev() {
  if (!uart_->IsOpen()) {
    if (!uart_->Open(config_.dev_config.name)) {
      //打开串口，并将状态转换为 kFsmDevPrep
      FsmTransferState(kPrepareDev);
    }
  } else {
    //直接将状态转换为 kFsmDevPrep
    FsmTransferState(kPrepareDev);
  }
}

//准备设备状态（即：打开设备后，等待3s）
void TimeSync::FsmPrepareDev() {
  //计算当前时间与前一个状态转换时间 transfer_time_ 的时间差 time_gap，并判断时间差是否超过了 3000 毫秒（即 3 秒）。如果超过了，则将状态转换为 kFsmDevCheck。
  chrono::steady_clock::time_point t = chrono::steady_clock::now();
  chrono::milliseconds time_gap = chrono::duration_cast<chrono::milliseconds>(t - transfer_time_);
  /** delay some time when device is opened, 4s */
  if (time_gap.count() > 3000) {
    FsmTransferState(kCheckDevState);
  }
}

//检查设备状态
//定时检查串口接收到的数据字节数 rx_bytes_ 是否有变化。
//当连续两次检查间隔时间超过了 2000000 毫秒（即 2 秒），且两次检查的数据字节数相同，则说明串口连接已经断开，需要关闭串口并将状态转换为 kFsmDevOpen。
void TimeSync::FsmCheckDevState() {
  static uint32_t last_rx_bytes = 0;
  static chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::milliseconds time_gap = chrono::duration_cast<chrono::milliseconds>(t2 - t1);

  if (time_gap.count() > 2000000) { /* period : 2.5s */
    if (last_rx_bytes == rx_bytes_) {
      uart_->Close();
      FsmTransferState(kOpenDev);
      printf("Uart is disconnected, close it\n");
    }
    last_rx_bytes = rx_bytes_;
    t1 = t2;
  }
}

}  // namespace livox_ros
