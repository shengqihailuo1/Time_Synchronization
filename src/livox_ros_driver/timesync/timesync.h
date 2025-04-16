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

#ifndef TIMESYNC_TIMESYNC_H_
#define TIMESYNC_TIMESYNC_H_

#include <thread>
#include "comm_device.h"
#include "comm_protocol.h"
#include "user_uart.h"

namespace livox_ros {

typedef void (*FnReceiveSyncTimeCb)(const char *rmc, uint32_t rmc_length,
                                    void *client_data);

enum FsmPollState { kOpenDev, kPrepareDev, kCheckDevState, kFsmDevUndef };

typedef struct {
  CommDevConfig dev_config;
  ProtocolConfig protocol_config;
} TimeSyncConfig;

class TimeSync {
 public:

 //单例模式的实现，用于获取 TimeSync 类的唯一实例。
 //使用静态变量 time_sync 来保存 TimeSync 的唯一实例，并通过 GetInstance() 静态方法返回该实例。
 //单例模式可以确保一个类只有一个实例，并提供全局访问点。在这个例子中，TimeSync 类的唯一实例被创建为静态变量，并在第一次调用 GetInstance() 方法时进行初始化。以后的每次调用都会返回同一个实例。
 //好处是可以方便地共享相同的实例，并确保全局只有一个对象存在，避免了重复创建和资源浪费的问题。
  static TimeSync *GetInstance() {
    static TimeSync time_sync;

    return &time_sync;
  }


  int32_t InitTimeSync(const TimeSyncConfig &config);
  int32_t DeInitTimeSync();
  void StartTimesync() {
    start_poll_state_ = true;
    start_poll_data_ = true;
  }

  int32_t SetReceiveSyncTimeCb(FnReceiveSyncTimeCb cb, void *data) {
    if ((cb != nullptr) || (data != nullptr)) {
      fn_cb_ = cb;
      client_data_ = data;
      return 0;
    } else {
      return -1;
    }
  }

 private:
  TimeSync();
  ~TimeSync();
  TimeSync(const TimeSync &) = delete;
  TimeSync &operator=(const TimeSync &) = delete;

  void PollStateLoop();
  void PollDataLoop();
  void StopTimesync();

  std::shared_ptr<std::thread> t_poll_state_;
  volatile bool exit_poll_state_;
  volatile bool start_poll_state_;

  std::shared_ptr<std::thread> t_poll_data_;
  volatile bool exit_poll_data_;
  volatile bool start_poll_data_;

  TimeSyncConfig config_;
  UserUart *uart_;
  CommProtocol *comm_;
  volatile uint32_t rx_bytes_;

  FnReceiveSyncTimeCb fn_cb_;
  void *client_data_;

  volatile uint8_t fsm_state_;
  std::chrono::steady_clock::time_point transfer_time_;
  void FsmTransferState(uint8_t new_state);
  void FsmOpenDev();
  void FsmPrepareDev();
  void FsmCheckDevState();
};

}  // namespace livox_ros
#endif
