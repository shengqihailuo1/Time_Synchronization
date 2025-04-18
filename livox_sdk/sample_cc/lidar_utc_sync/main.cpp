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

#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <string.h>
#include "cmdline.h"
#include "lds_lidar.h"
#include "synchro.h"


/** Cmdline input broadcast code */
static std::vector<std::string> cmdline_broadcast_code;

/** Set the program options.
* You can input the registered device broadcast code and decide whether to save the log file.
*/
void SetProgramOption(int argc, const char *argv[]) {
  cmdline::parser cmd;
  cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
  cmd.add("log", 'l', "Save the log file");
  cmd.add("help", 'h', "Show help");
  cmd.parse_check(argc, const_cast<char **>(argv));
  if (cmd.exist("code")) {
    std::string sn_list = cmd.get<std::string>("code");
    printf("Register broadcast code: %s\n", sn_list.c_str());
    size_t pos = 0;
    cmdline_broadcast_code.clear();
    while ((pos = sn_list.find("&")) != std::string::npos) {
      cmdline_broadcast_code.push_back(sn_list.substr(0, pos));
      sn_list.erase(0, pos + 1);
    }
    cmdline_broadcast_code.push_back(sn_list);
  }
  if (cmd.exist("log")) {
    printf("Save the log file.\n");
    SaveLoggerFile();
  }
  return;
}

int main(int argc, const char *argv[]) {
/** Set the program options. */
  SetProgramOption(argc, argv);

  LdsLidar& read_lidar = LdsLidar::GetInstance();

  int ret = read_lidar.InitLdsLidar(cmdline_broadcast_code);
  if (!ret) {
    printf("Init lds lidar success!\n");
  } else {
    printf("Init lds lidar fail!\n");
    return 0;
  }

  printf("Start discovering device.\n");

  Synchro& synchro = Synchro::GetInstance();
 #ifdef WIN32
  synchro.SetPortName("COM3");
 #else
  synchro.SetPortName("/dev/ttyUSB0");
#endif



/** Set Synchro's Baudrate: 9600. */
  synchro.SetBaudRate(BR115200);
  //synchro.SetBaudRate(BR115200);



/** Set Synchro's Parity: No parity (8N1). */
  synchro.SetParity(P_8N1);
/** SyncTimer Callback will trigger when Synchro's GPRMC/GNRMC signal is ready. */
//接受到完整的gprmc数据后，就调用read_lidar.SetRmcSyncTime这个回调函数。。
  synchro.SetSyncTimerCallback(read_lidar.SetRmcSyncTime);
  if (synchro.Start()) {
    printf("Synchro start success");
  } else {
    printf("Synchro start failed");
    return 0;
  }

  std::this_thread::sleep_for(std::chrono::seconds(100));

  //写一些代码，把点云数据发布出来。。。

  synchro.Stop();
  read_lidar.DeInitLdsLidar();
  printf("Livox lidar demo end!\n");
}
