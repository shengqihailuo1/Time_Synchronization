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

#include "livox_sdk.h"
#include "command_handler/command_handler.h"
#include "data_handler/data_handler.h"
#include "base/logging.h"
#include "device_manager.h"
#ifdef WIN32
#include<winsock2.h>
#endif // WIN32

using namespace livox;
IOThread *g_thread = NULL;
static bool is_initialized = false;

void GetLivoxSdkVersion(LivoxSdkVersion *version) {
  if (version != NULL) {
    version->major = LIVOX_SDK_MAJOR_VERSION;
    version->minor = LIVOX_SDK_MINOR_VERSION;
    version->patch = LIVOX_SDK_PATCH_VERSION;
  }
}

bool Init() {
  if (is_initialized) {
    return false;
  }
#ifdef WIN32
  WORD sockVersion = MAKEWORD(2, 0);
  WSADATA wsdata;
  if (WSAStartup(sockVersion, &wsdata) != 0) {
    return false;
  }
#endif // WIN32

  bool result = false;
  do {
    InitLogger();

    g_thread = new IOThread();
    g_thread->Init();

    if (device_discovery().Init() == false) {
      result = false;
      break;
    }

    if (device_manager().Init() == false) {
      result = false;
      break;
    }

    if (command_handler().Init(g_thread->loop()) == false) {
      result = false;
      break;
    }
    if (data_handler().Init() == false) {
      result = false;
      break;
    }

    result = true;
  } while (0);

  is_initialized = result;
  return result;
}

//确保在程序退出或不再需要使用时，能够正确地释放和清理所有相关资源，避免内存泄漏和资源浪费。
void Uninit() {
  if (!is_initialized) {
    return;
  }
#ifdef WIN32
    WSACleanup();
#endif // WIN32
  if (g_thread) {
    //调用 Quit() 方法来终止线程的运行，并使用 Join() 方法等待线程结束。
    g_thread->Quit();
    g_thread->Join();
  }

  if (g_thread) {
    //调用 g_thread 的 Uninit() 方法进行线程的反初始化，并删除 g_thread 对象。
    g_thread->Uninit();
    delete g_thread;
    g_thread = NULL;
  }

  //依次调用各个组件的 Uninit() 方法来释放相应的资源，包括设备发现、命令处理、数据处理和设备管理等
  device_discovery().Uninit();
  command_handler().Uninit();
  data_handler().Uninit();
  device_manager().Uninit();
  
  UninitLogger();//关闭日志记录器，并将 is_initialized 设置为 false，表示反初始化完成。
  is_initialized = false;
}

//启动在单独线程上运行的设备扫描例程。*@如果成功启动，则返回true，否则返回false。
bool Start() {
  if (g_thread && device_discovery().Start(g_thread->loop()) && g_thread->Start()) {
    return true;
  }
  return false;
}

void SaveLoggerFile() {
  is_save_log_file = true;
}

void DisableConsoleLogger() {
  is_console_log_enable = false;
}