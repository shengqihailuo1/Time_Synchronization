#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using namespace std;

bool g_bExit = false;
image_transport::Publisher pub;

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void) {
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress enter to exit.\n");
  while (getchar() != '\n')
    ;
  g_bExit = true;
  sleep(1);
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined
    // name
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  } else
    printf("Not support.\n");

  return true;
}

static void *WorkThread(void *pUser) {//这里的 pUser 就是相机句柄
  int nRet = MV_OK;

  // ch:获取数据包大小 | en:Get payload size
  //通过使用memset函数将stParam的所有字节都设置为零，可以确保将stParam的成员变量都初始化为零。
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};//该结构体用于存储图像的相关信息，例如图像宽度、高度、帧率等。
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));//初始化为0
  //使用malloc函数为图像数据的缓冲区分配内存空间。缓冲区的大小为stParam.nCurValue字节
  unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData)
    return NULL;

  unsigned int nDataSize = stParam.nCurValue;//最后，将缓冲区的大小保存在变量nDataSize中，以便后续使用。

  while (1) {

    //发送一个软件触发命令"TriggerSoftware"给相机
    nRet = MV_CC_SetCommandValue(pUser, "TriggerSoftware");
    if (MV_OK != nRet)
      printf("failed in TriggerSoftware[%x]\n", nRet);//nRet如果是：80000106-->节点访问条件有误

    //从相机中获取一帧图像，存放到pData里，图像信息放在stImageInfo，超时时间1000毫秒
    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      //nFrameNum值是当前帧为是第x帧，从0开始递增。
      printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
      cv::Mat srcImage;
      srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      //使用cv_bridge::CvImage类将srcImage转换为ROS的sensor_msgs::Image消息类型，并通过发布器pub发布该消息。
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
      pub.publish(msg);
    }
    else
        printf("Get One Frame failed![%x]\n", nRet);

    if (g_bExit)//按下回车键，退出
      break;
  }

  if (pData) {
    free(pData);//销毁指针
    pData = NULL;
  }

  return 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trigger");
  // std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  pub = it.advertise("mvs_camera/image", 1);

  int nRet = MV_OK;

  void *handle = NULL;
  do {
    MV_CC_DEVICE_INFO_LIST stDeviceList;//设备信息列表
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
      break;
    }
    if (stDeviceList.nDeviceNum > 0) {
      for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        printf("[device %d]:\n", i);
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
          break;

        PrintDeviceInfo(pDeviceInfo);
      }
    } else {
      printf("Find No Devices!\n");
      break;
    }

    printf("Please Intput camera index: ");
    unsigned int nIndex = 0;
    scanf("%d", &nIndex);

    if (nIndex >= stDeviceList.nDeviceNum) {
      printf("Intput error!\n");
      break;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
      printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
      break;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
      break;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal
    // package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
      int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
      if (nPacketSize > 0) {
        nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
        if (nRet != MV_OK)
          printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
      } else
        printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
    }

    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);//把“启用采集帧率“设置为false，即：禁用相机的采集帧率限制
    if (MV_OK != nRet) {
      printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
      break;
    }

//MV_CC_SetEnumValue函数用于设置相机属性的枚举值，第一个参数handle表示相机的句柄，第二个参数"ExposureAuto"是相机属性的名称，第三个参数2表示要设置的属性值为自动曝光模式。
//设置为自动曝光模式后，相机会根据场景的亮度自动调整曝光时间，以确保图像的适当曝光。如果需要手动设置曝光时间，请将ExposureAuto属性设置为关闭（0或1），然后使用MV_CC_SetFloatValue函数设置曝光时间的值。
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 2);//2是自动曝光模式，0是固定曝光模式
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Exposure auto mode");

    nRet = MV_CC_SetAutoExposureTimeLower(handle, 65);//自动曝光时间最低值
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Exposure Time Lower");

    nRet = MV_CC_SetAutoExposureTimeUpper(handle, 10000);//自动曝光时间最高值
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Exposure Time Upper");

    nRet = MV_CC_SetEnumValue(handle, "GainAuto", 2);//自动增益
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Gain auto mode");

    // nRet = MV_CC_SetFrameRate(handle, 10);//设置采集帧率。
    // if (MV_OK != nRet)
    //     ROS_ERROR("Fail to set Frame Rate");

    // 设置触发模式为on
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);//打开触发模式
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
      break;
    }

    // 设置触发源
//   typedef enum _MV_CAM_TRIGGER_SOURCE_ {
//    MV_TRIGGER_SOURCE_LINE0 = 0,
//    MV_TRIGGER_SOURCE_LINE1 = 1,
//    MV_TRIGGER_SOURCE_LINE2 = 2,
//    MV_TRIGGER_SOURCE_LINE3 = 3,
//    MV_TRIGGER_SOURCE_COUNTER0 = 4,
//    MV_TRIGGER_SOURCE_SOFTWARE = 7,
//    MV_TRIGGER_SOURCE_FrequencyConverter = 8,
//    } MV_CAM_TRIGGER_SOURCE;
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);//将触发源设置为 Line0。
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
      break;
    }

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    pthread_t nThreadID;//创建线程执行WorkThread函数
    //handle：作为参数传递给WorkThread函数的用户数据。
    nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
    if (nRet != 0) {
      printf("thread create failed.ret = %d\n", nRet);
      break;
    }

    PressEnterToExit();//这里按下exit就退出程序。另一边的线程函数一直在执行。

    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      break;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
      break;
    }
  } while (0);

  if (nRet != MV_OK) {
    if (handle != NULL) {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
  }

  printf("exit\n");
  return 0;
}
