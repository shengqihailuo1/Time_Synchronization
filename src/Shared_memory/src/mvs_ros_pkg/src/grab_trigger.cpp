#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <chrono>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
struct time_stamp {
  int64_t high;
  int64_t low;
};

time_stamp *pointt;

using namespace std;

unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
image_transport::Publisher pub;
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
std::string CameraName;
float resize_divider;

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  resize_divider = Params["resize_divide"];
  if (resize_divider < 0.1) {
    resize_divider = 1;
  }
  int ExposureAuto = Params["ExposureAuto"];
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];

  int CenterAlign = Params["CenterAlign"];

  int nRet;
  if (CenterAlign) { //这里开始 设置中心对齐 
    
    // Strobe输出
    //选择要操作的线（见文档的表4.2）：   0:Line0 1:Line1 2:Line2
    nRet = MV_CC_SetEnumValue(handle, "LineSelector", 2);  //Line2（3号管脚）为可配置输入或输出
    if (MV_OK != nRet) {
      printf("LineSelector fail.\n");
    }

    //仅LineSelector为line2时需要特意设置（因为只有Line2为可配置输入或输出），其他输出不需要
    nRet = MV_CC_SetEnumValue(handle, "LineMode", 8); // 0:Input 1:Output 8:Strobe
    if (MV_OK != nRet) {
      printf("LineMode fail.\n");
    }
    
    int DurationValue = 0; //Strobe 持续时间,单位为 μs
    int DelayValue = 0;//Strobe 输出延迟
    int PreDelayValue = 0; // Strobe 预输出

    // nRet = MV_CC_SetIntValue(handle, "StrobeLineDuration", DurationValue);
    // if (MV_OK != nRet) {
    //   printf("StrobeLineDuration fail.\n");
    // }

    nRet = MV_CC_SetIntValue(handle, "LineSource", 0);//0 代表 Exposure Start Active。见文档表8-6

    //估计要连线line2，才不会报错。
    //但是这里为什么设置为“相机开始曝光时，输出Strobe信号到外部设备”？要用这个信号控制外部设备（比如：livox）？
    
    if (MV_OK != nRet) {
      printf("Line source:Exposure Start Activate(相机开始曝光时，输出信号到外部设备) fail! nRet [0x%x]\n", nRet);//这里报错了！！！General error
    }

    // strobe持续时间，设置为0，持续时间就是曝光时间，设置其他值，就是其他值时间
    nRet = MV_CC_SetIntValue(handle, "StrobeLineDelay", DelayValue); // strobe延时：从曝光开始延时DelayValue再输出strobe信号
    if (MV_OK != nRet) {
      printf("StrobeLineDelay fail.\n");
    }

    nRet = MV_CC_SetIntValue(handle, "StrobeLinePreDelay", PreDelayValue); // strobe提前：曝光开始前PreDelayValue时间就输出strobe信号
    if (MV_OK != nRet) {
      printf("StrobeLinePreDelay fail.\n");
    }

    // Strobe输出使能，使能之后，上面配置参数生效，IO输出与曝光同步
    nRet = MV_CC_SetBoolValue(handle, "StrobeEnable", 1);
    if (MV_OK != nRet) {
      printf("StrobeEnable fail.\n");
    }
  }
  
  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
  if (MV_OK == nRet) {
    std::string msg = "Set ExposureAutoMode: " + std::to_string(ExposureAutoMode);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set ExposureAutoMode");
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    // nRet = MV_CC_SetFloatValue(handle, "ExposureAuto", ExposureAuto);
    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Auto: " + ExposureAutoStr[ExposureAuto];
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure auto mode");
    }
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) + "ms";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) + "ms";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Time: " + std::to_string(ExposureTime) + "ms";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gain auto mode");
  }

  if (GainAuto == 0) { //关闭GainAuto
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gamma");
  }
}

void PressEnterToExit(void) {
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress enter to exit.\n");
  while (getchar() != '\n')
    ;
  exit_flag = true;
  sleep(1);
}

static void *WorkThread(void *pUser) { //pUser是相机句柄
  int nRet = MV_OK;
  // ch:获取数据包大小 | en:Get payload size
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
  {
    printf("  NULL == pData  \n");
    return NULL;
  }
  //最后，将缓冲区的大小保存在变量nDataSize中，以便后续使用。
  unsigned int nDataSize = stParam.nCurValue;


 
  while (ros::ok()) {

    //从相机中获取一帧图像，存放到pData里，图像信息放在stImageInfo，超时时间1000毫秒
    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      printf("  MV_CC_GetOneFrameTimeout success!  \n");

      // 获取电脑系统当前时间，并将其转换为秒为单位的浮点数。 
      // auto time_pc_clk = std::chrono::high_resolution_clock::now();
      // double time_pc = uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(time_pc_clk.time_since_epoch()).count()) / 1000000000.0;


      // 时间戳 在这里读取
      // int64_t a = pointt->high;
      // int64_t b = pointt->low + 1e8;//这里的单位是纳秒（ns）
      // bias 干掉之后不需要加0.1 s了

      // if (pointt == nullptr) {
      //     printf("\n   pointt == nullptr  空空   \n");
      // } 
      // else{
      //       printf("\n rcv_time \n");
      //       int64_t b = pointt->low;//这里的单位是纳秒（ns）
      //       int64_t a = pointt->high;
      //       ROS_INFO("  a: %f ,  b : %f  \n", a,b);
      //       // double time_pc = b / 1000000000.0;//得到秒（s）为单位的数值
      //       // ros::Time rcv_time = ros::Time(time_pc);//这帧图像的时间戳，后面和图像消息一起发布出去。
      //       // ROS_INFO("  rcv_time: %f  \n", rcv_time.toSec());
      // }


      //如果只连相机的话，共享内存文件为空，这里报错。

      //这里的pointt是共享内存文件中LiDAR的帧头时间戳。 
      int64_t b = pointt->low;//这里的单位是纳秒（ns）
      ROS_INFO("共享文件中，读取到的值为: %ld  \n", b);
      double time_pc = b / 1000000000.0;//得到秒（s）为单位的数值
      ros::Time rcv_time = ros::Time(time_pc);//这帧图像的时间戳，后面和图像消息一起发布出去。

      ROS_INFO("转化为ros时间戳为: rcv_time :  %f \n", rcv_time.toSec());

      std::string debug_msg;
      debug_msg = CameraName + " GetOneFrame,nFrameNum[" + std::to_string(stImageInfo.nFrameNum) + "], FrameTime:" + std::to_string(rcv_time.toSec());
      ROS_INFO_STREAM(debug_msg.c_str());

      cv::Mat srcImage;
      srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      //从相机获取一帧图像数据后，用resize_divider缩放因子对图像宽高进行调整
      cv::resize(srcImage, srcImage, cv::Size(resize_divider * srcImage.cols, resize_divider * srcImage.rows),CV_INTER_LINEAR);

      //将cv::Mat对象转换为ROS图像消息类型，并发布出去。
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", srcImage).toImageMsg();
      msg->header.stamp = rcv_time;
      pub.publish(msg);
    }
    else {
      printf("MV_CC_GetOneFrameTimeout fail!   nRet [0x%x]\n", nRet);
      //break;
    }

    if (exit_flag)
      break;
  }

  if (pData) {
    free(pData);
    pData = NULL;
  }

  return 0;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mvs_trigger");
  std::string params_file = std::string(argv[1]);//left_camera_trigger.yaml
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  int nRet = MV_OK;
  void *handle = NULL;

  ros::Rate loop_rate(10);//设置频率为10！！！！

  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  std::string camera_name = Params["CameraName"];
  // std::string path_for_time_stamp = "/home/aa/test4";
  // nh.getParam("path_for_time_stamp", path_for_time_stamp);
  std::string path_for_time_stamp = Params["path_for_time_stamp"];
  CameraName = camera_name;
  pub = it.advertise(pub_topic, 1);
  const char *shared_file_name = path_for_time_stamp.c_str();

  int fd = open(shared_file_name, O_RDWR);
  //通过mmap函数，文件中的数据就被映射到了进程的地址空间中，并可以通过pointt指针来访问和修改这块内存区域的内容。
  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  
  while (ros::ok()) {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举检测到的相机数量
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("Enum Devices fail!");
      break;
    }

    bool find_expect_camera = false;
    int expect_camera_index = 0;
    if (stDeviceList.nDeviceNum == 0) {
      ROS_ERROR_STREAM("No Camera.\n");
      break;
    } else {
      // 在插入电脑的多个相机中，根据yaml文件中的serial number启动指定相机
      for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        std::string serial_number = std::string((char *)stDeviceList.pDeviceInfo[i] ->SpecialInfo.stUsb3VInfo.chSerialNumber);
        //ROS_INFO("Serial Number: %s", serial_number.c_str()); //我的相机是：00DA0878853
        if (expect_serial_number == serial_number) {
          find_expect_camera = true;
          expect_camera_index = i;
          break;
        }
      }
    }
    if (!find_expect_camera) {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR_STREAM(msg.c_str());
      break;
    }

    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[expect_camera_index]);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Create Handle fail");
      break;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      printf("Open Device fail\n");
      break;
    }

    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);//把“启用采集帧率“设置为false，即：禁用相机的采集帧率限制
    if (MV_OK != nRet) {
      printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
      break;
    }

    // ch:获取数据包大小
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
      printf("Get PayloadSize fail\n");
      break;
    }
    g_nPayloadSize = stParam.nCurValue;//这个变量好像没啥用

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);//设置图像像素格式为0x02180014
    if (nRet != MV_OK) {
      printf("Pixel setting can't work.");
      break;
    }

    setParams(handle, params_file);//left_camera_trigger.yaml

    // 设置触发模式为on
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
      break;
    }

    // 设置触发源 // set trigger source
    //将触发源设置为 Line0。  就是stm32的pwm波引角。
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
      break;
    }

      ROS_INFO("Finish all params set! Start grabbing...");

      nRet = MV_CC_StartGrabbing(handle);
      if (MV_OK != nRet) {
        printf("Start Grabbing fail.\n");
        break;
      }


      pthread_t nThreadID;
      nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);//开启WorkThread线程
      if (nRet != 0) {
        printf("thread create failed.ret = %d\n", nRet);
        break;
      }

      PressEnterToExit();

      nRet = MV_CC_StopGrabbing(handle);
      if (MV_OK != nRet) {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        break;
      }

      nRet = MV_CC_CloseDevice(handle);
      if (MV_OK != nRet) {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        break;
      }

      nRet = MV_CC_DestroyHandle(handle);
      if (MV_OK != nRet) {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        break;
      }

      break;
    }

    //使用了munmap函数来释放通过mmap函数映射的内存区域。munmap函数用于解除对某个内存区域的映射，将其从进程的虚拟地址空间中移除。
    //munmap函数的参数pointt是一个指向映射内存区域起始地址的指针，sizeof(time_stamp) * 5则是映射内存区域的大小。
    munmap(pointt, sizeof(time_stamp) * 5);
    return 0;
}