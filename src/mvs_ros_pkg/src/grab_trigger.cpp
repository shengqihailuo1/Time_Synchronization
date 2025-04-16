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
#include <sensor_msgs/TimeReference.h>
#include <vector>

#include <thread>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;
struct my_t {
    int64_t gps_tt;//stm32发送过来的utc时间,自1970年到现在的纳秒
    //std::chrono::high_resolution_clock::time_point pc_tt;
    long pc_tt;//自1970年到当前接受到gps时刻所经过的ns数量。
};
my_t *my_pointt; // 共享内存指针,用于实现时间戳同步

using namespace std;

// // 海康相机采集到图像后的回调函数
// void __stdcall ImageCallBackEx(
//     unsigned char * pData, 
//     MV_FRAME_OUT_INFO_EX* pFrameInfo, 
//     void* pUser)
// {
//     if (pFrameInfo)
//     {
//         std::cout << "获取到一帧图像，大小: " << pFrameInfo->nWidth << "x" << pFrameInfo->nHeight << std::endl;
//         // 这里可以添加图像处理的代码
//     }
// }

//接受到livox驱动发过来的时间戳数据后，执行这个回调函数
std::vector<std::pair<double, double>> time_reference_buffer;// <系统时间，utc时间>
void timeReferenceCallback(const sensor_msgs::TimeReference::ConstPtr& msg) {
  //发送过来的msg->header.stamp和msg->time_ref是ros::time()格式的时间
  //消息队列，怎么解决？
    double header_stamp = msg->header.stamp.toSec();
    double time_ref = msg->time_ref.toSec();
    time_reference_buffer.push_back(std::make_pair(header_stamp, time_ref));//转化为秒的格式
}

//根据系统时间，找到最接近的数据对。
std::pair<double, double> findClosestPair(const std::vector<std::pair<double, double>>& buffer, double current_time) {
    if (buffer.empty()) {
        // 如果 buffer 为空，则返回一个默认的 pair
        return std::make_pair(0.0, 0.0);
    }
    // 初始化最小距离和最接近的数据对
    double min_distance = std::abs(buffer[0].first - current_time);
    std::pair<double, double> closest_pair = buffer[0];
    // 遍历整个 buffer，找到最接近的数据对
    for (const auto& pair : buffer) {
        double distance = std::abs(pair.first - current_time);//系统时间之差
        if (distance < min_distance) {
            min_distance = distance;
            closest_pair = pair;
        }
    }
    //相差26ms
    printf("当前系统时间为：%f s,找到最接近的系统时间为：%f s,相差时间为：%f s \n", current_time,closest_pair.first,current_time-closest_pair.first);
    return closest_pair;
}

unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
image_transport::Publisher pub;
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
std::string CameraName;
float resize_divider;
double ExposureTime_global = 0.0;
long align_thre ;//// 用于判断gps和cam是否足够接近,,阈值设置为半个周期,单位ns 
std::mutex mu;//互斥锁
int per_PPS_cnt = 0;         // 当前PPS之后已经收到的帧数
bool update_flag = false;    // gps时间是否更新
bool first_aligned_flag = false; // 是否完成第一次对齐
long gps_ns, last_frame_ns;
int64_t num; //当前帧是接受到gprmc信号后的第num帧image
int64_t gps_t = -1;//最近的gprmc里面的时间,初始化为无效值.
int64_t cap_image_T_ns;//捕获图像的周期,单位:ns

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
  ExposureTime_global = ExposureTime;//作为全局变量
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];

  int CenterAlign = Params["CenterAlign"];

  int nRet;
  if (CenterAlign) { //这里开始 设置中心对齐 
    // Strobe输出，好像没用到吧？？可以用于多个相机同步触发。
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
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);//yaml文件里面没有！
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
      std::string msg = "Set Exposure Time: " + std::to_string(ExposureTime) + "us";//微秒
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




/**
 * @brief thread for update GPS timestamp
 * TODO:换成消息队列以提高性能
 */
void queryTimeLoop() {
    // 高频率查看是否收到新的GPS时间戳
    while (true) {
        //尝试对互斥锁mu进行加锁的操作。如果当前线程能够成功获得锁，try_lock()方法会返回true，表示成功获取到了锁；否则，如果锁已经被其他线程占用，则try_lock()会立即返回false，表示获取锁失败。这种非阻塞的尝试加锁的方式可以避免线程因为等待锁而被阻塞，从而提高程序的响应速度。
        if (mu.try_lock() )
        {
          if(gps_t != my_pointt->gps_tt) { // 内存中的gps时间变化了，需要更新gps接收时的pc时间,cast to ns
            gps_t = my_pointt->gps_tt; //更新，接受到的gprmc里面的时间
            gps_ns = my_pointt->pc_tt;//开始发送这条gprmc消息的时间（已补偿了串口传输延迟）,就是gprmc上升沿时间
            update_flag = true;//标志位，gprmc的时间发生更新。
            first_aligned_flag = true;//完成第一次对齐
            mu.unlock();//把互斥锁丢掉。
            // 等待下一秒
            std::this_thread::sleep_for(std::chrono::milliseconds(800));//等800ms（理论是1000ms就会有下一次gps时间更新）
          }
          mu.unlock(); // 内存中的gps时间没有变化的话，就把互斥锁丢掉。
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));//等待1ms
    }
}


static void *WorkThread(void *pUser) { //pUser是相机句柄
  int nRet = MV_OK;
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


  // 在多线程中订阅 ROS 话题时，通常建议为每个线程创建一个私有节点句柄，以确保线程间的独立性和稳定性。
  ros::NodeHandle privateNh("~");  // 创建私有节点句柄
  //订阅livox发过来的时间戳信息  //然后执行 timeReferenceCallback() 回调函数
  ros::Subscriber time_reference_sub = privateNh.subscribe("/mvs_camera_trigger/image_time_reference", 1, timeReferenceCallback);


  while (ros::ok()) {

    ros::spinOnce(); // 处理订阅的消息
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));// 等待一段时间，以免循环过快

    //从相机中获取一帧图像，存放到pData里，图像信息放在stImageInfo，超时时间1000毫秒
    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
        //printf("  MV_CC_GetOneFrameTimeout success!  \n");
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
      /*
      //使用共享内存的方法
      int64_t b = pointt->low;//这里的单位是纳秒（ns）//这里的pointt是共享内存文件中LiDAR的帧头时间戳。 
      printf("camera 从共享内存文件中读取的timestamp为:  %ld \n",b);
      double time_pc = b / 1000000000.0;//得到秒（s）为单位的数值
      ros::Time rcv_time = ros::Time(time_pc);//这帧图像的时间戳，后面和图像消息一起发布出去。
      //ROS_INFO("转化为ros时间戳为: rcv_time :  %f \n", rcv_time.toSec());
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
      */

      //这段if的代码运行时间1ms

      ros::Time current_time_image = ros::Time::now();  // 当前系统时间，一般精度在几毫秒到微秒级别之间。
      double time_sec = current_time_image.toSec(); // 将 ROS 时间转换为秒
      current_time_image = ros::Time().fromSec(time_sec- (ExposureTime_global / 1000000000.0f )); // 将秒转换回 ROS 时间对象,减去曝光时间。

      // std::chrono::high_resolution_clock::time_point now_time = std::chrono::high_resolution_clock::now();
      // long long now_time_sec = std::chrono::duration_cast<std::chrono::seconds>(now_time.time_since_epoch()).count();
      // // 将当前时间减去图像曝光时间。 ExposureTime_global 单位：ns
      // ros::Time current_time(now_time_sec - ExposureTime_global / 1000000000.0f);//单位：秒

      // closest_pair 中存放的就是与 current_time_image 最接近的数据对
      std::pair<double, double> closest_pair = findClosestPair(time_reference_buffer, current_time_image.toSec());

      cv::Mat srcImage;
      srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      //从相机获取一帧图像数据后，用 resize_divider 缩放因子对图像宽高进行调整
      cv::resize(srcImage, srcImage, cv::Size(resize_divider * srcImage.cols, resize_divider * srcImage.rows),CV_INTER_LINEAR);

      //将cv::Mat对象转换为ROS图像消息类型，并发布出去。
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", srcImage).toImageMsg();
      
      msg->header.stamp = ros::Time(closest_pair.second);//最接近的相机采样utc时间

      pub.publish(msg);
      printf("发布图像数据成功,时间为:%f s\n",closest_pair.second);

      //nDevTimeStamp: 设备时间戳，表示相机采集图像的时间戳。
      //printf("相机上电到现在共经过了: %f 秒\n", (((uint64_t)stImageInfo.nDevTimeStampHigh << 32) | stImageInfo.nDevTimeStampLow)/100000000.0);
      //调试出：两帧之差为2500000 纳秒（ns），也就是25ms。because帧率设置为40hz，所以这个结果是正确的。

      //printf("主机产生的时间戳：%ld",stImageInfo.nHostTimeStamp);//显示当前系统时间;单位：毫秒ms，类似用ros::Time::now()，但是后者时间精度会再高点，达到0.001毫秒。
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

  ros::init(argc, argv, "mvs_trigger");//节点名称
  std::string params_file = std::string(argv[1]);//left_camera_trigger.yaml
  ros::NodeHandle nh;
  //下面的操作是为了对opencv格式图像进行转换，才能用ros格式发出去
  image_transport::ImageTransport it(nh);
  int nRet = MV_OK;
  void *handle = NULL;


  ros::Rate loop_rate(60);//设置频率为10！！！！  40 

  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];//left_camera/image
  std::string camera_name = Params["CameraName"];
  // nh.getParam("path_for_time_stamp", path_for_time_stamp);
  std::string path_for_time_stamp = Params["path_for_time_stamp"];
  std::string path_for_livox_lidar_config_json = Params["path_for_livox_lidar_config_json"];



  //读取livox_lidar_config.json 文件，根据里面的camera_publish_freq 获取相机频率（20hz或40hz），用于设置一些参数。
  const char *livox_lidar_config_json_file = path_for_livox_lidar_config_json.c_str();
  int camera_publish_freq;
  FILE *raw_file = std::fopen(livox_lidar_config_json_file, "rb");
  if (!raw_file) {
    printf("Open json config file fail!\n");
    return -1;
  }
  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  rapidjson::Document doc;
  if (!doc.ParseStream(config_file).HasParseError()) {
    const rapidjson::Value &object = doc["timesync_config"];
    camera_publish_freq = object["camera_publish_freq"].GetInt();//获取相机发布频率
  }
  if(camera_publish_freq == 20){
    align_thre = 25000000; //取周期的一半
    cap_image_T_ns = 50000000;//捕获图像的周期,单位:ns
  }else if(camera_publish_freq == 40){
    align_thre = 12500000; //取周期的一半
    cap_image_T_ns = 25000000;//捕获图像的周期,单位:ns
  }else if(camera_publish_freq == 10){
    align_thre = 50000000; //取周期的一半
    cap_image_T_ns = 100000000;//捕获图像的周期,单位:ns
  }else{
    printf("相机采样频率应该是10,20或40,请检查 livox_lidar_config.json 文件\n");
    exit(-1);
  }


  const char *shared_file_name = path_for_time_stamp.c_str();
  //int fd = open(shared_file_name, O_RDWR);
  int fd = open( shared_file_name, O_CREAT | O_RDWR , 0777);
  //通过mmap函数，文件中的数据就被映射到了进程的地址空间中，并可以通过pointt指针来访问和修改这块内存区域的内容。
  //pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if(fd == -1)
  {
    ROS_ERROR("failed to open share mm!\n");
    exit(-1);
  }
  my_pointt = (my_t *)mmap(NULL, sizeof(my_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);


  CameraName = camera_name;
  pub = it.advertise(pub_topic, 1);//后面要把image发布到这个话题上


  //注意：这个while循环只执行一次就break了，但是里面创建了一个线程，并且还有PressEnterToExit函数在等待退出。
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


    // // 注册回调函数
    // nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, nullptr);
    // if (MV_OK != nRet)
    // {
    //     std::cout << "注册回调函数失败!" << std::endl;
    //     return -1;
    // }

      ROS_INFO("Finish all params set! Start grabbing...");
      nRet = MV_CC_StartGrabbing(handle);
      if (MV_OK != nRet) {
        printf("Start Grabbing fail.\n");
        break;
      }






  MV_FRAME_OUT_INFO_EX stImageInfo = {0};//该结构体用于存储图像的相关信息，例如图像宽度、高度、帧率等。
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));//初始化为0
  //使用malloc函数为图像数据的缓冲区分配内存空间。缓冲区的大小为stParam.nCurValue字节
  unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData)
  {
    printf("  NULL == pData  \n");//alloc memory for img data failed.
    return NULL;
  }
  //最后，将缓冲区的大小保存在变量nDataSize中，以便后续使用。
  //unsigned int nDataSize = stParam.nCurValue;

        // create a thread to update gps time stamp
        // pthread_create(&nThreadID, NULL, queryTimeLoop, NULL);
        std::thread t(queryTimeLoop);//std::thread 会自动管理线程的生命周期，当 t 对象销毁时，线程也会被join或者detach。
        // t.join();//阻塞调用，它会使主线程等待直到被调用的线程t执行完成。
        ROS_INFO("wariting first align...");// 等待第一次时间消息到来（第一次接收到gprmc时间后就会设置 first_aligned_flag == true）
        
        while (true)
        {
            if(mu.try_lock())
            {
                if(first_aligned_flag)//完成第一次对齐，要求：接受到第一个gprmc后才开始采样图像。
                {
                    ROS_INFO("begin acquisition!");
                    mu.unlock();
                    break;
                }
                mu.unlock();
            }
            ROS_INFO("waiting lidar reready");
            //std::this_thread::sleep_for(std::chrono::milliseconds(5));
            //ros::Duration(0.01).sleep();
        }
            


              //下面这块改为回调函数里面执行!

              
        /*
         *  采集图像的循环, core code begins here 
         */
        while (ros::ok()) {
            // 尝试采集一帧图像
            nRet = MV_CC_GetOneFrameTimeout(handle, pData, stParam.nCurValue,
                                            &stImageInfo, 1000);
            // 上升沿触发
            if (nRet == MV_OK) {
                // 记录新帧pc时间, record the pc time ns for the new frame

                auto this_frame_ns = //获取当前时间戳（以纳秒为单位）
                  std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                




                this_frame_ns -= static_cast<long>( ExposureTime_global * 1000); //补偿曝光时间,得到上升沿的时间.
                
                //标定结果: t_imu = t_cam + 0.020026s   试下把上面这行代码注释掉,刚好20ms







                mu.lock(); // 上锁
                long last_diff = last_frame_ns - gps_ns;
                long this_diff = this_frame_ns - gps_ns;

                // 如果发现GPS时间发生了更新，就选择这个时刻相机的左/右帧作为对齐gprmc的帧
                if (update_flag) {
                    // 分别计算当前帧和上一帧与gps时间的差值，找到最近的那个 //注意：此时last_diff为负
                    int incre = 0;
                    bool alinged_flag = false;
                    if (abs(last_diff) <= abs(this_diff)) {//将上一帧作为对齐帧 
                        if (abs(last_diff) < align_thre) {
                            incre = 2; // 当前帧是对齐之后的第二帧
                            alinged_flag = true;
                        }
                    } else {//将当前帧作为对齐帧 
                        if (abs(this_diff) < align_thre) {
                            incre = 1; // 当前帧是对齐之后的第一帧
                            alinged_flag = true;
                        }
                    }

                    // 更新了gps时间，但是左右两帧都距离GPS的时间不够近（即：距离都大于align_thre阈值45ms。它相机是10hz，100ms就一帧）,则说明相机丢帧
                    if (!alinged_flag) {
                      printf("---------------------------\n");
                      printf(" 更新了gps时间,但是左右两帧image都距离GPS的时间不够近!\n");
                      
                      incre = (this_diff + cap_image_T_ns / 2) / cap_image_T_ns + 1;
                      //如果是40hz：
                      //incre = (this_diff + 12500000)/25000000 + 1;//(0,12.5】是第一帧，(12.5，37.5】是第二帧，依次类推。
                      //如果是20hz：
                      //incre = (this_diff + 25000000)/50000000 + 1;//(0,25】是第一帧，(25，75】是第二帧，依次类推。
                      
                      ROS_WARN_STREAM("[camera ]Lost when PPS fired " << incre << " frames.");
                      //lost_cnt += incre;//相机的丢帧总数   没用到
                    }
                    per_PPS_cnt = incre ; // 当前这帧图像是接受gprmc之后收到的第per_PPS_cnt帧
                    num = per_PPS_cnt;//我添加的
                    update_flag = false;//更新变量
                } else // 若最近没有发生gps更新
                {
                    // 通过this_diff（单位ns） 计算这是pps内的第几帧, 四舍五入
                    //int64_t num = (this_diff + 50000000) / 100000000;//100ms=0.1s //原来的。image是10hz
                    //int64_t num = (this_diff + 周期的一半) / 周期;//单位ns

                    num = (this_diff + cap_image_T_ns / 2) / cap_image_T_ns + 1;
                    //如果是40hz：
                    //num = (this_diff + 12500000)/25000000+1;//[0,12.5)是第一帧，[12.5，37.5)是第二帧，依次类推。
                    //如果是20hz：
                    //num = (this_diff + 25000000)/50000000+1;//[0,25)是第一帧，[25，75)是第二帧，依次类推。
                    
                    // 丢帧判断
                    if (num - per_PPS_cnt > 1) {//应该是 >= 1 吧
                      //当前这个pps信号内（1s内）共丢失了（num - per_PPS_cnt）帧image
                      ROS_WARN_STREAM("[camera] Lost frame between PPS " << num - per_PPS_cnt << " frames.");
                    } else {
                        per_PPS_cnt++;// 当前PPS之后已经收到的相机帧数++
                    }
                }

                printf("(用时间算出来)当前帧是接受到gprmc信号后的第 %ld 帧image。\n",num);//理论上，应该从1递增到20或40.
                
                double pub_t_cam = gps_t + num * cap_image_T_ns;
                //如果是20hz：
                //double pub_t_cam = gps_t + num * 50000000;
                //如果是40hz:
                //double pub_t_cam = gps_t + num * 25000000;


                mu.unlock(); // 解锁
                if(pub_t_cam <= 0){
                  printf(" pub_t_cam = %f < = 0 ,gps_t = %ld ,num = %ld \n",pub_t_cam,gps_t,num);
                }else{
                  ros::Time rcv_time = ros::Time(pub_t_cam/1000000000);
                  cv::Mat srcImage;
                  srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth,CV_8UC3, pData);
                  cv::resize(srcImage, srcImage,
                            cv::Size(resize_divider * srcImage.cols,resize_divider * srcImage.rows),CV_INTER_LINEAR);

                  sensor_msgs::ImagePtr msg =
                      cv_bridge::CvImage(std_msgs::Header(), "rgb8", srcImage).toImageMsg();
                  msg->header.stamp = rcv_time;
                  pub.publish(msg);

                  last_frame_ns = this_frame_ns;
                }
            } else { // sleep for a while
                printf("获取一张图像失败！\n");
                ros::Duration(0.001).sleep();
            } // process one frame

        } // end of acquisition inf loop

    
    


    //下面这个是原来的，用rostopic订阅相机时间戳的方法。上面是用共享内存。

      // pthread_t nThreadID;
      // nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);//开启WorkThread线程
      // if (nRet != 0) {
      //   printf("thread create failed.ret = %d\n", nRet);
      //   break;
      // }

      // PressEnterToExit();//在这里有个while循环一直等着。。。

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