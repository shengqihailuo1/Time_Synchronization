 #include <string.h>;
 #include "lvx_file.h"
 #include "cmdline.h"
 #include <stdio.h>;
 #include <stdlib.h>;
 #include "lds_lidar.h"
 #include "synchro.h"

DeviceItem devices[kMaxLidarCount];
 LvxFileHandle lvx_file_handler;
 std::list point_packet_list;
 std::vectorstd::string broadcast_code_rev;
 std::condition_variable lidar_arrive_condition;
 std::condition_variable extrinsic_condition;
 std::condition_variable point_pack_condition;
 std::mutex mtx;
 int lvx_file_save_time =10; //设置运行时间(s)
 bool is_finish_extrinsic_parameter = false;
 bool is_read_extrinsic_from_xml = false;
 uint8_t connected_lidar_count = 0;

#define FRAME_RATE 20

using namespace std::chrono;

/** Cmdline input broadcast code */
 static std::vectorstd::string cmdline_broadcast_code;

/** Connect all the broadcast device in default and connect specific device when use program options or broadcast_code_list is not empty. */
 std::vectorstd::string broadcast_code_list = {
 "3GGDJ3300100351"
 };

/** Receiving error message from Livox Lidar. */
 void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
 static uint32_t error_message_count = 0;
 if (message != NULL) {
 ++error_message_count;
 if (0 == (error_message_count % 100)) {
 printf("handle: %u\n", handle);
 printf("temp_status : %u\n", message-&gt;lidar_error_code.temp_status);
 printf("volt_status : %u\n", message-&gt;lidar_error_code.volt_status);
 printf("motor_status : %u\n", message-&gt;lidar_error_code.motor_status);
 printf("dirty_warn : %u\n", message-&gt;lidar_error_code.dirty_warn);
 printf("firmware_err : %u\n", message-&gt;lidar_error_code.firmware_err);
 printf("pps_status : %u\n", message-&gt;lidar_error_code.device_status);
 printf("fan_status : %u\n", message-&gt;lidar_error_code.fan_status);
 printf("self_heating : %u\n", message-&gt;lidar_error_code.self_heating);
 printf("ptp_status : %u\n", message-&gt;lidar_error_code.ptp_status);
 printf("time_sync_status : %u\n", message-&gt;lidar_error_code.time_sync_status);
 printf("system_status : %u\n", message-&gt;lidar_error_code.system_status);
 }
 }
 }

/** Receiving point cloud data from Livox LiDAR. */
 void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
 if (data) {
 if (handle < connected_lidar_count &amp;&amp; is_finish_extrinsic_parameter) {
 std::unique_lockstd::mutex lock(mtx);
 LvxBasePackDetail packet;
 packet.device_index = handle;
 lvx_file_handler.BasePointsHandle(data, packet);
 point_packet_list.push_back(packet);
 }
 }
 }

/** Callback function of starting sampling. */
 void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
 printf("OnSampleCallback statues %d handle %d response %d \n", status, handle, response);
 if (status == kStatusSuccess) {
 if (response != 0) {
 devices[handle].device_state = kDeviceStateConnect;
 }
 } else if (status == kStatusTimeout) {
 devices[handle].device_state = kDeviceStateConnect;
 }
 }

/** Callback function of stopping sampling. */
 void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
 }

/** Callback function of get LiDARs' extrinsic parameter. */
 void OnGetLidarExtrinsicParameter(livox_status status, uint8_t handle, LidarGetExtrinsicParameterResponse *response, void *data) {
 if (status == kStatusSuccess) {
 if (response != 0) {
 printf("OnGetLidarExtrinsicParameter statue %d handle %d response %d \n", status, handle, response-&gt;ret_code);
 std::unique_lockstd::mutex lock(mtx);
 LvxDeviceInfo lidar_info;
 strncpy((char *)lidar_info.lidar_broadcast_code, devices[handle].info.broadcast_code, kBroadcastCodeSize);
 memset(lidar_info.hub_broadcast_code, 0, kBroadcastCodeSize);
 lidar_info.device_index = handle;
 lidar_info.device_type = devices[handle].info.type;
 lidar_info.extrinsic_enable = true;
 lidar_info.pitch = response-&gt;pitch;
 lidar_info.roll = response-&gt;roll;
 lidar_info.yaw = response-&gt;yaw;
 lidar_info.x = static_cast(response-&gt;x / 1000.0);
 lidar_info.y = static_cast(response-&gt;y / 1000.0);
 lidar_info.z = static_cast(response-&gt;z / 1000.0);
 lvx_file_handler.AddDeviceInfo(lidar_info);
 if (lvx_file_handler.GetDeviceInfoListSize() == connected_lidar_count) {
 is_finish_extrinsic_parameter = true;
 extrinsic_condition.notify_one();
 }
 }
 }
 else if (status == kStatusTimeout) {
 printf("GetLidarExtrinsicParameter timeout! \n");
 }
 }

/** Get LiDARs' extrinsic parameter from file named "extrinsic.xml". */
 void LidarGetExtrinsicFromXml(uint8_t handle) {
 LvxDeviceInfo lidar_info;
 ParseExtrinsicXml(devices[handle], lidar_info);
 lvx_file_handler.AddDeviceInfo(lidar_info);
 lidar_info.extrinsic_enable = true;
 if (lvx_file_handler.GetDeviceInfoListSize() == broadcast_code_list.size()) {
 is_finish_extrinsic_parameter = true;
 extrinsic_condition.notify_one();
 }
 }

/** Query the firmware version of Livox LiDAR. */
 void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
 if (status != kStatusSuccess) {
 printf("Device Query Informations Failed %d\n", status);
 }
 if (ack) {
 printf("firm ver: %d.%d.%d.%d\n",
 ack-&gt;firmware_version[0],
 ack-&gt;firmware_version[1],
 ack-&gt;firmware_version[2],
 ack-&gt;firmware_version[3]);
 }
 }

void LidarConnect(const DeviceInfo *info) {
 uint8_t handle = info-&gt;handle;
 QueryDeviceInformation(handle, OnDeviceInformation, NULL);
 if (devices[handle].device_state == kDeviceStateDisconnect) {
 devices[handle].device_state = kDeviceStateConnect;
 devices[handle].info = *info;
 }
 }

void LidarDisConnect(const DeviceInfo *info) {
 uint8_t handle = info-&gt;handle;
 devices[handle].device_state = kDeviceStateDisconnect;
 }

void LidarStateChange(const DeviceInfo *info) {
 uint8_t handle = info-&gt;handle;
 devices[handle].info = *info;
 }

/** Callback function of changing of device state. */
 void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
 if (info == nullptr) {
 return;
 }
 printf("OnDeviceChange broadcast code %s update type %d\n", info-&gt;broadcast_code, type);
 uint8_t handle = info-&gt;handle;
 if (handle &gt;= kMaxLidarCount) {
 return;
 }

if (type == kEventConnect) {
 LidarConnect(info);
 printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info-&gt;broadcast_code);
 } else if (type == kEventDisconnect) {
 LidarDisConnect(info);
 printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info-&gt;broadcast_code);
 } else if (type == kEventStateChange) {
 LidarStateChange(info);
 printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info-&gt;broadcast_code);
 }

if (devices[handle].device_state == kDeviceStateConnect) {
 printf("Device Working State %d\n", devices[handle].info.state);
 if (devices[handle].info.state == kLidarStateInit) {
 printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
 } else {
 printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
 }
 printf("Device feature %d\n", devices[handle].info.feature);
 SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
 if (devices[handle].info.state == kLidarStateNormal) {
 if (!is_read_extrinsic_from_xml) {
 LidarGetExtrinsicParameter(handle, OnGetLidarExtrinsicParameter, nullptr);
 } else {
 LidarGetExtrinsicFromXml(handle);
 }
 LidarStartSampling(handle, OnSampleCallback, nullptr);
 devices[handle].device_state = kDeviceStateSampling;
 }
 }
 }

/** Callback function when broadcast message received.

You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
 void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
 if (info == nullptr || info-&gt;dev_type == kDeviceTypeHub) {
 return;
 }

printf("Receive Broadcast Code %s\n", info-&gt;broadcast_code);
 if ((broadcast_code_rev.size() == 0) ||
 (std::find(broadcast_code_rev.begin(), broadcast_code_rev.end(), info-&gt;broadcast_code) == broadcast_code_rev.end())) {
 broadcast_code_rev.push_back(info-&gt;broadcast_code);
 lidar_arrive_condition.notify_one();
 }
 }

/** Wait until no new device arriving in 2 second. */
 void WaitForDevicesReady( ) {
 bool device_ready = false;
 seconds wait_time = seconds(2);
 steady_clock::time_point last_time = steady_clock::now();
 while (!device_ready) {
 std::unique_lockstd::mutex lock(mtx);
 lidar_arrive_condition.wait_for(lock,wait_time);
 if ((steady_clock::now() - last_time + milliseconds(50)) &gt;= wait_time) {
 device_ready = true;
 } else {
 last_time = steady_clock::now();
 }
 }
 }

void WaitForExtrinsicParameter() {
 std::unique_lockstd::mutex lock(mtx);
 extrinsic_condition.wait(lock);
 }

void AddDevicesToConnect() {
 if (broadcast_code_rev.size() == 0)
 return;

for (int i = 0; i < broadcast_code_rev.size(); ++i) {
 if ((broadcast_code_list.size() != 0) &amp;&amp;
 (std::find(broadcast_code_list.begin(), broadcast_code_list.end(), broadcast_code_rev[i]) == broadcast_code_list.end())) {
 continue;
 }
 uint8_t handle = 0;
 livox_status result = AddLidarToConnect(broadcast_code_rev[i].c_str(), &amp;handle);
 if (result == kStatusSuccess) {
 /** Set the point cloud data for a specific Livox LiDAR. */
 SetDataCallback(handle, GetLidarData, nullptr);
 devices[handle].handle = handle;
 devices[handle].device_state = kDeviceStateDisconnect;
 connected_lidar_count++;
 }
 }
 }

/** Set the program options.

You can input the registered device broadcast code and decide whether to save the log file.
 */
 void SetProgramOption(int argc, const char *argv[]) {
 cmdline::parser cmd;
 cmd.addstd::string("code", 'c', "Register device broadcast code", false);
 cmd.add("log", 'l', "Save the log file");
 cmd.add("time", 't', "Time to save point cloud to the lvx file", false);
 cmd.add("param", 'p', "Get the extrinsic parameter from extrinsic.xml file");
 cmd.add("help", 'h', "Show help");
 cmd.parse_check(argc, const_cast<char **&gt;(argv));
 if (cmd.exist("code")) {
 std::string sn_list = cmd.getstd::string("code");
 printf("Register broadcast code: %s\n", sn_list.c_str());
 size_t pos = 0;
 broadcast_code_list.clear();
 while ((pos = sn_list.find("&amp;")) != std::string::npos) {
 broadcast_code_list.push_back(sn_list.substr(0, pos));
 sn_list.erase(0, pos + 1);
 }
 broadcast_code_list.push_back(sn_list);
 }
 if (cmd.exist("log")) {
 printf("Save the log file.\n");
 SaveLoggerFile();
 }
 if (cmd.exist("time")) {
 printf("Time to save point cloud to the lvx file:%d.\n", cmd.get("time"));
 lvx_file_save_time = cmd.get("time");
 }
 if (cmd.exist("param")) {
 printf("Get the extrinsic parameter from extrinsic.xml file.\n");
 is_read_extrinsic_from_xml = true;
 }
 return;
 }

int main(int argc, const char argv[]) {
 /* Set the program options. */
 SetProgramOption(argc, argv);

LdsLidar&amp; read_lidar = LdsLidar::GetInstance();

int ret = read_lidar.InitLdsLidar(cmdline_broadcast_code);
 if (!ret) {
 printf("Init lds lidar success!\n");
 } else {
 printf("Init lds lidar fail!\n");
 return 0;
 }

printf("Start discovering device.\n");

Synchro&amp; synchro = Synchro::GetInstance();
 #ifdef WIN32
 synchro.SetPortName("COM1");
 #else
 synchro.SetPortName("/dev/ttyUSB0");
 #endif

/** Set Synchro's Baudrate: 115200. /
 synchro.SetBaudRate(BR115200);
 /* Set Synchro's Parity: No parity (8N1). /
 synchro.SetParity(P_8N1);
 /* SyncTimer Callback will trigger when Synchro's GPRMC/GNRMC signal is ready. */
 synchro.SetSyncTimerCallback(read_lidar.SetRmcSyncTime);
 if (synchro.Start()) {
 printf("Synchro start success\n");
 } else {
 printf("Synchro start failed\n");
 return 0;
 }

std::this_thread::sleep_for(std::chrono::seconds(2));

//synchro.Stop();
 read_lidar.DeInitLdsLidar();
 //printf("Livox lidar demo end!\n");
 std::this_thread::sleep_for(std::chrono::seconds(2));

/*********************************************************/
 printf("Livox SDK initializing.\n");
 //Initialize Livox-SDK.
 if (!Init()) {
 return -1;
 }
 printf("Livox SDK has been initialized.\n");

LivoxSdkVersion _sdkversion;
 GetLivoxSdkVersion(&amp;_sdkversion);
 printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

memset(devices, 0, sizeof(devices));

/** Set the callback function receiving broadcast message from Livox LiDAR. */
 SetBroadcastCallback(OnDeviceBroadcast);

/** Set the callback function called when device state change,

which means connection/disconnection and changing of LiDAR state.
 */
 SetDeviceStateUpdateCallback(OnDeviceInfoChange);

/** Start the device discovering routine. */
 if (!Start()) {
 Uninit();
 return -1;
 }
 printf("Start discovering device.\n");

WaitForDevicesReady();

AddDevicesToConnect();

if (connected_lidar_count == 0) {
 printf("No device will be connected.\n");
 Uninit();
 return -1;
 }

WaitForExtrinsicParameter();

/等待GPS同步后, 才记录点云数据生成.lvx文件*******/

printf("Start initialize lvx file.\n");
 if (!lvx_file_handler.InitLvxFile()) {
 Uninit();
 return -1;
 }
 std::this_thread::sleep_for(std::chrono::seconds(5));
 lvx_file_handler.InitLvxFileHeader();
 int i = 0;
 steady_clock::time_point last_time = steady_clock::now();
 for (i = 0; i < lvx_file_save_time * FRAME_RATE; ++i) {
 std::list point_packet_list_temp;
 {
 std::unique_lockstd::mutex lock(mtx);
 point_pack_condition.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
 last_time = steady_clock::now();
 point_packet_list_temp.swap(point_packet_list);
 }
 if(point_packet_list_temp.empty()) {
 printf("Point cloud packet is empty.\n");
 break;
 }
 printf("Finish save %d frame to lvx file.\n", i);
 lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
 }

lvx_file_handler.CloseLvxFile();

for (i = 0; i < kMaxLidarCount; ++i) {
 if (devices[i].device_state == kDeviceStateSampling) {
 /** Stop the sampling of Livox LiDAR. */
 LidarStopSampling(devices[i].handle, OnStopSampleCallback, nullptr);
 }
 }

/** Uninitialize Livox-SDK. */
 Uninit();
 }