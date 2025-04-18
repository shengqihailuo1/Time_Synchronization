
#ifndef _MV_CAMERA_PARAMS_H_
#define _MV_CAMERA_PARAMS_H_

#include "PixelType.h"

#ifndef __cplusplus
typedef char bool;
#define true 1
#define false 0
#endif

// ch:?��??????? | en:Device Type Definition
#define MV_UNKNOW_DEVICE                                                       \
  0x00000000                      // ch:��??��????????????? | en:Unknown Device Type, Reserved
#define MV_GIGE_DEVICE 0x00000001 // ch:GigE?�� | en:GigE Device
#define MV_1394_DEVICE 0x00000002 // ch:1394-a/b ?�� | en:1394-a/b Device
#define MV_USB_DEVICE 0x00000004  // ch:USB3.0 ?�� | en:USB3.0 Device
#define MV_CAMERALINK_DEVICE                                                   \
  0x00000008 // ch:CameraLink?�� | en:CameraLink Device

typedef struct _MV_GIGE_DEVICE_INFO_ {
  unsigned int nIpCfgOption;
  unsigned int
      nIpCfgCurrent;       // IP configuration:bit31-static bit30-dhcp bit29-lla
  unsigned int nCurrentIp; // curtent  | en:
  unsigned int nCurrentSubNetMask; // curtent subnet mask
  unsigned int nDefultGateWay;     // current gateway | en
  unsigned char chManufacturerName[32];
  unsigned char chModelName[32];
  unsigned char chDeviceVersion[32];
  unsigned char chManufacturerSpecificInfo[48];
  unsigned char chSerialNumber[16];
  unsigned char chUserDefinedName[16];

  unsigned int nNetExport; // ch:????IP??? | en:GIGE IP Address

  unsigned int nReserved[4];

} MV_GIGE_DEVICE_INFO;

#define INFO_MAX_BUFFER_SIZE 64 //?????256
typedef struct _MV_USB3_DEVICE_INFO_ {
  unsigned char CrtlInEndPoint; // ch:?????????? | en:Control input endpoint
  unsigned char CrtlOutEndPoint; // ch:?????????? | en:Control output endpoint
  unsigned char StreamEndPoint; // ch:????? | en:Flow endpoint
  unsigned char EventEndPoint;  // ch:?????? | en:Event endpoint
  unsigned short idVendor;  // ch:?????ID?? | en:Vendor ID Number
  unsigned short idProduct; // ch:???ID?? | en:Device ID Number
  unsigned int nDeviceNumber; // ch:?��???��?  | en:Device Serial Number
  unsigned char chDeviceGUID[INFO_MAX_BUFFER_SIZE]; // ch:?��GUID?? | en:Device
                                                    // GUID Number
  unsigned char
      chVendorName[INFO_MAX_BUFFER_SIZE]; // ch:????????? | en:Vendor Name
  unsigned char chModelName[INFO_MAX_BUFFER_SIZE]; // ch:??????? | en:Model Name
  unsigned char
      chFamilyName[INFO_MAX_BUFFER_SIZE]; // ch:???????? | en:Family Name
  unsigned char
      chDeviceVersion[INFO_MAX_BUFFER_SIZE]; // ch:?��?��?? | en:Device Version
  unsigned char chManufacturerName[INFO_MAX_BUFFER_SIZE]; // ch:?????????? |
                                                          // en:Manufacturer
                                                          // Name
  unsigned char
      chSerialNumber[INFO_MAX_BUFFER_SIZE]; // ch:???��? | en:Serial Number
  unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE]; // ch:???????????? |
                                                         // en:User Defined Name
  unsigned int nbcdUSB; // ch:????USB��?? | en:Support USB Protocol
  unsigned int nReserved[3]; // ch:??????? | en:Reserved bytes
} MV_USB3_DEVICE_INFO;

typedef struct _MV_CamL_DEV_INFO_ {
  unsigned char chPortID[INFO_MAX_BUFFER_SIZE];
  unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
  unsigned char chFamilyName[INFO_MAX_BUFFER_SIZE];
  unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
  unsigned char chManufacturerName[INFO_MAX_BUFFER_SIZE];
  unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
  unsigned int nReserved[38];
} MV_CamL_DEV_INFO;

// ch:?��??? | en:Device Infomation
typedef struct _MV_CC_DEVICE_INFO_ {
  // ch:common info | en:common info
  unsigned short nMajorVer;
  unsigned short nMinorVer;
  unsigned int nMacAddrHigh; // ch:MAC ??? | en:MAC Address
  unsigned int nMacAddrLow;

  unsigned int nTLayerType; // ch:?��?????��???????e.g. MV_GIGE_DEVICE | en:Device
                            // Transport Layer Protocol Type, e.g.
                            // MV_GIGE_DEVICE

  unsigned int nReserved[4];

  union {
    MV_GIGE_DEVICE_INFO stGigEInfo;
    MV_USB3_DEVICE_INFO stUsb3VInfo;
    MV_CamL_DEV_INFO stCamLInfo;
    // more ...
  } SpecialInfo;

} MV_CC_DEVICE_INFO;

// ch:???��????????? | en:Network transmission information
typedef struct _MV_NETTRANS_INFO_ {
  int64_t nReviceDataSize; // ch:??????????��
                           // [???StartGrabbing??StopGrabbing??????????] |
                           // en:Received Data Size  [Calculate the Data Size
                           // between StartGrabbing and StopGrabbing]
  int nThrowFrameCount; // ch:??????? | en:Throw frame number
  unsigned int nNetRecvFrameCount;
  int64_t nRequestResendPacketCount; // ???????????
  int64_t nResendPacketCount;        // ???????

} MV_NETTRANS_INFO;

// ch:?????????????????? | en:The maximum number of supported transport layer
// instances
#define MV_MAX_TLS_NUM 8
// ch:????????��???? | en:The maximum number of supported devices
#define MV_MAX_DEVICE_NUM 256

// ch:?��????��? | en:Device Information List
typedef struct _MV_CC_DEVICE_INFO_LIST_ {
  unsigned int nDeviceNum; // ch:?????��???? | en:Online Device Number
  MV_CC_DEVICE_INFO *pDeviceInfo[MV_MAX_DEVICE_NUM]; // ch:??????256???�� |
                                                     // en:Support up to 256
                                                     // devices

} MV_CC_DEVICE_INFO_LIST;

// ch:????????? | en:Output Frame Information
typedef struct _MV_FRAME_OUT_INFO_ {
  unsigned short nWidth;            // ch:???? | en:Image Width
  unsigned short nHeight;           // ch:???? | en:Image Height
  enum MvGvspPixelType enPixelType; // ch:?????? | en:Pixel Type

  unsigned int nFrameNum; // ch:??? | en:Frame Number
  unsigned int nDevTimeStampHigh; // ch:??????32�� | en:Timestamp high 32 bits
  unsigned int nDevTimeStampLow; // ch:??????32�� | en:Timestamp low 32 bits
  unsigned int nReserved0; // ch:??????8?????? | en:Reserved, 8-byte aligned
  int64_t nHostTimeStamp; // ch:????????????? | en:Host-generated timestamp

  unsigned int nFrameLen;

  unsigned int nLostPacket; // ?????????
  unsigned int nReserved[2];
} MV_FRAME_OUT_INFO;

// ch:Chunk???? | en:The content of ChunkData
typedef struct _MV_CHUNK_DATA_CONTENT_ {
  unsigned char *pChunkData;
  unsigned int nChunkID;
  unsigned int nChunkLen;

  unsigned int nReserved[8]; // ????

} MV_CHUNK_DATA_CONTENT;

// ch:????????? | en:Output Frame Information
typedef struct _MV_FRAME_OUT_INFO_EX_ {
  unsigned short nWidth;            // ch:???? | en:Image Width
  unsigned short nHeight;           // ch:???? | en:Image Height
  enum MvGvspPixelType enPixelType; // ch:?????? | en:Pixel Type

  unsigned int nFrameNum; // ch:??? | en:Frame Number
  unsigned int nDevTimeStampHigh; // ch:??????32�� | en:Timestamp high 32 bits
  unsigned int nDevTimeStampLow; // ch:??????32�� | en:Timestamp low 32 bits
  unsigned int nReserved0; // ch:??????8?????? | en:Reserved, 8-byte aligned
  int64_t nHostTimeStamp; // ch:????????????? | en:Host-generated timestamp

  unsigned int nFrameLen;

  // ch:?????chunk????????? | en:The followings are chunk add frame-specific
  // information
  // ch:?��????? | en:Device frame-specific time scale
  unsigned int nSecondCount;
  unsigned int nCycleCount;
  unsigned int nCycleOffset;

  float fGain;
  float fExposureTime;
  unsigned int nAverageBrightness; // ch:??????? | en:Average brightness

  // ch:???????? | en:White balance
  unsigned int nRed;
  unsigned int nGreen;
  unsigned int nBlue;

  unsigned int nFrameCounter;
  unsigned int nTriggerIndex; // ch:???????? | en:Trigger Counting

  // ch:Line ????/??? | en:Line Input/Output
  unsigned int nInput;  // ch:???? | en:Input
  unsigned int nOutput; // ch:??? | en:Output

  // ch:ROI???? | en:ROI Region
  unsigned short nOffsetX;
  unsigned short nOffsetY;
  unsigned short nChunkWidth;
  unsigned short nChunkHeight;

  unsigned int nLostPacket; // ch:????????? | en:Lost Pacekt Number In This Frame

  unsigned int nUnparsedChunkNum;
  union {
    MV_CHUNK_DATA_CONTENT *pUnparsedChunkContent;
    int64_t nRet;
  } UnparsedChunkList;

  unsigned int nReserved[36]; // ????
} MV_FRAME_OUT_INFO_EX;

// ch:?????��??????????????????? | en:Image Struct, output the pointer of Image
// and the information of the specific image
typedef struct _MV_FRAME_OUT_ {
  unsigned char *pBufAddr; // ch:???????? | en: pointer of image
  MV_FRAME_OUT_INFO_EX
      stFrameInfo; // ch:?????? | en:information of the specific image

  unsigned int nRes[16]; // ch:???? | en:reserved
} MV_FRAME_OUT;

typedef struct _MV_DISPLAY_FRAME_INFO_ {
  void *hWnd;             // ch:?????? | en:HWND
  unsigned char *pData;   // ch:????????? | en:Data Buffer
  unsigned int nDataLen;  // ch:??????? | en:Data Size
  unsigned short nWidth;  // ch:???? | en:Width
  unsigned short nHeight; // ch:???? | en:Height
  enum MvGvspPixelType enPixelType; // ch:?????? | en:Pixel format
  unsigned int nRes[4];

} MV_DISPLAY_FRAME_INFO;

// ch:????????? | en:Save image type
enum MV_SAVE_IAMGE_TYPE {
  MV_Image_Undefined = 0,
  MV_Image_Bmp = 1,
  MV_Image_Jpeg = 2,
  MV_Image_Png = 3, // ch:????? | en:Not support
  MV_Image_Tif = 4, // ch:????? | en:Not support
};
// ch:?????????? | en:Save image type
typedef struct _MV_SAVE_IMAGE_PARAM_T_ {
  unsigned char *pData; // [IN]     ch:??????????? | en:Input Data Buffer
  unsigned int nDataLen; // [IN]    ch:?????????�� | en:Input Data Size
  enum MvGvspPixelType
      enPixelType;        // [IN]     ch:??????????????? | en:Input Data Pixel Format
  unsigned short nWidth;  // [IN]     ch:???? | en:Image Width
  unsigned short nHeight; // [IN]     ch:???? | en:Image Height

  unsigned char *pImageBuffer; // [OUT]    ch:????????? | en:Output Image Buffer
  unsigned int nImageLen; // [OUT]    ch:???????�� | en:Output Image Size
  unsigned int nBufferSize; // [IN]     ch:???????????????�� | en:Output buffer
                            // size provided
  enum MV_SAVE_IAMGE_TYPE
      enImageType; // [IN]     ch:???????? | en:Output Image Format

} MV_SAVE_IMAGE_PARAM;

// ch:????????? | en:Save Image Parameters
typedef struct _MV_SAVE_IMAGE_PARAM_T_EX_ {
  unsigned char *pData; // [IN]     ch:??????????? | en:Input Data Buffer
  unsigned int nDataLen; // [IN]     ch:?????????�� | en:Input Data Size
  enum MvGvspPixelType
      enPixelType;        // [IN]     ch:??????????????? | en:Input Data Pixel Format
  unsigned short nWidth;  // [IN]     ch:???? | en:Image Width
  unsigned short nHeight; // [IN]     ch:???? | en:Image Height

  unsigned char *pImageBuffer; // [OUT]    ch:????????? | en:Output Image Buffer
  unsigned int nImageLen; // [OUT]    ch:???????�� | en:Output Image Size
  unsigned int nBufferSize; // [IN]     ch:???????????????�� | en:Output buffer
                            // size provided
  enum MV_SAVE_IAMGE_TYPE
      enImageType; // [IN]     ch:???????? | en:Output Image Format
  unsigned int nJpgQuality; // [IN]     ch:????????, (50-99] | en:Encoding
                            // quality, (50-99]

  // [IN]     ch:Bayer?????RGB24????????  0-????? 1-????? 2-Hamilton
  // ??????????????????????????
  // [IN]     en:Interpolation method of convert Bayer to RGB24  0-nearest
  // neighbour 1-bilinearity 2-Hamilton
  unsigned int iMethodValue;
  unsigned int nReserved[3];

} MV_SAVE_IMAGE_PARAM_EX;

// ch:?????????? | en:Pixel convert structure
typedef struct _MV_PIXEL_CONVERT_PARAM_T_ {
  unsigned short nWidth;  // [IN]     ch:???? | en:Width
  unsigned short nHeight; // [IN]     ch:???? | en:Height

  enum MvGvspPixelType
      enSrcPixelType; // [IN]     ch:??????? | en:Source pixel format
  unsigned char *pSrcData; // [IN]     ch:??????????? | en:Input data buffer
  unsigned int nSrcDataLen; // [IN]     ch:?????????�� | en:Input data size

  enum MvGvspPixelType
      enDstPixelType; // [IN]     ch:????????? | en:Destination pixel format
  unsigned char *pDstBuffer; // [OUT]    ch:?????????? | en:Output data buffer
  unsigned int nDstLen; // [OUT]    ch:????????�� | en:Output data size
  unsigned int nDstBufferSize; // [IN]     ch:???????????????�� | en:Provided
                               // outbut buffer size
  unsigned int nRes[4];
} MV_CC_PIXEL_CONVERT_PARAM;

// ch:????????? | en:Record Format Type
typedef enum _MV_RECORD_FORMAT_TYPE_ {
  MV_FormatType_Undefined = 0,
  MV_FormatType_AVI = 1,

} MV_RECORD_FORMAT_TYPE;

// ch:?????? | en:Record Parameters
typedef struct _MV_CC_RECORD_PARAM_T_ {
  enum MvGvspPixelType enPixelType; // [IN]     ???????????????

  unsigned short nWidth; // [IN]     ????(?????????????2?????)
  unsigned short nHeight; // [IN]     ????(?????????????2?????)

  float fFrameRate;      // [IN]     ???fps(1/16-120)
  unsigned int nBitRate; // [IN]     ????kbps(128kbps-16Mbps)

  MV_RECORD_FORMAT_TYPE enRecordFmtType; // [IN]     ?????

  char *strFilePath; // [IN]     ?????????��??(???��???��?????????????utf-8)

  unsigned int nRes[8];

} MV_CC_RECORD_PARAM;

// ch:??????? | en:Record Data
typedef struct _MV_CC_INPUT_FRAME_INFO_T_ {
  unsigned char *pData;  // [IN]     ??????????
  unsigned int nDataLen; // [IN]     ????��

  unsigned int nRes[8];

} MV_CC_INPUT_FRAME_INFO;

// ch:????? | en:Acquisition mode
typedef enum _MV_CAM_ACQUISITION_MODE_ {
  MV_ACQ_MODE_SINGLE = 0,     // ch:????? | en:Single Mode
  MV_ACQ_MODE_MUTLI = 1,      // ch:????? | en:Multi Mode
  MV_ACQ_MODE_CONTINUOUS = 2, // ch:????????? | en:Continuous Mode

} MV_CAM_ACQUISITION_MODE;

// ch:?????? | en:Gain Mode
typedef enum _MV_CAM_GAIN_MODE_ {
  MV_GAIN_MODE_OFF = 0,        // ch:??? | en:Single Mode
  MV_GAIN_MODE_ONCE = 1,       // ch:??? | en:Multi Mode
  MV_GAIN_MODE_CONTINUOUS = 2, // ch:???? | en:Continuous Mode

} MV_CAM_GAIN_MODE;

// ch:????? | en:Exposure Mode
typedef enum _MV_CAM_EXPOSURE_MODE_ {
  MV_EXPOSURE_MODE_TIMED = 0,         // Timed
  MV_EXPOSURE_MODE_TRIGGER_WIDTH = 1, // TriggerWidth
} MV_CAM_EXPOSURE_MODE;

// ch:???????? | en:Auto Exposure Mode
typedef enum _MV_CAM_EXPOSURE_AUTO_MODE_ {
  MV_EXPOSURE_AUTO_MODE_OFF = 0,        // ch:??? | en:Off
  MV_EXPOSURE_AUTO_MODE_ONCE = 1,       // ch:??? | en:Once
  MV_EXPOSURE_AUTO_MODE_CONTINUOUS = 2, // ch:???? | en:Continuous

} MV_CAM_EXPOSURE_AUTO_MODE;

typedef enum _MV_CAM_TRIGGER_MODE_ {
  MV_TRIGGER_MODE_OFF = 0, // ch:??? | en:Off
  MV_TRIGGER_MODE_ON = 1,  // ch:?? | en:ON

} MV_CAM_TRIGGER_MODE;

typedef enum _MV_CAM_GAMMA_SELECTOR_ {
  MV_GAMMA_SELECTOR_USER = 1,
  MV_GAMMA_SELECTOR_SRGB = 2,

} MV_CAM_GAMMA_SELECTOR;

typedef enum _MV_CAM_BALANCEWHITE_AUTO_ {
  MV_BALANCEWHITE_AUTO_OFF = 0,
  MV_BALANCEWHITE_AUTO_ONCE = 2,
  MV_BALANCEWHITE_AUTO_CONTINUOUS = 1, // ch:???? | en:Continuous

} MV_CAM_BALANCEWHITE_AUTO;

typedef enum _MV_CAM_TRIGGER_SOURCE_ {
  MV_TRIGGER_SOURCE_LINE0 = 0,
  MV_TRIGGER_SOURCE_LINE1 = 1,
  MV_TRIGGER_SOURCE_LINE2 = 2,
  MV_TRIGGER_SOURCE_LINE3 = 3,
  MV_TRIGGER_SOURCE_COUNTER0 = 4,

  MV_TRIGGER_SOURCE_SOFTWARE = 7,
  MV_TRIGGER_SOURCE_FrequencyConverter = 8,

} MV_CAM_TRIGGER_SOURCE;

typedef enum _MV_GIGE_TRANSMISSION_TYPE_ {
  MV_GIGE_TRANSTYPE_UNICAST = 0x0, // ch:???????(???) | en:Unicast mode
  MV_GIGE_TRANSTYPE_MULTICAST = 0x1, // ch:????�� | en:Multicast mode
  MV_GIGE_TRANSTYPE_LIMITEDBROADCAST =
      0x2, // ch:???????????????????? | en:Limited broadcast mode,not support
  MV_GIGE_TRANSTYPE_SUBNETBROADCAST =
      0x3, // ch:?????????????????? | en:Subnet broadcast mode,not support
  MV_GIGE_TRANSTYPE_CAMERADEFINED =
      0x4, // ch:??????????????????? | en:Transtype from camera,not support
  MV_GIGE_TRANSTYPE_UNICAST_DEFINED_PORT =
      0x5, // ch:?????????????????????????Port?? | en:User Defined Receive Data
           // Port
  MV_GIGE_TRANSTYPE_UNICAST_WITHOUT_RECV =
      0x00010000, // ch:?????????????????????????????????? | en:Unicast without
                  // receive data
  MV_GIGE_TRANSTYPE_MULTICAST_WITHOUT_RECV =
      0x00010001, // ch:????��???????????????????????? | en:Multicast without
                  // receive data
} MV_GIGE_TRANSMISSION_TYPE;

// GigEVision IP Configuration
#define MV_IP_CFG_STATIC 0x05000000
#define MV_IP_CFG_DHCP 0x06000000
#define MV_IP_CFG_LLA 0x04000000

// GigEVision Net Transfer Mode
#define MV_NET_TRANS_DRIVER 0x00000001
#define MV_NET_TRANS_SOCKET 0x00000002

// CameraLink Baud Rates (CLUINT32)
#define MV_CAML_BAUDRATE_9600 0x00000001
#define MV_CAML_BAUDRATE_19200 0x00000002
#define MV_CAML_BAUDRATE_38400 0x00000004
#define MV_CAML_BAUDRATE_57600 0x00000008
#define MV_CAML_BAUDRATE_115200 0x00000010
#define MV_CAML_BAUDRATE_230400 0x00000020
#define MV_CAML_BAUDRATE_460800 0x00000040
#define MV_CAML_BAUDRATE_921600 0x00000080
#define MV_CAML_BAUDRATE_AUTOMAX 0x40000000

// ch:??????? | en:Information Type
#define MV_MATCH_TYPE_NET_DETECT                                               \
  0x00000001 // ch:???????????????? | en:Network traffic and packet loss
             // information
#define MV_MATCH_TYPE_USB_DETECT                                               \
  0x00000002 // ch:host?????????U3V?��????????? | en:The total number of bytes
             // host received from U3V device

// ch:????????????????????? | en:The maximum number of child nodes corresponding
// to a node
#define MV_MAX_XML_NODE_NUM_C 128

// ch:???????????????? | en:The maximum length of node name string
#define MV_MAX_XML_NODE_STRLEN_C 64

// ch:???String????? | en:The maximum length of Node String
#define MV_MAX_XML_STRVALUE_STRLEN_C 64

// ch:?????????????? | en:The maximum length of the node description field
#define MV_MAX_XML_DISC_STRLEN_C 512

// ch:???????? | en:The maximum number of units
#define MV_MAX_XML_ENTRY_NUM 10

// ch:???????????? | en:The maximum number of parent nodes
#define MV_MAX_XML_PARENTS_NUM 8

// ch:???????????????????? | en:The length of the name of each unit that has
// been implemented
#define MV_MAX_XML_SYMBOLIC_STRLEN_C 64

#define MV_MAX_XML_SYMBOLIC_NUM 64

// ch:???????????????? | en:The default maximum number of retransmission packets

// ch:??????????????? | en:A fully matched information structure
typedef struct _MV_ALL_MATCH_INFO_ {
  unsigned int nType; // ch:????????????????e.g. MV_MATCH_TYPE_NET_DETECT |
                      // en:Information type need to output ,e.g.
                      // MV_MATCH_TYPE_NET_DETECT
  void *pInfo;        // ch:???????????��?????????? | en:Output information cache,
                      // which is allocated by the caller
  unsigned int nInfoSize; // ch:?????????�� | en:Information cache size

} MV_ALL_MATCH_INFO;

// ch:???????????????????????��???????? MV_MATCH_TYPE_NET_DETECT
// en:Network traffic and packet loss feedback structure, the corresponding type
// is MV_MATCH_TYPE_NET_DETECT
typedef struct _MV_MATCH_INFO_NET_DETECT_ {
  int64_t nReviceDataSize; // ch:??????????��
                           // [???StartGrabbing??StopGrabbing??????????] |
                           // en:Received data size
  int64_t nLostPacketCount; // ch:?????????? | en:Number of packets lost
  unsigned int nLostFrameCount; // ch:??????? | en:Number of frames lost
  unsigned int nNetRecvFrameCount;   // ch:???? | en:Reserved
  int64_t nRequestResendPacketCount; // ???????????
  int64_t nResendPacketCount;        // ???????
} MV_MATCH_INFO_NET_DETECT;

// ch:host?????u3v?��???????????????????? MV_MATCH_TYPE_USB_DETECT | en:The total
// number of bytes host received from the u3v device side, the corresponding
// type is MV_MATCH_TYPE_USB_DETECT
typedef struct _MV_MATCH_INFO_USB_DETECT_ {
  int64_t nReviceDataSize; // ch:??????????��
                           // [???OpenDevicce??CloseDevice??????????] |
                           // en:Received data size
  unsigned int
      nRevicedFrameCount; // ch:?????????? | en:Number of frames received
  unsigned int nErrorFrameCount; // ch:??????? | en:Number of error frames
  unsigned int nReserved[2]; // ch:???? | en:Reserved
} MV_MATCH_INFO_USB_DETECT;

typedef struct _MV_IMAGE_BASIC_INFO_ {
  // width
  unsigned short nWidthValue;
  unsigned short nWidthMin;
  unsigned int nWidthMax;
  unsigned int nWidthInc;

  // height
  unsigned int nHeightValue;
  unsigned int nHeightMin;
  unsigned int nHeightMax;
  unsigned int nHeightInc;

  // framerate
  float fFrameRateValue;
  float fFrameRateMin;
  float fFrameRateMax;

  // ch:?????? | en:pixel format
  unsigned int enPixelType; // ch:??????????? | en:Current pixel format
  unsigned int
      nSupportedPixelFmtNum; // ch:?????????????? | en:Support pixel format
  unsigned int enPixelList[MV_MAX_XML_SYMBOLIC_NUM];
  unsigned int nReserved[8];

} MV_IMAGE_BASIC_INFO;

// ch: ????????? | en:Exception message type
#define MV_EXCEPTION_DEV_DISCONNECT                                            \
  0x00008001 // ch:?��??????? | en:The device is disconnected
#define MV_EXCEPTION_VERSION_CHECK                                             \
  0x00008002 // ch:SDK???????��????? | en:SDK does not match the driver version

// ch:?��??????? | en:Device Access Mode
// ch:???????????APP???????CCP????? | en:Exclusive authority, other APP is only
// allowed to read the CCP register
#define MV_ACCESS_Exclusive 1
// ch:?????5?????????????????????? | en:You can seize the authority from the 5
// mode, and then open with exclusive authority
#define MV_ACCESS_ExclusiveWithSwitch 2
// ch:????????????APP?????????��???? | en:Control authority, allows other APP
// reading all registers
#define MV_ACCESS_Control 3
// ch:?????5????????????????????????? | en:You can seize the authority from the
// 5 mode, and then open with control authority
#define MV_ACCESS_ControlWithSwitch 4
// ch:???????????????? | en:Open with seized control
// authority
#define MV_ACCESS_ControlSwitchEnable 5
// ch:?????5???????????????????????????????? | en:You can seize the authority
// from the 5 mode, and then open with seized control authority
#define MV_ACCESS_ControlSwitchEnableWithKey 6
// ch:???????��???????????????? | en:Open with read mode and is available under
// control authority
#define MV_ACCESS_Monitor 7

/************************************************************************/
/* ?????GenICam??C?????????????                                     */
/* Package of GenICam C interface-related parameters definition         */
/************************************************************************/

// ch:???????????????? | en:Interface type corresponds to each node
enum MV_XML_InterfaceType {
  IFT_IValue,       //!> IValue interface
  IFT_IBase,        //!> IBase interface
  IFT_IInteger,     //!> IInteger interface
  IFT_IBoolean,     //!> IBoolean interface
  IFT_ICommand,     //!> ICommand interface
  IFT_IFloat,       //!> IFloat interface
  IFT_IString,      //!> IString interface
  IFT_IRegister,    //!> IRegister interface
  IFT_ICategory,    //!> ICategory interface
  IFT_IEnumeration, //!> IEnumeration interface
  IFT_IEnumEntry,   //!> IEnumEntry interface
  IFT_IPort,        //!> IPort interface
};

// ch:????????? | en:Node Access Mode
enum MV_XML_AccessMode {
  AM_NI,          //!< Not implemented
  AM_NA,          //!< Not available
  AM_WO,          //!< Write Only
  AM_RO,          //!< Read Only
  AM_RW,          //!< Read and Write
  AM_Undefined,   //!< Object is not yet initialized
  AM_CycleDetect, //!< used internally for AccessMode cycle detection

};

enum MV_XML_Visibility {
  V_Beginner = 0,  //!< Always visible
  V_Expert = 1,    //!< Visible for experts or Gurus
  V_Guru = 2,      //!< Visible for Gurus
  V_Invisible = 3, //!< Not Visible
  V_Undefined = 99 //!< Object is not yet initialized
};

// Event????????? | en:Event callback infomation
#define MAX_EVENT_NAME_SIZE                                                    \
  128 //???Event??????????? | en:Max length of event name

typedef struct _MV_EVENT_OUT_INFO_ {
  char EventName[MAX_EVENT_NAME_SIZE]; // Event???? | en:Event name

  unsigned short nEventID;       // Event?? | en:Event ID
  unsigned short nStreamChannel; //???????? | en:Circulation number

  unsigned int nBlockIdHigh; //????�� | en:BlockId high
  unsigned int nBlockIdLow;  //????�� | en:BlockId low

  unsigned int nTimestampHigh; //??????�� | en:Timestramp high
  unsigned int nTimestampLow;  //??????�� | en:Timestramp low

  void *pEventData;            // Event???? | en:Event data
  unsigned int nEventDataSize; // Event??????? | en:Event data len

  unsigned int nReserved[16]; //??? | en:Reserved
} MV_EVENT_OUT_INFO;

// ch:?????? | en:File Access
typedef struct _MV_CC_FILE_ACCESS_T {
  const char *pUserFileName; //???????? | en:User file name
  const char *pDevFileName;  //?��????? | en:Device file name

  unsigned int nReserved[32]; //??? | en:Reserved
} MV_CC_FILE_ACCESS;

// ch:?????????? | en:File Access Progress
typedef struct _MV_CC_FILE_ACCESS_PROGRESS_T {
  int64_t nCompleted; //????????? | en:Completed Length
  int64_t nTotal;     //????? | en:Total Length

  unsigned int nReserved[8]; //??? | en:Reserved
} MV_CC_FILE_ACCESS_PROGRESS;

// ch:??????????????????????��???? | en:Transmission type
typedef struct _MV_TRANSMISSION_TYPE_T {
  MV_GIGE_TRANSMISSION_TYPE enTransmissionType; //?????? | en:Transmission type
  unsigned int nDestIp; //???IP???��?????????? | en:Destination IP
  unsigned short nDestPort; //???Port???��?????????? | en:Destination port

  unsigned int nReserved[32]; //??? | en:Reserved
} MV_TRANSMISSION_TYPE;

// ch:??????????? | en:Action Command
typedef struct _MV_ACTION_CMD_INFO_T {
  unsigned int nDeviceKey; //?��???
  unsigned int nGroupKey;  //???
  unsigned int nGroupMask; //??????

  unsigned int bActionTimeEnable; //??????��?1?Action Time????��????1???��
  int64_t nActionTime; //??????????????��?

  const char *pBroadcastAddress; //???????
  unsigned int nTimeOut; //???ACK???????????0????????ACK

  unsigned int nReserved[16]; //??? | en:Reserved

} MV_ACTION_CMD_INFO;

// ch:????????????? | en:Action Command Result
typedef struct _MV_ACTION_CMD_RESULT_T {
  unsigned char strDeviceAddress[12 + 3 + 1]; // IP address of the device

  // status code returned by the device
  int nStatus; // 1.0x0000:success.
  // 2.0x8001:Command is not supported by the device.
  // 3.0x8013:The device is not synchronized to a master clock to be used as
  // time reference.
  // 4.0x8015:A device queue or packet data has overflowed.
  // 5.0x8016:The requested scheduled action command was requested at a time
  // that is already past.

  unsigned int nReserved[4]; //??? | en:Reserved

} MV_ACTION_CMD_RESULT;

// ch:??????????????��? | en:Action Command Result List
typedef struct _MV_ACTION_CMD_RESULT_LIST_T {
  unsigned int nNumResults; //?????????
  MV_ACTION_CMD_RESULT *pResults;

} MV_ACTION_CMD_RESULT_LIST;

// ch:?????????????? | en:Single Node Basic Attributes
typedef struct _MV_XML_NODE_FEATURE_ {
  enum MV_XML_InterfaceType enType;    // ch:??????? | en:Node Type
  enum MV_XML_Visibility enVisivility; // ch:????? | en:Is visibility
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????     ???????? |
                                                 // en:Node Description, NOT
                                                 // SUPPORT NOW
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C]; // ch:??????? | en:Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];    // ch:????? | en:Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C]; // ch:??? | en:Notice

  unsigned int nReserved[4];
} MV_XML_NODE_FEATURE;

// ch:????��? | en:Node List
typedef struct _MV_XML_NODES_LIST_ {
  unsigned int nNodeNum; // ch:?????? | en:Node Number
  MV_XML_NODE_FEATURE stNodes[MV_MAX_XML_NODE_NUM_C];
} MV_XML_NODES_LIST;

typedef struct _MV_XML_FEATURE_Value_ {
  enum MV_XML_InterfaceType enType; // ch:??????? | en:Node Type
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????     ???????? |
                                                 // en:Node Description, NOT
                                                 // SUPPORT NOW
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C]; // ch:??????? | en:Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];    // ch:????? | en:Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C]; // ch:??? | en:Notice
  unsigned int nReserved[4];
} MV_XML_FEATURE_Value;

typedef struct _MV_XML_FEATURE_Base_ {
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
} MV_XML_FEATURE_Base;

typedef struct _MV_XML_FEATURE_Integer_ {
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????? | en:NOT SUPPORT
                                                 // NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility; //????? | en:Visible
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW
  int64_t nValue;                      // ch:???? | en:Current Value
  int64_t nMinValue;                   // ch:??��? | en:Min Value
  int64_t nMaxValue;                   // ch:???? | en:Max Value
  int64_t nIncrement;                  // ch:???? | en:Increment

  unsigned int nReserved[4];

} MV_XML_FEATURE_Integer;

typedef struct _MV_XML_FEATURE_Boolean_ {
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????? | en:NOT SUPPORT
                                                 // NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW
  bool bValue;                         // ch:???? | en:Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_Boolean;

typedef struct _MV_XML_FEATURE_Command_ {
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????? | en:NOT SUPPORT
                                                 // NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW

  unsigned int nReserved[4];
} MV_XML_FEATURE_Command;

typedef struct _MV_XML_FEATURE_Float_ {
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????? | en:NOT SUPPORT
                                                 // NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility; //????? | en:Visible
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW
  double dfValue;                      // ch:???? | en:Current Value
  double dfMinValue;                   // ch:??��? | en:Min Value
  double dfMaxValue;                   // ch:???? | en:Max Value
  double dfIncrement;                  // ch:???? | en:Increment

  unsigned int nReserved[4];
} MV_XML_FEATURE_Float;

typedef struct _MV_XML_FEATURE_String_ {
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????? | en:NOT SUPPORT
                                                 // NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW
  char strValue[MV_MAX_XML_STRVALUE_STRLEN_C]; // ch:???? | en:Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_String;

typedef struct _MV_XML_FEATURE_Register_ {
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????? | en:NOT SUPPORT
                                                 // NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];

  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW
  int64_t nAddrValue;                  // ch:???? | en:Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_Register;

typedef struct _MV_XML_FEATURE_Category_ {
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:??????? ???????? | en:Node
                                                 // Description, NOT SUPPORT NOW
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C]; // ch:??????? | en:Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];    // ch:????? | en:Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C]; // ch:??? | en:Notice

  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible

  unsigned int nReserved[4];
} MV_XML_FEATURE_Category;

typedef struct _MV_XML_FEATURE_EnumEntry_ {
  char strName[MV_MAX_XML_NODE_STRLEN_C];
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:???????? | en:NOT SUPPORT
                                                 // NOW
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C];
  int bIsImplemented;
  int nParentsNum;
  MV_XML_NODE_FEATURE stParentsList[MV_MAX_XML_PARENTS_NUM];

  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible
  int64_t nValue;                      // ch:???? | en:Current Value
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW
  int nReserved[8];

} MV_XML_FEATURE_EnumEntry;

typedef struct _MV_XML_FEATURE_Enumeration_ {
  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:??????? ???????? | en:Node
                                                 // Description, NOT SUPPORT NOW
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C]; // ch:??????? | en:Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];    // ch:????? | en:Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C]; // ch:??? | en:Notice

  int nSymbolicNum; // ch:Symbolic?? | en:Symbolic Number
  char strCurrentSymbolic[MV_MAX_XML_SYMBOLIC_STRLEN_C]; // ch:???Symbolic???? |
                                                         // en:Current Symbolic
                                                         // Index
  char strSymbolic[MV_MAX_XML_SYMBOLIC_NUM][MV_MAX_XML_SYMBOLIC_STRLEN_C];
  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW
  int64_t nValue;                      // ch:???? | en:Current Value

  unsigned int nReserved[4];
} MV_XML_FEATURE_Enumeration;

typedef struct _MV_XML_FEATURE_Port_ {
  enum MV_XML_Visibility enVisivility; // ch:????? | en:Visible
  char strDescription[MV_MAX_XML_DISC_STRLEN_C]; // ch:??????? ???????? | en:Node
                                                 // Description, NOT SUPPORT NOW
  char strDisplayName[MV_MAX_XML_NODE_STRLEN_C]; // ch:??????? | en:Display Name
  char strName[MV_MAX_XML_NODE_STRLEN_C];    // ch:????? | en:Node Name
  char strToolTip[MV_MAX_XML_DISC_STRLEN_C]; // ch:??? | en:Notice

  enum MV_XML_AccessMode enAccessMode; // ch:?????? | en:Access Mode
  int bIsLocked;                       // ch:?????????0-??1-??    ???????? | en:Locked. 0-NO; 1-YES,
                                       // NOT SUPPORT NOW

  unsigned int nReserved[4];
} MV_XML_FEATURE_Port;

typedef struct _MV_XML_CAMERA_FEATURE_ {
  enum MV_XML_InterfaceType enType;
  union {
    MV_XML_FEATURE_Integer stIntegerFeature;
    MV_XML_FEATURE_Float stFloatFeature;
    MV_XML_FEATURE_Enumeration stEnumerationFeature;
    MV_XML_FEATURE_String stStringFeature;
  } SpecialFeature;

} MV_XML_CAMERA_FEATURE;

typedef struct _MVCC_ENUMVALUE_T {
  unsigned int nCurValue; // ch:???? | en:Current Value
  unsigned int nSupportedNum; // ch:???????��??????? | en:Number of valid data
  unsigned int nSupportValue[MV_MAX_XML_SYMBOLIC_NUM];

  unsigned int nReserved[4];
} MVCC_ENUMVALUE;

typedef struct _MVCC_INTVALUE_T {
  unsigned int nCurValue; // ch:???? | en:Current Value
  unsigned int nMax;
  unsigned int nMin;
  unsigned int nInc;

  unsigned int nReserved[4];
} MVCC_INTVALUE;

typedef struct _MVCC_INTVALUE_EX_T {
  int64_t nCurValue; // ch:???? | en:Current Value
  int64_t nMax;
  int64_t nMin;
  int64_t nInc;

  unsigned int nReserved[16];
} MVCC_INTVALUE_EX;

typedef struct _MVCC_FLOATVALUE_T {
  float fCurValue; // ch:???? | en:Current Value
  float fMax;
  float fMin;

  unsigned int nReserved[4];
} MVCC_FLOATVALUE;

typedef struct _MVCC_STRINGVALUE_T {
  char chCurValue[256]; // ch:???? | en:Current Value

  int64_t nMaxLength;
  unsigned int nReserved[2];
} MVCC_STRINGVALUE;

#endif /* _MV_CAMERA_PARAMS_H_ */
