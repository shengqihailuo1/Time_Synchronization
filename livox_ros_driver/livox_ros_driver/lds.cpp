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

#include "lds.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <chrono>
struct time_stamp {
  int64_t high;
  int64_t low;
};
extern struct time_stamp *pointt;
namespace livox_ros {

/** Common function --------------------------------------------------------- */
bool IsFilePathValid(const char *path_str) {
  int str_len = strlen(path_str);

  if ((str_len > kPathStrMinSize) && (str_len < kPathStrMaxSize)) {
    return true;
  } else {
    return false;
  }
}

/** Replace nonstardard function "timegm" with mktime.
 *  For a portable version of timegm, set the TZ environment variable to UTC,
 *  call mktime and restore the value of TZ.
 *  "localtime" and "timegm" are nonstandard GNU extensions that are also present
 *  on the BSDs. Avoid their use!!!
 */
time_t replace_timegm(struct tm *tm) {
  time_t ret;
  char *tz;

  tz = getenv("TZ");
  setenv("TZ", "", 1);
  tzset();
  ret = mktime(tm);

  if (tz)
    setenv("TZ", tz, 1);
  else
    unsetenv("TZ");
  tzset();

  return ret;
}

uint64_t RawLdsStampToNs(LdsStamp &timestamp, uint8_t timestamp_type) {
  if (timestamp_type == kTimestampTypePps) {
    return timestamp.stamp;
  } else if (timestamp_type == kTimestampTypeNoSync) {
    return timestamp.stamp;
  } else if (timestamp_type == kTimestampTypePtp) {
    return timestamp.stamp;
  } else if (timestamp_type == kTimestampTypePpsGps) {
    struct tm time_utc;
    time_utc.tm_isdst = 0;
    time_utc.tm_year = timestamp.stamp_bytes[0] + 100;  // map 2000 to 1990
    time_utc.tm_mon = timestamp.stamp_bytes[1] - 1;     // map 1~12 to 0~11
    time_utc.tm_mday = timestamp.stamp_bytes[2];
    time_utc.tm_hour = timestamp.stamp_bytes[3];
    time_utc.tm_min = 0;
    time_utc.tm_sec = 0;

    // uint64_t time_epoch = mktime(&time_utc);
    uint64_t time_epoch = timegm(&time_utc);  // no timezone
    time_epoch = time_epoch * 1000000 + timestamp.stamp_word.high;  // to us
    time_epoch = time_epoch * 1000;                                 // to ns

    return time_epoch;
  } else {
    printf("Timestamp type[%d] invalid.\n", timestamp_type);
    return 0;
  }
}
#include <chrono>
#include <fcntl.h>

#include <sys/mman.h>

//通过存储包中的原始数据获取时间戳，并进行处理返回。
uint64_t GetStoragePacketTimestamp(StoragePacket *packet, uint8_t data_src) {
  LivoxEthPacket *raw_packet =
      reinterpret_cast<LivoxEthPacket *>(packet->raw_data);
  LdsStamp timestamp;
  memcpy(timestamp.stamp_bytes, raw_packet->timestamp, sizeof(timestamp));

  static uint16_t count = 0;
  count ++;
  if(count % 200 == 0 ){
    count = 0;
    printf(" : %d ", raw_packet->timestamp_type);//0,kTimestampTypeNoSync /**< No sync signal mode.
  }

  //根据原始数据中的时间戳类型进行不同的处理：
  // typedef enum {
  //   kTimestampTypeNoSync = 0, /**< 无同步信号模式. */
  //   kTimestampTypePtp = 1,    /**< 1588v2.0 PTP 同步模式. */
  //   kTimestampTypeRsvd = 2,   /**< 保留使用. */
  //   kTimestampTypePpsGps = 3, /**< pps+gps 同步模式. */
  //   kTimestampTypePps = 4,    /**< 仅 pps 同步模式. */
  //   kTimestampTypeUnknown = 5 /**< 未知模式. */
  // } TimestampType;

  //如果是 PPS 类型，则将时间戳和接收时间相加；
  if (raw_packet->timestamp_type == kTimestampTypePps) {
    
    // double a1 = timestamp.stamp / 1e9;
    // double a2 = packet->time_rcv / 1e9;

    // printf("kTimestampTypePps: %.6f\n", a1 + a2);
    if (data_src != kSourceLvxFile) {
      return (timestamp.stamp + packet->time_rcv);//运行的是这里。
    } else {
      return timestamp.stamp;
    }
  } else if (raw_packet->timestamp_type == kTimestampTypeNoSync) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePtp) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePpsGps) {
    //如果是 PPS-GPS 类型，则需要将时间戳转换为 UTC 时间，再和高位时间戳相加，最后返回以纳秒为单位的时间戳。
    
    struct tm time_utc;
    time_utc.tm_isdst = 0;
    //将时间戳中的年份映射到1990年之后（例如2000年映射为1990年）。
    time_utc.tm_year = raw_packet->timestamp[0] + 100;  // map 2000 to 1990
    
    time_utc.tm_mon = raw_packet->timestamp[1] - 1;     // map 1~12 to 0~11 //将时间戳中的月份映射到0~11。
    time_utc.tm_mday = raw_packet->timestamp[2];
    time_utc.tm_hour = raw_packet->timestamp[3];
    // time_utc.tm_mon = 0;  // map 1~12 to 0~11
    // time_utc.tm_mday =1;
    // time_utc.tm_hour = 0;

    // time_utc.tm_min = raw_packet->timestamp[4];
    // time_utc.tm_sec = raw_packet->timestamp[5];
    time_utc.tm_min = 0;
    time_utc.tm_sec = 0;

    //使用 timegm 函数将转换后的 UTC 时间转换为自1970年1月1日以来的秒数。
    // uint64_t time_epoch = mktime(&time_utc);
    uint64_t time_epoch = timegm(&time_utc);  // no timezone 没有显式地指定任何特定的时区信息，而是默认使用本地时区来计算时间戳。
    //printf("111 time_epoch: %.9lu\n", time_epoch);//都是1520812800，
    //printf("timestamp.stamp_word.high : %u\n", timestamp.stamp_word.high);//44600174微妙

    //将秒数转换为微秒，并与高位时间戳相加，得到以微秒为单位的时间戳。
    //注意：这里的高位时间是livox内置计时器计算的毫秒数量
    time_epoch = time_epoch * 1000000 + timestamp.stamp_word.high; 
    time_epoch = time_epoch * 1000;//转化为纳秒。



    // static uint64_t last_timestamp_stamp_word_high = timestamp.stamp_word.high;
    //  if(timestamp.stamp_word.high -last_timestamp_stamp_word_high >= 1000000){
    //   last_timestamp_stamp_word_high = timestamp.stamp_word.high;
    //  printf("！！！！！！！！！！！！livox内部计时已经达到1秒了！！！！！！！！！！！！！！\n");
    //  }




    // struct tm
    // {
    //   int tm_sec;			/* Seconds.	[0-60] (1 leap second) */
    //   int tm_min;			/* Minutes.	[0-59] */
    //   int tm_hour;			/* Hours.	[0-23] */
    //   int tm_mday;			/* Day.		[1-31] */
    //   int tm_mon;			/* Month.	[0-11] */
    //   int tm_year;			/* Year	- 1900.  */
    //   int tm_wday;			/* Day of week.	[0-6] */
    //   int tm_yday;			/* Days in year.[0-365]	*/
    //   int tm_isdst;			/* DST.		[-1/0/1]*/
    // }
    
    //printf(" raw_packet->timestamp:  %d   %d   %d   %d \n",raw_packet->timestamp[0],raw_packet->timestamp[1],raw_packet->timestamp[2],raw_packet->timestamp[3]);
    //上面打印：18   3   12   0，从gprmc里面读取出来的年月日时 。等等把32的gprmc“时”设置为10，看看这里最后是否打印出10。


    //printf("time_utc   %d   %d   %d   %d   %d   %d  \n",time_utc.tm_year,time_utc.tm_mon,time_utc.tm_mday,time_utc.tm_hour,time_utc.tm_min,time_utc.tm_sec);
    //上面打印   118(2018-1900)   2（3月）   12（12日）   0（时）   0（分）   0（秒） 

    //printf("222 time_epoch: %.9lu\n", time_epoch);
    //上面打印1520812826797249000纳秒

    //printf("---------------------------\n");
    return time_epoch;
  } else {
    printf("Timestamp type[%d] invalid.\n", raw_packet->timestamp_type);
    return 0;
  }
}

uint32_t CalculatePacketQueueSize(uint32_t interval_ms, uint8_t product_type,
                                  uint8_t data_type) {
  uint32_t queue_size =
      (interval_ms * GetPacketNumPerSec(product_type, data_type)) / 1000;

  queue_size = queue_size * 2;
  if (queue_size < kMinEthPacketQueueSize) {
    queue_size = kMinEthPacketQueueSize;
  } else if (queue_size > kMaxEthPacketQueueSize) {
    queue_size = kMaxEthPacketQueueSize;
  }

  return queue_size;
}

void ParseCommandlineInputBdCode(const char *cammandline_str,
                                 std::vector<std::string> &bd_code_list) {
  char *strs = new char[strlen(cammandline_str) + 1];
  strcpy(strs, cammandline_str);

  std::string pattern = "&";
  char *bd_str = strtok(strs, pattern.c_str());
  std::string invalid_bd = "000000000";
  while (bd_str != NULL) {
    printf("Commandline input bd:%s\n", bd_str);
    if ((kBdCodeSize == strlen(bd_str)) &&
        (NULL == strstr(bd_str, invalid_bd.c_str()))) {
      bd_code_list.push_back(bd_str);
    } else {
      printf("Invalid bd:%s!\n", bd_str);
    }
    bd_str = strtok(NULL, pattern.c_str());
  }

  delete[] strs;
}

void EulerAnglesToRotationMatrix(EulerAngle euler, RotationMatrix matrix) {
  double cos_roll = cos(static_cast<double>(euler[0]));
  double cos_pitch = cos(static_cast<double>(euler[1]));
  double cos_yaw = cos(static_cast<double>(euler[2]));
  double sin_roll = sin(static_cast<double>(euler[0]));
  double sin_pitch = sin(static_cast<double>(euler[1]));
  double sin_yaw = sin(static_cast<double>(euler[2]));

  matrix[0][0] = cos_pitch * cos_yaw;
  matrix[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  matrix[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  matrix[1][0] = cos_pitch * sin_yaw;
  matrix[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  matrix[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  matrix[2][0] = -sin_pitch;
  matrix[2][1] = sin_roll * cos_pitch;
  matrix[2][2] = cos_roll * cos_pitch;

  /*
    float rotate[3][3] = {
    {
      std::cos(info.pitch) * std::cos(info.yaw),
      std::sin(info.roll) * std::sin(info.pitch) * std::cos(info.yaw) -
    std::cos(info.roll) * std::sin(info.yaw), std::cos(info.roll) *
    std::sin(info.pitch) * std::cos(info.yaw) + std::sin(info.roll) *
    std::sin(info.yaw) },
    {
      std::cos(info.pitch) * std::sin(info.yaw),
      std::sin(info.roll) * std::sin(info.pitch) * std::sin(info.yaw) +
    std::cos(info.roll) * std::cos(info.yaw), std::cos(info.roll) *
    std::sin(info.pitch) * std::sin(info.yaw) - std::sin(info.roll) *
    std::cos(info.yaw) },
    {
      -std::sin(info.pitch),
      std::sin(info.roll) * std::cos(info.pitch),
      std::cos(info.roll) * std::cos(info.pitch)
    }
    };
  */
}

void PointExtrisincCompensation(PointXyz *dst_point, const PointXyz &src_point,
                                ExtrinsicParameter &extrinsic) {
  dst_point->x = src_point.x * extrinsic.rotation[0][0] +
                 src_point.y * extrinsic.rotation[0][1] +
                 src_point.z * extrinsic.rotation[0][2] + extrinsic.trans[0];
  dst_point->y = src_point.x * extrinsic.rotation[1][0] +
                 src_point.y * extrinsic.rotation[1][1] +
                 src_point.z * extrinsic.rotation[1][2] + extrinsic.trans[1];
  dst_point->z = src_point.x * extrinsic.rotation[2][0] +
                 src_point.y * extrinsic.rotation[2][1] +
                 src_point.z * extrinsic.rotation[2][2] + extrinsic.trans[2];
}

/** Livox point procees for different raw data format
 * --------------------------------------------*/
uint8_t *LivoxPointToPxyzrtl(uint8_t *point_buf, LivoxEthPacket *eth_packet,
    ExtrinsicParameter &extrinsic, uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxPoint *raw_point = reinterpret_cast<LivoxPoint *>(eth_packet->data);

  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, raw_point);
    if (extrinsic.enable && IsTripleFloatNoneZero(raw_point->x,
        raw_point->y, raw_point->z)) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = 0;
    dst_point->line = 0;
    ++raw_point;
    ++dst_point;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxRawPointToPxyzrtl(uint8_t *point_buf, LivoxEthPacket *eth_packet,
    ExtrinsicParameter &extrinsic, uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxRawPoint *raw_point =
      reinterpret_cast<LivoxRawPoint *>(eth_packet->data);

  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, raw_point);
    if (extrinsic.enable && IsTripleIntNoneZero(raw_point->x,
        raw_point->y, raw_point->z)) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = 0;
    dst_point->line = 0;
    ++raw_point;
    ++dst_point;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxSpherPointToPxyzrtl(uint8_t *point_buf, \
    LivoxEthPacket *eth_packet, ExtrinsicParameter &extrinsic, \
    uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxSpherPoint *raw_point =
      reinterpret_cast<LivoxSpherPoint *>(eth_packet->data);

  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, raw_point);
    if (extrinsic.enable && raw_point->depth) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = 0;
    dst_point->line = 0;
    ++raw_point;
    ++dst_point;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxExtendRawPointToPxyzrtl(uint8_t *point_buf, \
    LivoxEthPacket *eth_packet, ExtrinsicParameter &extrinsic, \
    uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendRawPoint *raw_point =
      reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, (LivoxRawPoint *)raw_point);
    if (extrinsic.enable && IsTripleIntNoneZero(raw_point->x,
        raw_point->y, raw_point->z)) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxExtendSpherPointToPxyzrtl(uint8_t *point_buf, \
    LivoxEthPacket *eth_packet, ExtrinsicParameter &extrinsic, \
    uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendSpherPoint *raw_point =
      reinterpret_cast<LivoxExtendSpherPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, (LivoxSpherPoint *)raw_point);
    if (extrinsic.enable && raw_point->depth) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxDualExtendRawPointToPxyzrtl(uint8_t *point_buf, \
    LivoxEthPacket *eth_packet, ExtrinsicParameter &extrinsic, \
    uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendRawPoint *raw_point =
      reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data);

  /* LivoxDualExtendRawPoint = 2*LivoxExtendRawPoint */
  points_per_packet = points_per_packet * 2;
  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, (LivoxRawPoint *)raw_point);
    if (extrinsic.enable && IsTripleIntNoneZero(raw_point->x,
        raw_point->y, raw_point->z)) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag;
    if (line_num > 1) {
      dst_point->line = (line_id / 2) % line_num;
    } else {
      dst_point->line = 0;
    }
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxDualExtendSpherPointToPxyzrtl(uint8_t *point_buf, \
    LivoxEthPacket *eth_packet, ExtrinsicParameter &extrinsic, \
    uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxDualExtendSpherPoint *raw_point =
      reinterpret_cast<LivoxDualExtendSpherPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point,
                    (LivoxPointXyzr *)(dst_point + 1),
                    (LivoxDualExtendSpherPoint *)raw_point);
    if (extrinsic.enable && raw_point->depth1) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag1;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++dst_point;

    if (extrinsic.enable && raw_point->depth2) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag2;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++dst_point;

    ++raw_point; /* only increase one */
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxTripleExtendRawPointToPxyzrtl(uint8_t *point_buf, \
    LivoxEthPacket *eth_packet, ExtrinsicParameter &extrinsic, \
    uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendRawPoint *raw_point =
      reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data);

  /* LivoxTripleExtendRawPoint = 3*LivoxExtendRawPoint, echo_num */
  points_per_packet = points_per_packet * 3;
  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, (LivoxRawPoint *)raw_point);
    if (extrinsic.enable && IsTripleIntNoneZero(raw_point->x,
        raw_point->y, raw_point->z)) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag;
    if (line_num > 1) {
      dst_point->line = (line_id / 3) % line_num;
    } else {
      dst_point->line = 0;
    }
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

static uint8_t *LivoxTripleExtendSpherPointToPxyzrtl(uint8_t *point_buf, \
    LivoxEthPacket *eth_packet, ExtrinsicParameter &extrinsic, \
    uint32_t line_num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxTripleExtendSpherPoint *raw_point =
      reinterpret_cast<LivoxTripleExtendSpherPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point,
                    (LivoxPointXyzr *)(dst_point + 1),
                    (LivoxPointXyzr *)(dst_point + 2),
                    (LivoxTripleExtendSpherPoint *)raw_point);
    if (extrinsic.enable && raw_point->depth1) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag1;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++dst_point;

    if (extrinsic.enable && raw_point->depth2) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag2;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++dst_point;

    if (extrinsic.enable && raw_point->depth3) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag3;
    if (line_num > 1) {
      dst_point->line = line_id % line_num;
    } else {
      dst_point->line = 0;
    }
    ++dst_point;

    ++raw_point; /* only increase one */
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxImuDataProcess(uint8_t *point_buf, LivoxEthPacket *eth_packet) {
  memcpy(point_buf, eth_packet->data, sizeof(LivoxImuPoint));
  return point_buf;
}

const PointConvertHandler to_pxyzi_handler_table[kMaxPointDataType] = {
    LivoxRawPointToPxyzrtl,
    LivoxSpherPointToPxyzrtl,
    LivoxExtendRawPointToPxyzrtl,
    LivoxExtendSpherPointToPxyzrtl,
    LivoxDualExtendRawPointToPxyzrtl,
    LivoxDualExtendSpherPointToPxyzrtl,
    nullptr,
    LivoxTripleExtendRawPointToPxyzrtl,
    LivoxTripleExtendSpherPointToPxyzrtl
    };

PointConvertHandler GetConvertHandler(uint8_t data_type) {
  if (data_type < kMaxPointDataType)
    return to_pxyzi_handler_table[data_type];
  else
    return nullptr;
}

// 将存储数据包（StoragePacket）中的点云数据清零。
void ZeroPointDataOfStoragePacket(StoragePacket* storage_packet) {
  LivoxEthPacket *raw_packet = reinterpret_cast<LivoxEthPacket *>(storage_packet->raw_data);
  uint32_t point_length = GetPointLen(raw_packet->data_type);
  memset(raw_packet->data, 0, point_length * storage_packet->point_num);
}

#if 0

static void PointCloudConvert(LivoxPoint *p_dpoint, LivoxRawPoint *p_raw_point) {
  p_dpoint->x = p_raw_point->x/1000.0f;
  p_dpoint->y = p_raw_point->y/1000.0f;
  p_dpoint->z = p_raw_point->z/1000.0f;
  p_dpoint->reflectivity = p_raw_point->reflectivity;
}

#endif

/* Member function --------------------------------------------------------- */
Lds::Lds(uint32_t buffer_time_ms, uint8_t data_src) : \
    lidar_count_(kMaxSourceLidar), semaphore_(0), \
    buffer_time_ms_(buffer_time_ms), data_src_(data_src), request_exit_(false) {
    ResetLds(data_src_);
};

Lds::~Lds() {
  lidar_count_ = 0;
  ResetLds(0);
};

void Lds::ResetLidar(LidarDevice *lidar, uint8_t data_src) {
  DeInitQueue(&lidar->data);
  DeInitQueue(&lidar->imu_data);

  memset(lidar, 0, sizeof(LidarDevice));

  /** unallocated state */
  lidar->handle = kMaxSourceLidar;
  lidar->data_src = data_src;
  lidar->data_is_pubulished = false;
  lidar->connect_state = kConnectStateOff;
  lidar->raw_data_type = 0xFF;
}

void Lds::SetLidarDataSrc(LidarDevice *lidar, uint8_t data_src) {
  lidar->data_src = data_src;
}

void Lds::ResetLds(uint8_t data_src) {
  lidar_count_ = kMaxSourceLidar;
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    ResetLidar(&lidars_[i], data_src);
  }
}

void Lds::RequestExit() {
  request_exit_ = true;
}

bool Lds::IsAllQueueEmpty() {
  for (int i = 0; i < lidar_count_; i++) {
    if (!QueueIsEmpty(&lidars_[i].data)) {
      return false;
    }
  }

  return true;
}

bool Lds::IsAllQueueReadStop() {
  for (int i = 0; i < lidar_count_; i++) {
    uint32_t data_size = QueueUsedSize(&lidars_[i].data);
    if (data_size && (data_size > lidars_[i].onetime_publish_packets)) {
      return false;
    }
  }

  return true;
}

uint8_t Lds::GetDeviceType(uint8_t handle) {
  if (handle < kMaxSourceLidar) {
    return lidars_[handle].info.type;
  } else {
    return kDeviceTypeHub;
  }
}

void Lds::UpdateLidarInfoByEthPacket(LidarDevice *p_lidar,
    LivoxEthPacket* eth_packet) {
  if (p_lidar->raw_data_type != eth_packet->data_type) {
    p_lidar->raw_data_type = eth_packet->data_type;
    p_lidar->packet_interval = GetPacketInterval(p_lidar->info.type,
        eth_packet->data_type);
    p_lidar->timestamp_type = eth_packet->timestamp_type;
    p_lidar->packet_interval_max = p_lidar->packet_interval * 1.8f;
    p_lidar->onetime_publish_packets =
        GetPacketNumPerSec(p_lidar->info.type,
        p_lidar->raw_data_type) * buffer_time_ms_ / 1000;
    printf("Lidar[%d][%s] DataType[%d] timestamp_type[%d] PacketInterval[%d] "
        "PublishPackets[%d]\n", p_lidar->handle, p_lidar->info.broadcast_code,
        p_lidar->raw_data_type, p_lidar->timestamp_type,
        p_lidar->packet_interval, p_lidar->onetime_publish_packets);
  }
}

void Lds::StorageRawPacket(uint8_t handle, LivoxEthPacket* eth_packet) {
  LidarDevice *p_lidar = &lidars_[handle];
  LidarPacketStatistic *packet_statistic = &p_lidar->statistic_info;
  LdsStamp cur_timestamp;
  uint64_t timestamp;

  memcpy(cur_timestamp.stamp_bytes, eth_packet->timestamp,
         sizeof(cur_timestamp));
  timestamp = RawLdsStampToNs(cur_timestamp, eth_packet->timestamp_type);
  if (timestamp >= kRosTimeMax) {
    printf("Raw EthPacket time out of range Lidar[%d]\n", handle);
    return;
  }

  if (kImu != eth_packet->data_type) {
    UpdateLidarInfoByEthPacket(p_lidar, eth_packet);
    if (eth_packet->timestamp_type == kTimestampTypePps) {
      /** Whether a new sync frame */
      //下面的if判断条件就体现了pps模式的思想：一秒脉冲到达后，就调整时间。
      if ((cur_timestamp.stamp < packet_statistic->last_timestamp) &&
          (cur_timestamp.stamp < kPacketTimeGap)) {
        auto cur_time = std::chrono::high_resolution_clock::now();
        int64_t sync_time = cur_time.time_since_epoch().count();
        /** used receive time as timebase 使用接收时间作为时基  */
        packet_statistic->timebase = sync_time;//原来的时间  


        //在livox驱动里，将packet的timebase改为基准时间base_time。。。。发现：根本没有运行这里的代码。。。
        //packet_statistic->timebase = 123456;


      }
    }
    packet_statistic->last_timestamp = cur_timestamp.stamp;

    LidarDataQueue *p_queue = &p_lidar->data;
    if (nullptr == p_queue->storage_packet) {
      uint32_t queue_size = CalculatePacketQueueSize(
          buffer_time_ms_, p_lidar->info.type, eth_packet->data_type);
      InitQueue(p_queue, queue_size);
      printf("Lidar[%d][%s] storage queue size : %d %d\n", p_lidar->handle,
             p_lidar->info.broadcast_code, queue_size, p_queue->size);
    }
    if (!QueueIsFull(p_queue)) {
      QueuePushAny(p_queue, (uint8_t *)eth_packet,
          GetEthPacketLen(eth_packet->data_type),
          packet_statistic->timebase,
          GetPointsPerPacket(eth_packet->data_type));
      if (QueueUsedSize(p_queue) > p_lidar->onetime_publish_packets) {
        if (semaphore_.GetCount() <= 0) {
          semaphore_.Signal();
        }
      }
    }
  } else {
    if (eth_packet->timestamp_type == kTimestampTypePps) {
      /** Whether a new sync frame */
      if ((cur_timestamp.stamp < packet_statistic->last_imu_timestamp) &&
          (cur_timestamp.stamp < kPacketTimeGap)) {
        auto cur_time = std::chrono::high_resolution_clock::now();
        int64_t sync_time = cur_time.time_since_epoch().count();
        /**pps模式下，时间戳就是livox设备的开机时间 used receive time as timebase */
        packet_statistic->imu_timebase = sync_time;
      }
    }
    packet_statistic->last_imu_timestamp = cur_timestamp.stamp;

    LidarDataQueue *p_queue = &p_lidar->imu_data;
    if (nullptr == p_queue->storage_packet) {
      uint32_t queue_size = 256;  /* fixed imu data queue size */
      InitQueue(p_queue, queue_size);
      printf("Lidar[%d][%s] imu storage queue size : %d %d\n", p_lidar->handle,
             p_lidar->info.broadcast_code, queue_size, p_queue->size);
    }
    if (!QueueIsFull(p_queue)) {
      QueuePushAny(p_queue, (uint8_t *)eth_packet,
          GetEthPacketLen(eth_packet->data_type),
          packet_statistic->imu_timebase,
          GetPointsPerPacket(eth_packet->data_type));
    }
  }
}

void Lds::PrepareExit(void) {}

}  // namespace livox_ros
