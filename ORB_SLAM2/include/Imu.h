#ifndef _IMU_H_
#define _IMU_H_


#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>   /**/
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX终端控制定义*/
#include     <errno.h>      /*错误号定义*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <mutex>

typedef struct imu_data
{
  int16_t accl[3];
  int16_t gyro[3];
  int16_t mag[3];
  int16_t roll;
  int16_t pitch;
  uint16_t yaw;
  int32_t presure;
}imu_data_t;

int imu_get_data(char *buff,imu_data_t *data);

class Imu
{
 public:
  Imu();
  
  int Init(char *dev, int speed);
  int SendCmd(const char *cmd);
  int GetData(imu_data_t *data);
  void Run();

 public:
  std::mutex mMutexImu;
  imu_data_t mData;

 private:
  int mFd;
};


#endif
