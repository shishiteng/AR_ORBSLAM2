#include <iostream>
#include "Imu.h"

using namespace std;

/************************************分割线******************************************/
//此处开始是读取串口
int speed_arr[] = { B115200,  B38400, B19200, B9600, B4800, B2400, B1200, B300,
		    B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = { 115200, 38400,  19200,  9600,  4800,  2400,  1200,  300,
		  38400,  19200,  9600, 4800, 2400, 1200,  300, };
void set_speed(int fd, int speed)
{
  int   i;
  int   status;
  struct termios   Opt;
  int ret = -1;

  ret = tcgetattr(fd, &Opt);
  if(ret < 0) {
    printf("Unable to get the attribute.\n");
    return;
  }

  for ( i= 0;  i < sizeof(speed_arr)/sizeof(int);  i++) {
    if  (speed == name_arr[i]) {
      tcflush(fd, TCIOFLUSH);
      cfsetispeed(&Opt, speed_arr[i]);
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt);
      if  (status != 0)
	perror("tcsetattr fd1");
      return;
    }
    tcflush(fd,TCIOFLUSH);
  }
}

/**
 *@brief   设置串口数据位，停止位和效验位
 *@param  fd     类型  int  打开的串口文件句柄*
 *@param  databits 类型  int 数据位   取值 为 7 或者8*
 *@param  stopbits 类型  int 停止位   取值为 1 或者2*
 *@param  parity  类型  int  效验类型 取值为N,E,O,,S
 */
int set_Parity(int fd,int databits,int stopbits,int parity)
{
  struct termios options;
  if  ( tcgetattr( fd,&options)  !=  0)
    {
      perror("SetupSerial 1");
      return(false);
    }
  options.c_cflag &= ~CSIZE;
  switch (databits) /*设置数据位数*/
    {
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
      fprintf(stderr,"Unsupported data size\n");
      return (false);
    }
  switch (parity)
    {
    case 'n':
    case 'N':
      options.c_cflag &= ~PARENB;   /* Clear parity enable */
      options.c_iflag &= ~INPCK;     /* Enable parity checking */
      break;
    case 'o':
    case 'O':
      options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/ 
      options.c_iflag |= INPCK;             /* Disnable parity checking */
      break;
    case 'e':
    case 'E':
      options.c_cflag |= PARENB;     /* Enable parity */
      options.c_cflag &= ~PARODD;   /* 转换为偶效验*/  
      options.c_iflag |= INPCK;       /* Disnable parity checking */
      break;
    case 'S':
    case 's':  /*as no parity*/
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
      fprintf(stderr,"Unsupported parity\n");
      return (false);
    }
  /* 设置停止位*/   
  switch (stopbits)
    {
    case 1:
      options.c_cflag &= ~CSTOPB;
      break;
    case 2:
      options.c_cflag |= CSTOPB;
      break;
    default:
      fprintf(stderr,"Unsupported stop bits\n");
      return (false);
    }
  /* Set input parity option */
  if (parity != 'n')
    options.c_iflag |= INPCK;
  options.c_cc[VTIME] = 150; // 15 seconds
  options.c_cc[VMIN] = 0;

  tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
      perror("SetupSerial 3");
      return (false);
    }
  return (true);
}


int OpenDev(char *Dev)
{
  int fd = open( Dev, O_RDWR | O_NOCTTY | O_NDELAY );         //| O_NOCTTY | O_NDELAY
  if (-1 == fd)
    { /*设置数据位数*/
      cerr<<"Can't Open Serial Port"<<endl;
      return -1;
    }
  else
    return fd;

}

/*--------------------------------------读取数据-----------------------------------*/
enum input_status {
  STATUS_IDLE, /* 空闲态 */
  STATUS_SOF, /* 帧起始 */
  STATUS_LEN, /* 长度字节 */
  STATUS_DATA, /* 数据 */
  STATUS_FCS, /* 帧校验 */
};

static enum input_status status = STATUS_IDLE; /* running status machine */
static uint8_t rev_buf[64]; /* 状态机常量 */
imu_data_t imu_data;

int imu_rev_get_data(imu_data_t *data)
{
  data->accl[0] = (rev_buf[0]<<8) + rev_buf[1];
  data->accl[1] = (rev_buf[2]<<8) + rev_buf[3]; 
  data->accl[2] = (rev_buf[4]<<8) + rev_buf[5];
  data->gyro[0] = (rev_buf[6]<<8) + rev_buf[7];
  data->gyro[1] = (rev_buf[8]<<8) + rev_buf[9];
  data->gyro[2] = (rev_buf[10]<<8) + rev_buf[11];
  data->mag[0] = (rev_buf[12]<<8) + rev_buf[13];
  data->mag[1] = (rev_buf[14]<<8) + rev_buf[15];
  data->mag[2] = (rev_buf[16]<<8) + rev_buf[17];
  data->pitch = (rev_buf[18]<<8) + rev_buf[19];
  data->roll = (rev_buf[20]<<8) + rev_buf[21];
  data->yaw = (rev_buf[22]<<8) + rev_buf[23];
  data->presure = (rev_buf[27]<<24) + (rev_buf[26]<<16) + (rev_buf[25]<<8) + (rev_buf[24]<<0);
  //printf("P/R/Y/P:%05d %05d %05d %05d\r", data->pitch/100, data->roll/100, data->yaw/10, data->presure);

  data->pitch /= 100;
  data->roll /= 100;
  data->yaw /= 10;
  return 0;
}

void imu_rev_process(char ch)
{
  static int len = 0;
  static int i; /* 状态机 */

  switch(status) {
  case
    STATUS_IDLE:
    if((uint8_t)ch == 0x88) { 
      status = STATUS_SOF; 
    }
    break;
  case
    STATUS_SOF:
    if((uint8_t)ch == 0xAF) { 
      status = STATUS_LEN; 
    } 
    break;
  case
    STATUS_LEN:
    len = ch; 
    status = STATUS_DATA; 
    i = 0; 
    break;
  case
    STATUS_DATA:
    if(i == len) {
      //printf(" data:%x\n",rev_buf);
      status = STATUS_IDLE; /* 完整的接收完一帧数据,调用解析函数 */
      imu_rev_get_data(&imu_data);
      break;
    }
    rev_buf[i++] = ch; 
    break;
  default: break;
  }
}


bool isComplete(unsigned char *buff)
{
  if( !(buff[0] == 0x88 && buff[1] == 0xAF && buff[2] == 0x1C) ) 
    return false;


  int i = 0;
  unsigned char sum = 0;
  for(i=0;i<30;i++) {
    sum += buff[i];
  }

  printf(" %d,%d\n",sum,buff[31]);
  if (sum != buff[31])
    return false;

  return true;
}

void setImuData(unsigned char*buff,imu_data_t *data)
{
  unsigned char *rev_buf = buff+3;
  data->accl[0] = (rev_buf[0]<<8) + rev_buf[1];
  data->accl[1] = (rev_buf[2]<<8) + rev_buf[3]; 
  data->accl[2] = (rev_buf[4]<<8) + rev_buf[5];
  data->gyro[0] = (rev_buf[6]<<8) + rev_buf[7];
  data->gyro[1] = (rev_buf[8]<<8) + rev_buf[9];
  data->gyro[2] = (rev_buf[10]<<8) + rev_buf[11];
  data->mag[0] = (rev_buf[12]<<8) + rev_buf[13];
  data->mag[1] = (rev_buf[14]<<8) + rev_buf[15];
  data->mag[2] = (rev_buf[16]<<8) + rev_buf[17];
  data->pitch = (rev_buf[18]<<8) + rev_buf[19];
  data->roll = (rev_buf[20]<<8) + rev_buf[21];
  data->yaw = (rev_buf[22]<<8) + rev_buf[23];
  data->presure = (rev_buf[27]<<24) + (rev_buf[26]<<16) + (rev_buf[25]<<8) + (rev_buf[24]<<0);

  data->pitch /= 100;
  data->roll /= 100;
  data->yaw /= 10;
  //printf("P/R/Y/P:%05d %05d %05d %05d\r", data->pitch/100, data->roll/100, data->yaw/10, data->presure);
}

void printImuData(imu_data_t *data)
{
  printf("    gyro:%d %d %d\n",data->gyro[0],data->gyro[1],data->gyro[2]);
  printf("    mag:%d %d %d\n",data->mag[0],data->mag[1],data->mag[2]);
  printf("    P/R/Y/P:%05d %05d %05d %05d\n", data->pitch/100, data->roll/100, data->yaw/10, data->presure);
}


int main_(int argc, char **argv)
{
  int fd;
  int nread;
  unsigned char buff[128] = {0};
  imu_data_t data;

  char *dev ="/dev/ttyUSB0";
  fd = OpenDev(dev);
  if (fd>0)
    set_speed(fd,115200);
  else {
      printf("Can't Open Serial Port!\n");
      exit(0);
  }

  if (set_Parity(fd,8,1,'N')== false) {
    printf("Set Parity Error\n");
    exit(1);
  }

  while(1) {
    while((nread = read(fd,buff,sizeof(buff)))>0) {
      //buff[nread] = "\0";
      printf("len:%d  ",nread);
      for(int i=0;i<nread;i++)
	printf("%x",buff[i]);
      printf("\n");

      if(isComplete(buff)) {
	setImuData(buff,&data);
	printImuData(&data);
      }
    }
  }

  return 0;
}


Imu::Imu()
{
  
}

int Imu:: Init(char *dev, int speed)
{
  mFd = OpenDev(dev);
  if (mFd>0)
    set_speed(mFd,115200);
  else {
    cerr<<"Can't Open Serial Port!\n";
    perror("Can't Open Serial Port!\n");
    return -1;
  }

  if (set_Parity(mFd,8,1,'N') == false) {
    cerr<<"Set Parity Error\n";
    return -1;
  }

  return 0;
}

int Imu::SendCmd(const char *cmd)
{
  int n = write(mFd,cmd,strlen(cmd));
  if(n == strlen(cmd)) {
    printf("send command success.\n");
    return 0;
  }

  return -1;
}

int Imu::GetData(imu_data_t *data)
{
  int nread = -1;
  unsigned char buff[128] = {0};

  nread = read(mFd,buff,sizeof(buff));
  if( nread > 0) {
    //printf("len:%d  ",nread);
  }

  if(nread != 32) {
    return -1;
  }

  buff[nread] = '\0';
  if(isComplete(buff)) {
    setImuData(buff,data);
    //printImuData(data);
    return 0;
  }

  return -1;
}

#if 0
void Imu::Run()
{
  while(1) {
    unsigned char buff[64] = {0};
    int i = 0;
    int n = read(mFd,buff,sizeof(buff));
    while( i++ < n ) {
      imu_rev_process(buff[i]);
      if( STATUS_IDLE == status ) {
	memcpy(&mData,&imu_data,sizeof(imu_data_t));
	//printImuData(&imu_data);
      }
    }
  }

}
#endif

void Imu::Run()
{
  imu_data_t data;
  while(1) {
    if ( 0 == GetData(&data)) {
      memcpy(&mData,&data,sizeof(imu_data_t));
    }
  }

}

