#include <linux/i2c-dev.h> // 
#include <fcntl.h>
#include <sys/ioctl.h>

#define ACCEL_I2C "/dev/i2c-2" // i2cデバイスファイル
#define MPU6050_ADDR 0x68 // MPU6050(加速度／ジャイロセンサ)アドレス
#define MPU6050_ADDR2 0x69 // MPU6050(加速度／ジャイロセンサ)アドレス
#define AK8975_ADDR 0x0c  // AK8975(地磁気センサ)アドレス

#define ACCEL_XNORM 0.0000610 // normalization factor of accel value to one
#define ACCEL_YNORM 0.0000610 // normalization factor of accel value to one
#define ACCEL_ZNORM 0.0000547 // normalization factor of accel value to one
#define GYRO_NORM 0.0000950   // normalization factor of gyro value to radian/sec 
                              // from observed data
#define OFFSET_CALC_SEC 2.0

struct threed_type{
   double x;
   double y;
   double z;
}threed_type;

// 関数プロトタイプ
int MPU9150_write(unsigned char address, unsigned char data, int fd);
unsigned char MPU9150_read(unsigned char address, int fd);
struct threed_type MPU6050_ReadAccel(int fd, int addr);
struct threed_type MPU6050_ReadGyro(int fd, int addr);
struct threed_type AK8975_ReadCompass(int fd, int addr);
void MPU6050_TEMP_readData(long *data, int fd, int addr);
void MPU6050_init(int fd, int addr);
void AK8975_init(int fd);
void MPU9150_final(int fd);

 
// MPU9150
int MPU9150_write(unsigned char address, unsigned char data, int fd){
   unsigned char buf[2];
   buf[0] = address;
   buf[1] = data;
   if((write(fd,buf,2))!=2){
      printf("Error writing to i2c slave\n");
      return -1;
   }
}
   
// MPU9150
unsigned char MPU9150_read(unsigned char address, int fd){
   unsigned char buf[1];
   unsigned char val[1];
   buf[0] = address;
   if((write(fd,buf,1))!= 1){ 
      printf("Error writing to i2c slave\n");
      return -1;
   }
   if(read(fd,val,1)!=1){
      printf("Error reading from i2c slave\n");
      return -1;
   }else{ return val[0];}
}

// 
struct threed_type MPU6050_ReadGyro(int fd, int addr){
   int tmp[6];
   struct threed_type data;
   int i;
   for(i=0; i<6; i++){
      tmp[i]=MPU9150_read(addr+i,fd);
   }
   if(tmp[0]>127){data.x=tmp[0]-255;}
   else{data.x=tmp[0];}
   data.x=data.x*256+tmp[1];

   if(tmp[2]>127){data.y=tmp[2]-255;}
   else{data.y=tmp[2];}
   data.y=data.y*256+tmp[3];

   if(tmp[4]>127){data.z=tmp[4]-255;}
   else{data.z=tmp[4];}
   data.z=data.z*256+tmp[5];

   //printf("%d %d %d \n",data.x,data.y,data.z);
   //usleep(10000);
   return data;
}
 
// 
struct threed_type MPU6050_ReadAccel(int fd, int addr){
   int tmp[6];
   struct threed_type data;
   int offset=32768;
   int i;
   for(i=0; i<6; i++){
      tmp[i]=MPU9150_read(addr+i,fd);
   }
   data.x=tmp[0]*256+tmp[1];
   if(data.x>offset){data.x=2*offset-data.x;}
   else{data.x=-data.x;}
   data.y=tmp[2]*256+tmp[3];
   if(data.y>offset){data.y=2*offset-data.y;}
   else{data.y=-data.y;}
   data.z=tmp[4]*256+tmp[5];
   if(data.z>offset){data.z=2*offset-data.z;}
   else{data.z=-data.z;}
 

   //printf("%d %d %d \n",data.x,data.y,data.z);
   return data;
}

// 
void MPU6050_TEMP_readData(long *data, int fd, int addr){
   int tmp[2];
   int i; 
   for(i=0; i<2; i++){
      tmp[i]=MPU9150_read(addr+i,fd);
   }
   data[0]=(tmp[0]<<8|tmp[1]);
}

// AK8975
struct threed_type AK8975_ReadCompass(int fd, int addr){
   unsigned char Data;
   int tmp[6];
   int i;
   struct threed_type value;
   int offset=4096;

   Data = MPU9150_read(0x02,fd);
   //printf("AK8975_state=%x\n",Data);
   if(Data == 0x01){
      for(i=0; i<6; i++){
         tmp[i]=MPU9150_read(addr+i,fd);
      }
      value.x=(tmp[1]&31)*256+tmp[0];
      if(value.x>offset){value.x=value.x-2*offset;}
      value.y=(tmp[3]&31)*256+tmp[2];
      if(value.y>offset){value.y=value.y-2*offset;}
      value.z=(tmp[5]&31)*256+tmp[4];
      if(value.z>offset){value.z=value.z-2*offset;}

      MPU9150_write(0x0a, 0x01, fd);

      return value; 
   }
}

// 
void MPU6050_init(int fd, int addr){
   unsigned char Data;

   if (ioctl(fd, I2C_SLAVE, addr) < 0){   
      printf("Unable to get bus access to talk to MPU6050\n");
      exit(1);
   }

   //printf("MPU6050 init seq. start\n");
   Data = MPU9150_read(0x75,fd);
   //printf("Who am I:%s %x\n",ACCEL_I2C,addr);
   if((Data != 0x68)&&(Data != 0x69)){
      printf("MPU6050 is NOT working\n");
      exit(1);
   }

   //Digital Low Pass Ffilter (DLPF) 
   Data = MPU9150_read(0x1a,fd);
   //printf("DLPF_CONFIG=%u\n",Data);
   MPU9150_write(0x1a, 0x05, fd); // 0x06:5Hz cutoff
   fsync(fd);
   Data = MPU9150_read(0x1a,fd);
   //printf("Modified DLPF_CONFIG=%u\n",Data);

   //printf("read OK, Now writing check...\n");
   MPU9150_write(0x6b, 0x00, fd);
   Data = MPU9150_read(0x6b,fd);
   if(Data != 0x00){
      printf("Writing miss\n");
      exit(1);
   }
   MPU9150_write(0x37, 0x02, fd);
   Data = MPU9150_read(0x37,fd);
   if(Data != 0x02){
      printf("Writing miss\n");
      exit(1);
   }
}

// 
void AK8975_init(int fd){
   unsigned char Data;
     
   //printf("I2C_SLAVE=%d\n",I2C_SLAVE);
   //printf("AK8975_ADDR=%x\n",AK8975_ADDR);
   if (ioctl(fd, I2C_SLAVE, AK8975_ADDR) < 0){   
      printf("Unable to get bus access to talk to slave\n");
      exit(1);
   }   
     
   printf("AK8975 init seq. start\n");
   Data = MPU9150_read(0x00,fd);
   if(Data != 0x48){
      printf("AK8975 is not working\n");
      exit(1);
   }
   MPU9150_write(0x0a, 0x01, fd);

   printf("Compass OK\n");
}

// 
void MPU9150_final(int fd){
   MPU9150_write(0x6b, 0x80, fd);
}

