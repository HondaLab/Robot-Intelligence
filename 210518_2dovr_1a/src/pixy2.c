#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "pvm_4frobo.c" // pvm_sendなどの関数。HAUTE,BASSEなどマシン名もここに定義してある。

#define PIXY_DEV "/dev/i2c-1"
#define PIXY_ADD1 0x56
#define PIXY_ADD2 0x55

#define SLEEP 20000
#define MMRY_N  16
#define PERIOD 0.020 // 50Hz

int pixy_read(int fd, int address);
void mmry(int value, int *memory_array);

int x_mmry[MMRY_N],y_mmry[MMRY_N],w_mmry[MMRY_N];


struct obj_type watch_obj(int fd, int ADDRESS);

int main(){
   int checksum,signature;
   int count;
   int i;
   int fd;
   int rate;

   double x_ave, y_ave, w_ave;
   double obj_xos, obj_wos, obj_yos;
   double w_old, dw;
   double data[256];
   double time_init;
   double time;

   struct obj_type obj1,obj2;
   struct obj_type obj1_mmry[MMRY_N];

   setlinebuf(stdout);

   pvm_init_4pixy();

   while(pvm_recv_from_haute_to_pixy(data,3)==0){
      printf("Receiving from haute to pixy\n");
   }
   obj_xos=data[0];
   obj_wos=data[1];
   obj_yos=data[2];
   //printf("obj_Xos@pixy=%lf\n",obj_xos);
   //printf("obj_Wos@pixy=%lf\n",obj_wos);
   //printf("obj_Yos@pixy=%lf\n",obj_yos);

   fd=open(PIXY_DEV, O_RDWR);

   i=0;count=0;signature=0;
   w_old=0.0;
   time_init=gettimeofday_sec();
   while(1){ // Infinit loop to observe object
      // watch objects
      obj1=watch_obj(fd,PIXY_ADD1);
      //obj2=watch_obj(fd,PIXY_ADD2);
      if(obj1.signature>0){
         //printf("%3d %5d %5d %5d %5d \n",obj1.signature,obj1.x,obj1.y,obj1.width,obj1.height);
         count++;
      }

      usleep(SLEEP);

      time=gettimeofday_sec();
      if(time-time_init>PERIOD){
         //rate=(int)((double)count/PERIOD);
         //printf("signature @pixy=%d\n",signature);
         dw=(w_ave-w_old)/PERIOD;
         //printf("dw=%8.2lf\n",dw);
         w_old=w_ave;
         if((obj1.signature==1)||(obj2.signature==1)){
            data[0]=(double)obj1.signature;
            data[1]=(double)obj1.x;
            data[2]=(double)obj1.y;
            data[3]=dw;
            data[4]=(double)obj1.width;
            data[5]=(double)obj1.height;
            data[6]=(double)obj2.signature;
            data[7]=(double)obj2.x;
            data[8]=(double)obj2.y;
            data[9]=dw;
            data[10]=(double)obj2.width;
            data[11]=(double)obj2.height;
         }else{
            data[0]=-1.0; // No obj watched
            data[1]=0.0;
            data[2]=0.0;
            data[3]=0.0;
            data[4]=0.0;
            data[5]=0.0;
            data[6]=-1.0; // No obj watched
            data[7]=0.0;
            data[8]=0.0;
            data[9]=0.0;
            data[10]=0.0;
            data[11]=0.0;
         }
         //printf("%3.0f %5.0f %5.0f \n",data[6],data[7],data[8]);
         pvm_send_from_pixy_to_haute(data,12);
         time_init=time;
         count=0;
      }

   }

   close(fd);
}


struct obj_type watch_obj(int fd, int ADDRESS){
   struct obj_type obj;

   int i;
   int signature,checksum;
   int pixy_data;
   int x,y;
   int x_ave,y_ave,w_ave;
   int width,height;

   // No object watched
   signature=-1;
   x=-1;
   y=-1;
   width=-1;
   height=-1;

   pixy_data=pixy_read(fd,ADDRESS);
   if(pixy_data==0xaa55){ // finding obj
      pixy_data=pixy_read(fd,ADDRESS);
      if(pixy_data==0xaa55){ // New frame
         checksum=pixy_read(fd,ADDRESS);
      }else{
         checksum=pixy_data;
      }
      signature=pixy_read(fd,ADDRESS);
      x=pixy_read(fd,ADDRESS);
      y=pixy_read(fd,ADDRESS);
      width=pixy_read(fd,ADDRESS);
      height=pixy_read(fd,ADDRESS);

      //printf("%3d %5d %5d %5d %5d \n",signature,x,y,width,height);

      // For averaging
      mmry(x,x_mmry);
      mmry(y,y_mmry);
      mmry(width,w_mmry);
      //printf("%3d %5d %5d %5d %5d \n",signature,x,y,width,height);

      x_ave=0.0;
      y_ave=0.0;
      w_ave=0.0;
      i=0;
      while(i<MMRY_N){
         x_ave+=x_mmry[i];
         y_ave+=y_mmry[i];
         w_ave+=w_mmry[i];
         i++;
      }
      x_ave=x_ave/MMRY_N;
      y_ave=y_ave/MMRY_N;
      w_ave=w_ave/MMRY_N;



   } // end if finding obj
   obj.signature=signature;
   obj.x=x_ave;
   obj.y=y_ave;
   obj.width=w_ave;
   obj.height=height;

   return obj;
} // end of watch_obj


int pixy_read(int fd, int address){
   int sync;
   unsigned char rbuf[2];

   ioctl(fd, I2C_SLAVE, address);

   read(fd,rbuf,2);
   sync=rbuf[1]*256+rbuf[0];

   return sync;
}

void mmry(int value, int *memory_array){

   int i;

   i=0;
   while(i<MMRY_N-1){
      memory_array[i]=memory_array[i+1];
      i++;
   }
   memory_array[MMRY_N-1]=value;

}
