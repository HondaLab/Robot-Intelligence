#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pigpio.h>
#include <signal.h>

//#include "2dov.c"
//#include "pvm_4frobo.c"

#define MAX_OBJ 10
#define PIXY_ADDRESS 0x56

struct obj{
  int x;
  int y;
  int w;
  int h;
};

struct signature{
  int num_of_obj;
  struct obj obj[MAX_OBJ];
};

struct pixyframe{
  struct signature signature[7];
};

int pipix();
void clear_frame(struct pixyframe *frame);
void print_frame(struct pixyframe frame);
void sig_catch(int sig);

int handle;
int run=0;

int main(){

  struct pixyframe frame;
  int signature;
  int prev_signature;
  int x,y,w,h;
  int objnum = 0;
  int r;
  int new_frame_flag = 1;
  int checksum;
  int i;

  clear_frame(&frame);

  gpioInitialise();

  gpioSetSignalFunc(SIGINT, sig_catch);
  handle = i2cOpen(1,PIXY_ADDRESS,0);

  while(run==0){
    r = pipix();
    if( r == 0xaa55 ){

      r = pipix();
      if( r == 0xaa55 ){
        //new frame
        new_frame_flag = 1;
        checksum = pipix();
      }else{
        new_frame_flag = 0;
        checksum = r;
      }

      if( new_frame_flag == 1) print_frame(frame);

      signature = pipix();
      x = pipix();
      y = pipix();
      w = pipix();
      h = pipix();

      //printf("sig%d %d\n",signature,x);


      if( prev_signature != signature || new_frame_flag == 1){
        objnum = 0;
      }else{
        objnum++;
      }


      frame.signature[signature -1 ].num_of_obj = objnum + 1;
      frame.signature[signature -1 ].obj[objnum].x = x;
      frame.signature[signature -1 ].obj[objnum].y = y;
      frame.signature[signature -1 ].obj[objnum].w = w;
      frame.signature[signature -1 ].obj[objnum].h = h;

      prev_signature = signature;

    }else{
      //no obj
      //clear_frame(&frame);
      //usleep(1000000);
    }
  }

  i2cClose(handle);
  gpioTerminate();

  return 0;
}

int pipix(){

  int c;
  int w;
  c = i2cReadByte(handle);
  w = i2cReadByte(handle);
  return w * 256 + c;
}

void clear_frame(struct pixyframe *frame){

  int i;

  for(i=0;i<7;i++){
    frame->signature[i].num_of_obj = 0;
  }
}

void print_frame(struct pixyframe frame){

  //printf("\033[%d;%dH" ,1,1);

  static int cnt = 0;
  printf("%d\n",cnt++);

  int i,j;

  for(i=0;i<7;i++){
    printf("Sig%d ",i);

    for(j=0;j<frame.signature[i].num_of_obj;j++){
      printf("x %3d y %3d w %3d h %3d  ",frame.signature[i].obj[j].x,\
                             frame.signature[i].obj[j].y,\
                             frame.signature[i].obj[j].w,\
                             frame.signature[i].obj[j].h);
    }

    printf("\n");
  }
  //printf large cup distance
  //printf("%.1lfcm\n",(LARGE_CUP_A / pow(frame.signature[0].obj[0].w,LARGE_CUP_C) + LARGE_CUP_B));
  printf("\n");
}

void sig_catch(int sig){
  //printf("run1\n");
  run = 1;
}
