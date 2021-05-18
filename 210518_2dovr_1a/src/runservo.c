//Compile: gcc -pthread -c servo servo.c -lpigpio

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <math.h>

#include "servo.c"

int main(int argc,char *argv[]){

  init_pigpio();

  double rot_right = atof(argv[1]);
  double rot_left = -atof(argv[1]);

  double offset_right = atof(argv[2]);
  double offset_left = atof(argv[3]);

  rot_right = rot_right / 60 * 3.14159 * 2.0;
  rot_left = rot_left / 60 * 3.14159 * 2.0;

  int pulse1,pulse2;

  if(rot_right > 0){
    pulse1 = round(rot_right * ROTATION_TO_PULSEWIDTH_SLOOP + ROTATION_TO_PULSEWIDTH_PLUS_INTERCEPT + offset_right);
  }else{
    pulse1 = round(rot_right * ROTATION_TO_PULSEWIDTH_SLOOP + ROTATION_TO_PULSEWIDTH_MINUS_INTERCEPT + offset_right);
  }

  if(rot_left > 0){
    pulse2 = round(rot_left * ROTATION_TO_PULSEWIDTH_SLOOP + ROTATION_TO_PULSEWIDTH_PLUS_INTERCEPT + offset_left);
  }else{
    pulse2 = round(rot_left * ROTATION_TO_PULSEWIDTH_SLOOP + ROTATION_TO_PULSEWIDTH_MINUS_INTERCEPT + offset_left);
  }

  printf("%d %d\n",pulse1,pulse2);

  send_servo_pulse(GPIO_NUM_RIGHT_SERVO,pulse1);
  send_servo_pulse(GPIO_NUM_LEFT_SERVO,pulse2);

  int gomi;
  scanf("%d",&gomi);

  terminate_pigpio();
}
