//Compile: gcc -pthread -c servo servo.c -lpigpio

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <math.h>

#include "servo.c"

int main(int argc,char *argv[]){

  if(argc < 2){
    printf("argument:v_r v_l");
    exit(0);
  }

  init_pigpio();

  load_servo_calib_data();

  double v_r = atof(argv[1]);
  double v_l = atof(argv[2]);

  int pulse_r = rotation_to_pulsewidth_linear_right(v_to_rotation(v_r));
  int pulse_l = rotation_to_pulsewidth_linear_left(-v_to_rotation(v_l));

  printf("right servo alpha=%.6lf beta=%.6lf\n",right_calib_alpha,right_calib_beta);
  printf("left servo alpha=%.6lf beta=%.6lf\n",left_calib_alpha,left_calib_beta);
  printf("%d %d\n",pulse_r,pulse_l);

  send_servo_pulse(GPIO_NUM_RIGHT_SERVO,pulse_r);
  send_servo_pulse(GPIO_NUM_LEFT_SERVO,pulse_l);

  int gomi;
  scanf("%d",&gomi);

  terminate_pigpio();
}
