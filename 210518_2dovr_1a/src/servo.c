/*
説明：サーボとスイッチONOFF判定を動かすためのプログラム
RaspberryPi3のpigpio使用
2dovr用
作成：20170629
オーサー：宮島
ライセンス：CC BY_NC_SA
Compile: gcc -pthread -c servo servo.c -lpigpio
Reference:http://abyz.co.uk/rpi/pigpio/index.html
*/

#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>

#define WHEEL_RADIUS 0.037 //meter

//GPIO number of Servo for Raspberrypi3
#define GPIO_NUM_RIGHT_SERVO 18
#define GPIO_NUM_LEFT_SERVO 17

#define GPIO_TOUCH_RIGHT 23
#define GPIO_TOUCH_LEFT 22

#define TURN_PERIOD 0.001

#define ROTATION_TO_PULSEWIDTH_SLOOP -5.8
#define ROTATION_TO_PULSEWIDTH_PLUS_INTERCEPT 1510.95
#define ROTATION_TO_PULSEWIDTH_MINUS_INTERCEPT 1518.04
#define LINEAR_SLOOP -6.8

#define TURN_BACK_SPD 0.0 //m/s
#define TURN_BACK_TIME 0.05 //sec
#define TURN_SPD 0.3
#define TURN_TIME 0.4

int pulsewidth_r = 1520; //deault newtral pulsewidth
int pulsewidth_l = 1520;

double right_calib_alpha = 1.0;
double right_calib_beta = 0.0;
double left_calib_alpha = 1.0;
double left_calib_beta = 0.0;

void init_pigpio();
void terminate_pigpio();
void send_servo_pulse(int gpio_port,int pulsewidth);
int init_turn();
int turn();

void init_pigpio(){
  gpioCfgClock(1,1,0); //ideal clock period is 1us
  gpioCfgBufferSize(200); //buffer size
  int flag = gpioInitialise();
  if(flag < 0){
    printf("pigpio initialise failed");
    exit(0);
  }
}

void load_servo_calib_data(){
  char filename[100];
  char hostname[20];
  gethostname(hostname,sizeof(hostname));
  sprintf(filename,"/tmp/%s_servo_calib_data",hostname);
  printf("%s\n",filename);

  FILE *fp;
  fp = fopen(filename,"r");
  if(fp == NULL){
    printf("calibdata notfound!\n");
    return;
  }else{
    fscanf(fp,"%lf %lf %lf %lf",&right_calib_alpha,&right_calib_beta,&left_calib_alpha,&left_calib_beta);
    fclose(fp);
  }
  printf("right alpha=%.1lf,beta=%.1lf left alpha=%.1lf,beta=%.1lf\n",right_calib_alpha,right_calib_beta,left_calib_alpha,left_calib_beta);
}

void terminate_pigpio(){
  gpioTerminate();
}

void send_servo_pulse(int gpio_port,int pulsewidth){
  gpioServo(gpio_port,pulsewidth);
}

double v_to_rotation(double v){
  return v / WHEEL_RADIUS;
}

//parallax servoの対応
int rotation_to_pulsewidth(double rotation){

  if(rotation > 0){
    return round(rotation * ROTATION_TO_PULSEWIDTH_SLOOP + ROTATION_TO_PULSEWIDTH_PLUS_INTERCEPT);
  }else{
    return round(rotation * ROTATION_TO_PULSEWIDTH_SLOOP + ROTATION_TO_PULSEWIDTH_MINUS_INTERCEPT);
  }
}

//1500us neutral liner function
int rotation_to_pulsewidth_linear_right(double rotation){

  return round( rotation * LINEAR_SLOOP * right_calib_alpha + 1500 + right_calib_beta);
}

int rotation_to_pulsewidth_linear_left(double rotation){

  return round( rotation * LINEAR_SLOOP * left_calib_alpha + 1500 + left_calib_beta);
}

int init_turn(){

  gpioSetMode(GPIO_TOUCH_RIGHT,PI_INPUT);
  gpioSetMode(GPIO_TOUCH_LEFT,PI_INPUT);
  gpioSetMode(27,PI_INPUT);

  return 0;
}

int turn(){

  int flag = 0;

  while(flag == 0){

    int right = gpioRead(GPIO_TOUCH_RIGHT);
    int left = gpioRead(GPIO_TOUCH_LEFT);
    int right27 = gpioRead(27);
    
    if( right27 == 1 ){
        right = right27;
     }

    //printf("r=%d,l=%d\n",right,left);

    //0.5sec back  and  90 degree rorate
    if( right == 0 && left == 0){
      flag = 1;
    }else if( right == 1){
      send_servo_pulse(GPIO_NUM_RIGHT_SERVO,rotation_to_pulsewidth_linear_right(-v_to_rotation(TURN_BACK_SPD)));
      send_servo_pulse(GPIO_NUM_LEFT_SERVO,rotation_to_pulsewidth_linear_left(v_to_rotation(TURN_BACK_SPD)));
      usleep(TURN_BACK_TIME * 1000000);
      send_servo_pulse(GPIO_NUM_RIGHT_SERVO,rotation_to_pulsewidth_linear_right(v_to_rotation(TURN_SPD)));
      send_servo_pulse(GPIO_NUM_LEFT_SERVO,rotation_to_pulsewidth_linear_left(v_to_rotation(TURN_SPD)));
      usleep(TURN_TIME * 1000000);
      send_servo_pulse(GPIO_NUM_RIGHT_SERVO,rotation_to_pulsewidth_linear_right(-v_to_rotation(TURN_BACK_SPD)));
      send_servo_pulse(GPIO_NUM_LEFT_SERVO,rotation_to_pulsewidth_linear_left(v_to_rotation(TURN_BACK_SPD)));
      usleep(TURN_BACK_TIME * 1000000);
    }else if( left == 1){
      send_servo_pulse(GPIO_NUM_RIGHT_SERVO,rotation_to_pulsewidth_linear_right(-v_to_rotation(TURN_BACK_SPD)));
      send_servo_pulse(GPIO_NUM_LEFT_SERVO,rotation_to_pulsewidth_linear_left(v_to_rotation(TURN_BACK_SPD)));
      usleep(TURN_BACK_TIME * 1000000);
      send_servo_pulse(GPIO_NUM_RIGHT_SERVO,rotation_to_pulsewidth_linear_right(-v_to_rotation(TURN_SPD)));
      send_servo_pulse(GPIO_NUM_LEFT_SERVO,rotation_to_pulsewidth_linear_left(-v_to_rotation(TURN_SPD)));
      usleep(TURN_TIME * 1000000);
      send_servo_pulse(GPIO_NUM_RIGHT_SERVO,rotation_to_pulsewidth_linear_right(-v_to_rotation(TURN_BACK_SPD)));
      send_servo_pulse(GPIO_NUM_LEFT_SERVO,rotation_to_pulsewidth_linear_left(v_to_rotation(TURN_BACK_SPD)));
      usleep(TURN_BACK_TIME * 1000000);
    }
  }


  return 0;
}
