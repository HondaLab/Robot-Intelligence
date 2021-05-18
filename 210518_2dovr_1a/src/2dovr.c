// このプログラムをコンパイルして(2dovr)ロボットの/tmpに置く．
// ベースLinux(BASSE)のプログラムbasseからspawnされる．

#include "pvm_4frobo.c" // pvm_sendなどの関数。HAUTE,BASSEなどマシン名もここに定義してある。
#include "mpu9150.c" // 加速度、ジャイロセンセーのための関数など
#include "servo.c"  //pigpio wrapper for Raspberrypi3
#include "2dov.c" //2dov model

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
//#include <BBB.h>  // BBBでpwm出力するためのライブラリ

#define gain_objx 2.500 // 2.0 for throttle 300
#define gain_objy 3.700 // 3.0 for 300
#define gain_objdy 0 // 0
#define gain_objz 0 //0

#define OBJ_XOS 170 // 170
#define OBJ_WOS 30  //  40
#define OBJ_YOS 80  //  80

#define PULS_ZERO_LEFT 1480000  // 1480000 ns for servo zero
#define PULS_ZERO_RIGHT 1460000  // 1420000 ns for servo zero
#define PULS_UNIT 1000     // usec.  -500 < throttle < +500

#define SRF02_MAX 600
#define PULS4OBJ_Y_MUG 4.20 // 4.2

// calibration factor
#define ML_CALIB 0.8000 //
#define MR_CALIB 0.9999 //
#define ML_OFFSET -0*PULS_UNIT // 0--500
#define MR_OFFSET -0*PULS_UNIT //

// For frb18.0 anisotorpy factor
#define anstpy 1.00 // 2400mAh 0.75

// Reverse avoid factor (2016 3.5)
#define MMRY_N 32 // to keep memmory of height etc
#define REV_AVO_FAC 0.00 // ex 0.70 for GWS8040mod


#define N_INT 256 // the number of integrating values of gyro sensor (2016.2.18)

#define INT_FACTOR 5.50 // ジャイロ値を積分して加速度値に合わせるための定数(2016.2.18)
#define TANH_FAC 10.0

#define PI acos(-1.0)

//#define DATA_OUTPUT "/tmp/data.xy"
//#define MOTORS_OUTPUT "/tmp/motors.xy"

#define INIT_RECV_SRF02 2.0

#define DATASEND_PERIOD 0.1 // ex. 0.2sec
#define FILE_PERIOD 0.1

#define VERSION "1.0.0"

#define HART_L "/sys/class/gpio/gpio45/value"
#define HART_R "/sys/class/gpio/gpio44/value"

// Prototype of functions
// integrates values of gyro sensor to angle
struct threed_type IntegGyro(struct threed_type value, struct threed_type *memory_array);
// integrates values of accel sensor to velocity
struct threed_type IntegTrans(struct threed_type value, struct threed_type *memory_array);


struct threed_type norm_angl(struct threed_type, struct threed_type);
struct threed_type norm_gyro(struct threed_type, struct threed_type);

//void pwm_init(BBB_pwm *pwm);
//void pwm_final(BBB_pwm *pwm);

int Blink(char *file);
int Blink_H();

// Function prototype
// memmory a value to memory_array
void mmry(double value, double *memory_array);

double average(double *memory_array);

//void DriveLeftMotor(BBB_pwm *m, int power); // power : 0--500
//void DriveRightMotor(BBB_pwm *m, int power); // power : 0--500

int HartOn(char *file);
int HartOff(char *file);


int main(int argc,char **argv) {
   int    i;
   int    throttle,rL,rR;
   int    rL_calib,rR_calib;
   int    keyval=0;
   int    flag_height, flag_srf02, flag_pixy, flag_feeler;
   int    round_distance[10];
   int    indc_dist; // Indicator of distance
   int    mpu6050_fd, ak8975_fd;
   int    hmc6352_fd;
   int    puls4obj_x, puls4obj_y, puls4obj_dy, puls4obj_z;
   int    basse_stat;
   int    nhost,narch,infos;
   int    rate_counter;
   int    feeler_LO, feeler_LI, feeler_RO, feeler_RI;
   struct pvmhostinfo *hostp;
   struct obj_type obj1, obj2;

   unsigned char rbuf[6];
   unsigned char wbuf[2];

   double dw;
   double data[256];
   double init_sec, start_sec, now_sec, fstart_sec, rate_start_sec,ov_sec,turn_sec,turn_before_sec,turn_after_sec,file_sec;
   double normal_throttle;
   double sensitiv;
   double accel_mmry[MMRY_N],accel2_mmry[MMRY_N];
   double gyro_mmry[MMRY_N];
   double react_L_sec,react_R_sec,react_F_sec;
   double react_L,react_R,react_F;

   struct threed_type accel,accel2;
   struct threed_type accel_last,accel2_last;
   struct threed_type gyro,gyro2;
   struct threed_type gyro_last,gyro2_last;
   struct threed_type accel_ave,accel2_ave;
   struct threed_type gyro_ave,gyro2_ave;
   struct threed_type offset_accel;
   struct threed_type angl;
   struct threed_type angl_ave;
   struct threed_type last_angl;
   struct threed_type diff_angl;

   float offset_direction;

   //Get Command Line Option
   int opt;
   int longindex;
   int opt_c = 0;
   int opt_x = 0;
   int opt_n = 0;
   int opt_s = 0;
   int opt_v = 0;
   int opt_h = 0;
   char *opt_c_param = NULL;
   struct option long_opts[] = {
     { "version", no_argument, NULL, 'v'},
     { "help", no_argument, NULL, 'h'},
     { "nametag", required_argument, NULL, 'c'},
     { "novisible", no_argument, NULL, 'n'},
     { "noservo", no_argument, NULL, 's'},
     { "nofile", no_argument, NULL, 'x'}
   };
   while((opt = getopt_long(argc,argv,"xc:",long_opts,&longindex)) != -1){
     switch(opt){
       case 'c':
        opt_c = 1;
        opt_c_param = optarg;
        break;
       case 'x':
        opt_x = 1;
        break;
       case 'n':
        opt_n = 1;
        no_visible = 1;
        break;
       case 's':
        opt_s = 1;
        break;
       case 'v':
        opt_v = 1;
        break;
       case 'h':
        opt_h = 1;
        break;
     }
   }

   if( opt_v == 1){
     printf("2dovr version %s\n",VERSION);
     printf("License:CC BY-NC-SA\n");
     printf("Author:Yasushi Honda, Takashi Miyajima\n");
     exit(0);
   }
   if( opt_h == 1){
     printf("２次元最適速度ロボットプログラム\n");
     printf("option argument\n");
     printf("-c comment xyファイルに書くコメント\n");
     printf("-x xyファイルを残さない\n");
     printf("--version\n");
     printf("--help\n");
     exit(0);
   }

   // For making result filename
   time_t timer;
   char str[128];
   char gain_s[128];
   char *file_name;
   FILE *fp,*mfp;

   time(&timer);
   struct tm *local = localtime(&timer);
   char filename[100];
   char hostname[20];
   gethostname(hostname,sizeof(hostname));
   sprintf(filename,"/tmp/%04d%02d%02d_%s_%s_data.xy",local->tm_year + 1900,\
                                      local->tm_mon + 1,\
                                      local->tm_mday,\
                                      hostname,opt_c_param);
   fp=fopen(filename,"w");
   //mfp=fopen(MOTORS_OUTPUT,"a");

   setlinebuf(stdout);

   printf("\n");
   printf("XX=====================XX\n");
   printf("X    2dOVR B1 start!   X\n");
   printf("XX==============(^.^)/=XX\n");

   pvm_config(&nhost,&narch,&hostp);
   for(i=0;i<nhost;i++){
      printf("host name=%s\n",hostp[i].hi_name);
   }


   struct threed_type compass;
   long tempData[1];

   float direction;
/*
   // PWM prepare
   BBB_pwm *mL = BBB_open_pwm("P9_14");
   pwm_init(mL);
   printf("mL initialized \n");

   BBB_pwm *mR = BBB_open_pwm("P8_19");
   pwm_init(mR);
   printf("mR initialized \n");

   //throttle=ZERO;
   DriveLeftMotor(mL,0);
   DriveRightMotor(mR,0);
*/

   //initialise pigpio
   init_pigpio();
   //load calib file
   load_servo_calib_data();
   //init turn
   init_turn();
   //load prms data
   read_prms_file();

/*
   // MPU6050 （加速度　＆　ジャイロセンサ）
   if ((mpu6050_fd = open(ACCEL_I2C, O_RDWR)) < 0){
      printf("Faild to open MPU6050 port.\n");
      exit(1);
   }
   MPU6050_init(mpu6050_fd,MPU6050_ADDR);
*/

   /*
   // HMC6352 （コンパス）
   if ((hmc6352_fd = open(I2C_HMC6352, O_RDWR)) < 0){
      printf("Faild to open HMC6352 port.\n");
      exit(1);
   }
   HMC6352_init(hmc6352_fd);
   */

   pvm_init_4haute();

   //printf("Data recieve from basse starts\n");
   start_sec=gettimeofday_sec();
   init_sec=gettimeofday_sec();

   data[0]=2.78; // Sending 2.78 as signal of starting main loop
                 // The value has NO meaning.
   pvm_send_from_haute_to_basse(data,1);
   //printf("Start signal(%lf) is sent to basse.\n",data[0]);


   // Blocking receive of Keyboard value.
   while(pvm_recv_from_basse_to_haute(data,1)==0){}
   //printf("%lf\n",data[0]);
   keyval=(int)data[0];
   while(keyval!=-999){

      // Gains must be recieved from basse
      while(pvm_recv_from_basse_to_haute(data,2)<=0){}
      sensitiv=data[0];
      normal_throttle=data[1];

      //printf("======================\n");
      //printf("sensitivity=%lf\n",sensitiv);
      //printf("normal_throttle=%lf\n",normal_throttle);
      //printf("======================\n");

      //MPU6050_init(mpu6050_fd,MPU6050_ADDR);

      pvm_spawn_pixy();
      data[0]=(double)OBJ_XOS;
      data[1]=(double)OBJ_WOS;
      data[2]=(double)OBJ_YOS;
      pvm_send_from_haute_to_pixy(data,3);

      //pvm_spawn_feeler();

      throttle=0;
      rate_counter=0;

      rate_start_sec=gettimeofday_sec();
      start_sec=gettimeofday_sec();
      init_sec=gettimeofday_sec();
      ov_sec=gettimeofday_sec();
      turn_sec=gettimeofday_sec();
      file_sec=gettimeofday_sec();

      //Write File Header
      time(&timer);
      fprintf(fp,"# %s",ctime(&timer));
      fprintf(fp,"# Comment : %s\n",opt_c_param);
      fprintf(fp,"# 2odv params :\n");
      fprintf(fp,"# V_S %.3lf\n",vs);
      fprintf(fp,"# SENSITIVITY %.3lf\n",sensitivity);
      fprintf(fp,"# ALPHA %.3lf\n",alpha);
      fprintf(fp,"# BETA %.3lf\n",beta);
      fprintf(fp,"# B %.3lf\n",b);
      fprintf(fp,"# C %.3lf\n",c);
      fprintf(fp,"# delta_t %.3lf\n",delta_t);
      fprintf(fp,"# OBJ_MAX_LENGTH %.3lf\n",r);
      fprintf(fp,"# 2dov params:\n");
      //fprintf(fp,"# ROTATION_TO_PULSEWIDTH_SLOOP %.6lf\n",ROTATION_TO_PULSEWIDTH_SLOOP);
      //fprintf(fp,"# ROTATION_TO_PULSEWIDTH_PLUS_INTERCEPT %.6lf\n",ROTATION_TO_PULSEWIDTH_PLUS_INTERCEPT);
      //fprintf(fp,"# ROTATION_TO_PULSEWIDTH_MINUS_INTERCEPT %.6lf\n",ROTATION_TO_PULSEWIDTH_MINUS_INTERCEPT);
      fprintf(fp,"# LINEAR_SLOOP %.3lf\n",LINEAR_SLOOP);
      fprintf(fp,"# pixy params:\n");
      fprintf(fp,"# LARGE_CUP_A_2 %.4lf\n",LARGE_CUP_A_2);
      fprintf(fp,"# LARGE_CUP_B_2 %.4lf\n",LARGE_CUP_B_2);
      fprintf(fp,"# LARGE_CUP_C_2 %.4lf\n",LARGE_CUP_C_2);
      fprintf(fp,"# robot params:\n");
      fprintf(fp,"# ROBOT_LENGTH %.3lf\n",ROBOT_LENGTH);
      fprintf(fp,"# WHEEL_RADIUS %.3lf\n",WHEEL_RADIUS);
      fprintf(fp,"# turn params:\n");
      fprintf(fp,"# TURN_PERIOD %.3lf\n",TURN_PERIOD);
      fprintf(fp,"# TURN_BACK_SPD %.3lf\n",TURN_BACK_SPD);
      fprintf(fp,"# TURN_BACK_TIME %.3lf\n",TURN_BACK_TIME);
      fprintf(fp,"# TURN_SPD %.3lf\n",TURN_SPD);
      fprintf(fp,"# TURN_TIME %.3lf\n",TURN_TIME);
      fprintf(fp,"# servo calib params:\n");
      fprintf(fp,"# right_calib_alpha beta left_calib_alpha beta %.3lf %.3lf %.3lf %.3lf\n",right_calib_alpha,right_calib_beta,left_calib_alpha,left_calib_beta);
      fprintf(fp,"# HARITSUKI_SX:%.2lf\n",HARITSUKI_SX);
      fprintf(fp,"# HARITSUKI_SY:%.2lf\n",HARITSUKI_SY);
      fprintf(fp,"# time v_r v_l theta length obj_visible\n");

      //fprintf(mfp,"\n# %s",ctime(&timer));
      //fprintf(mfp,"#%s\n",argv[2]);

      react_L_sec=0.0;
      react_R_sec=0.0;
      react_F_sec=0.0;
      while(throttle>=0){

         // pvm_recv(diff)
         // data[0]:Increment of throttle value send from basse
         if(pvm_recv_from_basse_to_haute(data,1)>0){
            throttle+=(int)data[0];
            //printf("%d\n",throttle);
            rL=throttle;
            rR=throttle;
         }

         //Blink_H();

         // Receive position data from PIXY camera via PVM
         flag_pixy=pvm_recv_from_pixy_to_haute(data,12);
         if(flag_pixy>0){
            obj1.signature=(int)data[0];
            obj1.x  =(int)data[1];
            obj1.y  =(int)data[2];
            dw      =(int)data[3];
            obj1.width =(int)data[4];
            obj1.height =(int)data[5];
            puls4obj_x=gain_objx*(obj1.x-OBJ_XOS);
            puls4obj_y=gain_objy*(obj1.width-OBJ_WOS);
            if(obj1.width-OBJ_WOS<0){
               puls4obj_y=PULS4OBJ_Y_MUG*puls4obj_y;
            }
            puls4obj_dy=gain_objdy*dw;
            puls4obj_z=-gain_objz*(obj1.y-OBJ_YOS);
            //printf("%3d %5d %5d %5d %5d \n",obj_sig,obj_x,obj_y,obj_w,obj_h);
         }
         // To follow object
         rL=throttle+puls4obj_x;
         rR=throttle-puls4obj_x;
         rL-=puls4obj_y;
         rR-=puls4obj_y;

         /*
         // Receive feeler data from FEELER via PVM
         flag_feeler=pvm_recv_from_feeler_to_haute(data,4);
         if(flag_feeler>0){
            feeler_LO=(int)data[0];
            feeler_LI=(int)data[1];
            feeler_RO=(int)data[2];
            feeler_RI=(int)data[3];
            //printf("%d %d %d %d\n",feeler_LO,feeler_LI,feeler_RO,feeler_RI);
         }
        */

         /*
         // To react wall (Left)
         now_sec=gettimeofday_sec(); // Mesure second for now
         if(now_sec-react_L_sec<0.7){
            // turn right back
            rL=-throttle*0.8;
            rR=-throttle*0.8;
            HartOn(HART_L);
         }else if(now_sec-react_L_sec<2.0){
            rL=+throttle*0.8;
            rR=-throttle*0.8;
            //Blink(HART_L);
            HartOn(HART_L);
         }else{
            if(feeler_LO>0||feeler_LI>0){
               react_L_sec=gettimeofday_sec();
               react_R_sec=0.0;
            }
            //Blink_H();
            //HartOff(HART_L);
         }

         // To react wall (Right)
         now_sec=gettimeofday_sec(); // Mesure second for now
         if(now_sec-react_R_sec<0.7){
            // turn left back
            rL=-throttle*0.8;
            rR=-throttle*0.8;
            //Blink(HART_R);
            HartOn(HART_R);
         }else if(now_sec-react_R_sec<2.0){
            rL=-throttle*0.8;
            rR=+throttle*0.8;
            //Blink(HART_R);
            HartOn(HART_R);
         }else{
            if(feeler_RO>0||feeler_RI>0){
               react_R_sec=gettimeofday_sec();
               react_L_sec=0.0;
            }
            //Blink_H();
            //HartOff(HART_R);
         }
         */

         // Print RATE
         now_sec = gettimeofday_sec();
         rate_counter++;
         if(now_sec-rate_start_sec>0.2){
            rate_start_sec=gettimeofday_sec();
            //printf("%5.2f %5d\n",now_sec-init_sec,rate_counter);
            rate_counter=0;
         }

         if(now_sec-start_sec>DATASEND_PERIOD){
            // Compus read
            //direction=HMC6352_read(hmc6352_fd)/10.0-180.0;

            start_sec=gettimeofday_sec();

            //data[0]=accel2_ave.z;
            data[0]=pulsewidth_r;
            //data[1]=react_R;
            data[1]=pulsewidth_l;
            //data[2]=gyro.x;
            data[2]=v_r;
            //data[3]=gyro.x-gyro_last.x;
            data[3]=v_l;
            data[4]=(double)obj1.x;
            data[5]=obj_visible_theta;
            data[6]=(double)obj1.width;
            data[7]=obj_visible_distance;
            data[8]=(double)obj_visible;

            pvm_send_from_haute_to_basse(data,9);
            // it had better to commen out this pvm_send in order to decrease time delay in wireless LAN

            // 8 USsensors 0--7 are selected by this indicator
            if(indc_dist==7)indc_dist=0;
            else indc_dist++;
          }

          if(now_sec - file_sec>FILE_PERIOD){

            file_sec = gettimeofday_sec();
            // Output to DATA.xy
            fprintf(fp,"%5.2f ",now_sec-init_sec); // #1
            fprintf(fp,"%4.3lf %4.3lf %6.4f %6.4f %d\n",v_r,\
                                                          v_l,\
                                                          obj_visible_theta,\
                                                          obj_visible_distance,\
                                                          obj_visible);

            // Output to MOTORS_OUTPUT
            //fprintf(mfp,"%5.2f ",now_sec-init_sec);
            //fprintf(mfp,"%6.4f %6.4f\n",v_r,v_l);

          }  // end of DATASEND_PRIOD if

         //2dov model
         if(now_sec-ov_sec > delta_t){
            //printf("sec %lf\n",now_sec-ov_sec);
            //if(now_sec - ov_sec > 0.010) printf("ov time over\n");
            ov_sec=gettimeofday_sec();
            //x_d_now[0] = x_d_next[0];
            //x_d_now[1] = x_d_next[1];
            x_d_now[0] = norm(x_d_next);
            x_d_now[1] = 0;
            //printf("x%lf\n",x_d_now[0]);
            calc_v(&v_r,&v_l,x_d_now,x_d_next,delta_t,robot_length,obj1);
            pulsewidth_r = rotation_to_pulsewidth_linear_right(v_to_rotation(v_r));
            pulsewidth_l = rotation_to_pulsewidth_linear_left(-v_to_rotation(v_l));
            //pulsewidth_r = rotation_to_pulsewidth(v_to_rotation(throttle/100.0));
            //pulsewidth_l = rotation_to_pulsewidth(-v_to_rotation(throttle/100.0));
         }

         //turn
         if(now_sec-turn_sec > TURN_PERIOD){
           turn_sec = gettimeofday_sec();
           turn_before_sec = gettimeofday_sec();
           turn();
           turn_after_sec = gettimeofday_sec();

           rate_start_sec += (turn_after_sec - turn_before_sec);
           start_sec += (turn_after_sec - turn_before_sec);
           ov_sec += (turn_after_sec - turn_before_sec);
         }

         // pwm_output(motors)
         if(throttle>0 && opt_s == 0){
            //DriveLeftMotor(mL,rL);
            //DriveRightMotor(mR,rR);
            send_servo_pulse(GPIO_NUM_RIGHT_SERVO,pulsewidth_r);
            send_servo_pulse(GPIO_NUM_LEFT_SERVO,pulsewidth_l);
         }else{
            //DriveLeftMotor(mL,0);
            //DriveRightMotor(mR,0);
            send_servo_pulse(GPIO_NUM_RIGHT_SERVO,0);
            send_servo_pulse(GPIO_NUM_LEFT_SERVO,0);
         }




      }// end of throttle loop
      //pvm_kill(srf02_tid);
      //pvm_kill(feeler_tid);
      printf("killing pixy\n");
      fflush(stdout);
      pvm_kill(pixy_tid);

      if(flag_height==1){
         now_sec=gettimeofday_sec();
         printf("# flight sec=%5.1f\n",now_sec-fstart_sec);
         fprintf(fp,"# flight sec=%5.1f\n",now_sec-fstart_sec);
         data[0]=now_sec-fstart_sec;
         pvm_send_from_haute_to_basse(data,1);
      }else{
         data[0]=0.0;
         pvm_send_from_haute_to_basse(data,1);
      }


      while(pvm_recv_from_basse_to_haute(data,1)==0){}
      keyval=(int)data[0];
      //printf("keyval=%d\n",keyval);
   }// end of keyval loop


   // fprintf(gains)

   //pwm_final(mL);
   //pwm_final(mR);

   //MPU9150_final(mpu6050_fd);
   //close(mpu6050_fd);

   fclose(fp);
   //fclose(mfp);
   if(opt_x == 1){
     int result = remove(filename);
     printf("result=%d\n",result);
   }


   //terminate pigpio
   terminate_pigpio();

   return 0;
} // end of main

/*
void DriveLeftMotor(BBB_pwm *m, int power){
   int pwm_puls;

   pwm_puls=PULS_ZERO_LEFT+(int)(power*ML_CALIB*PULS_UNIT)+ML_OFFSET;

   // pwm_output(motors)
   m->set_duty(m,pwm_puls);
} // end of DriveLeftMotor

void DriveRightMotor(BBB_pwm *m, int power){
   int pwm_puls;

   pwm_puls=PULS_ZERO_RIGHT-(int)(power*MR_CALIB*PULS_UNIT)-MR_OFFSET;

   // pwm_output(motors)
   m->set_duty(m,pwm_puls);
} // end of DriveLeftMotor
*/

struct threed_type IntegGyro(struct threed_type value, struct threed_type *memory_array){
// integrates values of gyro sensor

   int i;
   struct threed_type integ;

   // Update short range memory of gyro values
   i=0;
   while(i<N_INT-1){
      memory_array[i].x=memory_array[i+1].x;
      memory_array[i].y=memory_array[i+1].y;
      memory_array[i].z=memory_array[i+1].z;
      i++;
   }
   memory_array[N_INT-1].x=value.x;
   memory_array[N_INT-1].y=value.y;
   memory_array[N_INT-1].z=value.z;

   // Calulate integration to get angle
   integ.x=0.0; integ.y=0.0; integ.z=0.0;
   i=0;
   while(i<N_INT-1){
      integ.x += memory_array[i].x;
      integ.y += memory_array[i].y;
      integ.z += memory_array[i].z;
      i++;
   }
   integ.x=-INT_FACTOR*integ.x/N_INT;
   integ.y=-INT_FACTOR*integ.y/N_INT;
   integ.z=-INT_FACTOR*integ.z/N_INT;

   return integ;

}


struct threed_type norm_angl(struct threed_type data,
                             struct threed_type offset){

   struct threed_type angl;

   // normalize and translate sensor axis to frobo axis

   angl.x= data.x*ACCEL_YNORM-offset.x;
   angl.x=angl.x;
   if(angl.x>1.0){
      printf("angl.x=%f\n",angl.x);
      angl.x=1.0;
   }
   if(angl.x<-1.0){
      printf("angl.x=%f\n",angl.x);
      angl.x=-1.0;
   }
   angl.x=asin(angl.x);

   angl.y= data.y*ACCEL_XNORM-offset.y;
   angl.y=angl.y;
   if(angl.y>1.0){
      printf("angl.y=%f\n",angl.y);
      angl.y=1.0;
   }
   if(angl.y<-1.0){
      printf("angl.y=%f\n",angl.y);
      angl.y=-1.0;
   }
   angl.y=-asin(angl.y);

   angl.z= data.z*ACCEL_ZNORM;
   if(angl.z>1.0){angl.z=1.0;}
   if(angl.z<-1.0){angl.z=-1.0;}
   angl.z=asin(angl.z);

   return angl;

}

struct threed_type norm_gyro(struct threed_type data, struct threed_type offset){

   struct threed_type gyro;
   gyro.x= data.x*GYRO_NORM-offset.x;
   gyro.y= data.y*GYRO_NORM-offset.y;
   gyro.z= data.z*GYRO_NORM-offset.z;


   return gyro;

}

/*
void pwm_init(BBB_pwm *pwm) {
  if(pwm == NULL){
    printf("open pwm error.\n");
    exit(0);
  }
  pwm->set_polarity(pwm,0);
  pwm->set_period(pwm,20000000);
  pwm->set_duty(pwm,1000000);

  pwm->start(pwm);
}

void pwm_final(BBB_pwm *pwm) {
  pwm->stop(pwm);
  BBB_close_pwm(pwm);
}
*/

double average(double *memory_array){
    int i;
    double val;

    val=0.0;
    i=0;
    while(i<MMRY_N){
       val+=memory_array[i];
       i++;
    }
    val=val/MMRY_N;

    return val;
}

void mmry(double value, double *memory_array){

   int i;

   i=0;
   while(i<MMRY_N-1){
      memory_array[i]=memory_array[i+1];
      i++;
   }
   memory_array[MMRY_N-1]=value;

}

int Blink(char *file){

   FILE *fp;
   fp=fopen(file,"w");
   fprintf(fp,"1");
   fclose(fp);
   usleep(100000);

   fp=fopen(file,"w");
   fprintf(fp,"0");
   fclose(fp);
   usleep(100000);

}

int Blink_H(){

   FILE *fpL,*fpR;

   fpL=fopen(HART_L,"w");
   fpR=fopen(HART_R,"w");
   fprintf(fpL,"1");
   fprintf(fpR,"1");
   fclose(fpL);
   fclose(fpR);
   usleep(100000);

   fpL=fopen(HART_L,"w");
   fpR=fopen(HART_R,"w");
   fprintf(fpL,"0");
   fprintf(fpR,"0");
   fclose(fpL);
   fclose(fpR);
   usleep(100000);

}

int HartOn(char *file){

   FILE *fp;
   fp=fopen(file,"w");
   fprintf(fp,"1");
   fclose(fp);

}
int HartOff(char *file){

   FILE *fp;
   fp=fopen(file,"w");
   fprintf(fp,"0");
   fclose(fp);

}
