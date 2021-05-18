/*
説明：2dovr用の2次元最適速度モデル
作成：20170630
オーサー：宮島
ライセンス：CC BY_NC_SA
Compile:
Reference:
*/

#include <math.h>

//#include "pvm_4frobo.c"

//#define V_S 0.0 //m/s
//#define SENSITIVITY 4.0
//#define ALPHA 0.05
//#define BETA 32.0
//#define B 0.2
//#define C 2.0
#define DELTA_T 0.01
#define ROBOT_LENGTH 0.12

#define OBJ_MIN_LENGTH 0.0
//#define OBJ_MAX_LENGTH 2.0

#define LARGE_CUP_A_2 775.928
#define LARGE_CUP_B_2 -32.8268
#define LARGE_CUP_C_2 0.5969

//20171130.prm用パラメータ
#define LARGE_CUP_A 1085.19
#define LARGE_CUP_B -15.0269
#define LARGE_CUP_C 0.756091

#define HARITSUKI_SX 10.0
#define HARITSUKI_SY 10.0

#define PRMS_FILE "/tmp/prms"

/*
2dovr用2次元最適速度によるタイヤの回転速度を求める
v_r 求めたい右タイヤの速度が入る
v_l 求めたい左タイヤの速度が入る
x_d_now 現在速度のベクトル　現在速度のベクトルに対して進行方向の角度が0であることが前提
x_d_next 次ステップの速度情報が入る
delta_t 微小時間
robot_length タイヤ間長さ
obj pixy情報
*/
void calc_v(double *v_r,double *v_l,const double x_d_now[],double x_d_next[],double delta_t,double robot_length,struct obj_type obj);
double norm(const double x[]);
double norm2(const double x[]);
double delta_theta(const double x_d_now[],const double x_d_next[]);
double delta_theta2(const double x_d_now[],const double x_d_next[]);
void calc_x_d_next(double x_d_next[],const double x_d_now[],double delta_t,struct obj_type obj);
void calc_x_dd_now(double x_dd_now[],const double x_d_now[],struct obj_type obj);
void calc_large_v(double large_v[],const double x_d_now[],struct obj_type obj);
void calc_large_v_new(double large_v[],const double x_d_now[],struct obj_type obj);
double optimal_velocity(double obj_length);
double obj_theta(struct obj_type obj);
double obj_length(struct obj_type obj);
int haritsuki_x(struct obj_type obj);
int haritsuki_y(struct obj_type obj);
int read_prms_file();

double v_r = 0;
double v_l = 0;
double x_d_now[2] = {0.01,0};
double x_d_next[2] = {0.01,0};
double delta_t = DELTA_T;
double robot_length = ROBOT_LENGTH;
int obj_visible = 0; //0 visible
double obj_visible_distance = 0;
double obj_visible_theta = 0;
int no_visible = 0;
double vs,sensitivity,alpha,beta,b,c,r;

void calc_v(double *v_r,double *v_l,const double x_d_now[],double x_d_next[],double delta_t,double robot_length,struct obj_type obj){

  //先にx_d_next計算する
  //printf("x_d_next %lf %lf\n",x_d_next[0],x_d_next[1]);
  calc_x_d_next(x_d_next,x_d_now,delta_t,obj);
  //printf("x_d_next %lf %lf\n",x_d_next[0],x_d_next[1]);

  *v_r = norm(x_d_now) + robot_length/2 * delta_theta(x_d_now,x_d_next) / delta_t;
  *v_l = norm(x_d_now) - robot_length/2 * delta_theta(x_d_now,x_d_next) / delta_t;

  //printf("xd %lf %lf xdn %lf %lf\n",x_d_now[0],x_d_now[1],x_d_next[0],x_d_next[1]);
}

double norm2(const double x[]){

  double tmp = x[0]*x[0]+x[1]*x[1];

  if(tmp > 0){
    return sqrt(x[0]*x[0]+x[1]*x[1]);
  }else if(tmp == 0){
    return 0;
  }else{
    printf("sqrt error\n");
  }
}

double norm(const double x[]){

  return sqrt(x[0] * x[0] + x[1] * x[1]);
}

double delta_theta(const double x_d_now[],const double x_d_next[]){

  //絶対角度求める
  double theta;
  double tmp = (x_d_now[0] * x_d_next[0] + x_d_now[1] * x_d_next[1]) / norm(x_d_now) / norm(x_d_next);
  if( tmp > 1){
    return 0;
  }else{
    theta = acos(tmp);
  }

  //外積ベクトルのｚ項
  double outer_z = x_d_now[0] * x_d_next[1] - x_d_now[1] * x_d_next[0];

  if(outer_z >= 0){
    return theta;
  }else{
    return -theta;
  }
}

double delta_theta2(const double x_d_now[],const double x_d_next[]){

  double angle = atan2(x_d_next[1],x_d_next[0]) - atan2(x_d_now[1],x_d_now[0]);
  return angle;
}

void calc_x_d_next(double x_d_next[],const double x_d_now[],double delta_t,struct obj_type obj){

  double x_dd_now[2];
  calc_x_dd_now(x_dd_now,x_d_now,obj);
  //printf("xdd %lf\n",x_dd_now[0]);

  x_d_next[0] = x_d_now[0] + delta_t * x_dd_now[0];
  x_d_next[1] = x_d_now[1] + delta_t * x_dd_now[1];
  //x_d_next[0] = x_d_now[0] + delta_t * 0;
  //x_d_next[1] = x_d_now[1] + delta_t * 0;
}

void calc_x_dd_now(double x_dd_now[],const double x_d_now[],struct obj_type obj){

  double large_v[2];
  calc_large_v_new(large_v,x_d_now,obj);
  //printf("largev %lf\n",large_v[0]);

  x_dd_now[0] = sensitivity * (vs * x_d_now[0] / norm(x_d_now) + large_v[0] - x_d_now[0]);
  x_dd_now[1] = sensitivity * (vs * x_d_now[1] / norm(x_d_now) + large_v[1] - x_d_now[1]);
  //printf("norm %lf\n",norm(x_d_now));
}

void calc_large_v(double large_v[],const double x_d_now[],struct obj_type obj){

  //printf("signature %d\n",obj.signature);
  if(obj.signature != 1){
    large_v[0] = 0;
    large_v[1] = 0;
    obj_visible = 1;
    return;
  }

  obj_visible = 0;
  //カメラ中心を角度0とした物体の角度と距離
  double theta = obj_theta(obj);
  double distance= obj_length(obj);

  if( r < distance ){
    large_v[0] = 0;
    large_v[1] = 0;
    return;
  }

  if( distance < OBJ_MIN_LENGTH ){
    distance = OBJ_MIN_LENGTH;
  }

  //printf("theta %lf distance %lf\n",theta,distance);
  //theta = 1;

  double n[2];

  n[0] = cos(theta);
  n[1] = sin(theta);

  large_v[0] = (1 + cos(theta)) * optimal_velocity(distance) * n[0];
  large_v[1] = (1 + cos(theta)) * optimal_velocity(distance) * n[1];
}

void calc_large_v_new(double large_v[],const double x_d_now[],struct obj_type obj){

  double theta;
  double distance;

  //printf("haritsuki %d\n",haritsuki_x(obj));
  //printf("signature %d\n",obj.signature);
  if(obj.signature == 1 && obj_length(obj) < r && no_visible == 0){
    theta = obj_theta(obj);
    distance= obj_length(obj);
    obj_visible = 0;

    if( distance < OBJ_MIN_LENGTH ){
      distance = OBJ_MIN_LENGTH;
    }

  }else{
    distance = 999;
    theta = 0;
    obj_visible = 1;
  }
  obj_visible_distance = distance;
  obj_visible_theta = theta;

  //printf("distance=%lf\n",distance);

  //カメラ中心を角度0とした物体の角度と距離
  /*
  if( distance > r ){
    large_v[0] = 0;
    large_v[1] = 0;
    return;
  }
  */

  //printf("theta %lf distance %lf\n",theta,distance);
  //theta = 1;

  double n[2];

  n[0] = cos(theta);
  n[1] = sin(theta);

  large_v[0] = (1 + cos(theta)) * optimal_velocity(distance) * n[0];
  large_v[1] = (1 + cos(theta)) * optimal_velocity(distance) * n[1];
}

double optimal_velocity(double distance){

  return alpha * (tanh(beta * (distance - b)) + c );
  //return distance * 0.1;
}

//物体中心のｘ座標から線形に求める、画角75度
double obj_theta(struct obj_type obj){
  return -(obj.x - 160) * (double)35 / 160 * M_PI / (double)180;
}

//若月君の論文データからカップまでの距離求める
double obj_length(struct obj_type obj){
  //double distance = 0.01 * (LARGE_CUP_A / pow(obj.width,LARGE_CUP_C) + LARGE_CUP_B) ;
  double distance = 0.01 * (LARGE_CUP_A_2 / pow(obj.width,LARGE_CUP_C_2) + LARGE_CUP_B_2) ;

  return distance;
}

//物体が欠けてると測定距離がおかしくなるので欠けてると見えてなーい
int haritsuki_x(struct obj_type obj){

  return ((obj.x - obj.width / 2.0) < HARITSUKI_SX) || (320 - (obj.x + obj.width / 2.0) < HARITSUKI_SX);
}

int haritsuki_y(struct obj_type obj){

  return ((obj.y - obj.height / 2.0) < HARITSUKI_SY) || (240 - (obj.y + obj.height / 2.0) <  HARITSUKI_SY);
}

int read_prms_file(){

  FILE *fp;
  fp = fopen(PRMS_FILE,"r");
  if(fp == NULL){
    printf("prms file open error\n");
    exit(0);
  }

  int i=0;
  char tmp[100];

	while(fgets(tmp,100,fp) != NULL){
    if(tmp[0] != '#'){
      //printf("%s",tmp);
      sscanf(tmp,"%lf %lf %lf %lf %lf %lf %lf\n",&vs,&sensitivity,&alpha,&beta,&b,&c,&r);
    }
	}

  printf("prms\n");
  printf("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",vs,sensitivity,alpha,beta,b,c,r);
  fclose(fp);
  return 0;
}
