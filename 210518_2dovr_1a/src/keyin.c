// functions in order to input characters from keyboard

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GAINS "gain.dat"

#define GAIN_US_MIN  2000  // 1700
#define GAIN_US_MAX  2200  // 1800
#define GAIN_DUS_MIN 10
#define GAIN_DUS_MAX 20

#define GAIN_AX_MIN 80000
#define GAIN_AX_MAX 90000
#define GAIN_GX_MIN 60000
#define GAIN_GX_MAX 70000
#define GAIN_Z_MIN 450
#define GAIN_Z_MAX 550
#define GAIN_DZ_MIN 250
#define GAIN_DZ_MAX 350


#define MOTOR_MAX 300
#define MOTOR_MIN 0
#define STEP 15

#define FRAME_MIN 0
#define FRAME_MAX 1
#define TRY_MAX 200000 // ゲインの試行回数


struct termio oldtio;

struct gain_t{
   double ax;
   double ay;
   double gx;
   double gy;
   double gz;
   double z;
   double dz;
   double us;
   double dus;
   double direc;
} gain_t;

int mpwr_from_kybd(){
   int mpwr;

   char ch='\0';

   //Read value of motor power form keyboard
   read(0,&ch,1);

   switch(ch){
     case 'h':
         mpwr=-STEP;
         break;
     case 'j':
         mpwr=-1;
         break;
     case 'k':
         mpwr=+1;
         break;
     case 'l':
         mpwr=+STEP;
         break;
     case '0':
         mpwr=0;
         break;
     case 'q':
         mpwr=-999;
         break;
     default:
         mpwr=0;
         break;
   }

   return mpwr;
}

int keyin(){

   int  val=-1;
   char key[1];

   key[0]='i';

   while((strcmp(key,"c")!=0)&&(strcmp(key,"q")!=0)){
      printf("\n");
      //printf(" *** Please select from following choice. **** \n");
      //printf("     Continue      : c \n");
      //printf("     Quit          : q \n");
      //printf(" ********************************************* \n");
      printf("Please Input Key c:continue q:quit\n");
      scanf("%s",key);
      //printf("%s\n",key);
   }

   switch(key[0]){
      case 'c':
         val=0;
         break;
      case 'q':
         val=-999;
         break;
   }

   return val;
}


void init_keyboard(void){

  struct termio newtio; //変更後の設定を保持

  //最初に現在の設定を退避します
  ioctl(0, TCGETA, &oldtio);
  newtio = oldtio;

  //設定を変更し、反映させます
  newtio.c_lflag &= ~(ECHO | ICANON); //エコーを止め、RAW モードへ変更
  newtio.c_cc[VMIN] = 0; //0文字入力された時点で入力を受け取る
  newtio.c_cc[VTIME] = 0; //1のとき、何も入力がない場合、1/10秒待つ設定となる
  ioctl(0, TCSETAF, &newtio); //ここで設定を反映
}

void final_keyboard(){
  ioctl(0, TCSETAF, &oldtio); //ここで設定を反映
}

double PeriodicBoundary(double x){
   double period_max;
   double period_min;

   period_max=FRAME_MAX;
   period_min=FRAME_MIN;

   if(x>period_max){
      x=x-period_max;
   }
   if(x<period_min){
      x=x+period_max;
   }
   return x;
}// end of period function

// ==== Generate new gains by the Self-Avoiding Walk algorithm from old gain ====
struct gain_t new_gain(struct gain_t gain){

   char   line[256];
   double x,y;
   double gain_a, gain_g, gain_gz, gain_z, gain_dz;
   double gain_us, gain_dus, gain_direc;
   double sensitiv;
   double x_mmry[TRY_MAX];
   double y_mmry[TRY_MAX];
   double dx,dy;
   double d2;
   double new_x,new_y;
   double r;
   double max;
   double step;

   int avoid;
   int keep_height, normal_throttle;
   int i,j;
   int hstry;
   int crush;
   FILE *fp_gain;

   max=(double)RAND_MAX;
   step=0.5000; // 試行ステップ幅
   r=0.70000; // 排除体積の半径 [0 -- 1]


   fp_gain=fopen(GAINS,"r");

   fgets(line,sizeof(line),fp_gain);
   printf("%s",line);
   j=0;
   while(fgets(line,sizeof(line),fp_gain)!=NULL){
      printf("%s",line);
      sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf %d %lf %d %d",
         &gain_a,&gain_g,&gain_gz,&gain_z,&gain_dz,&gain_us,&gain_dus,&gain_direc,&avoid,&sensitiv,&keep_height,&normal_throttle);
      // ++++ Set gains to SAW ++++
      x_mmry[j]=gain_us;         //
      y_mmry[j]=gain_dus;        //
      // ++++++++++++++++++++++++++
      j++;
   }

   fclose(fp_gain);

   hstry=j;
   printf("hstry=%d\n",hstry);

   // ++++ Set old gains as starting point ++++++++++++++
   x=(gain.us-GAIN_US_MIN)/(GAIN_US_MAX-GAIN_US_MIN);     //
   y=(gain.dus-GAIN_DUS_MIN)/(GAIN_DUS_MAX-GAIN_DUS_MIN);  //
   // +++++++++++++++++++++++++++++++++++++++++++++++++++

   dx=step*(rand()-0.5*max)/max;
   dy=step*(rand()-0.5*max)/max;
   //printf("dx:dy=%lf:%lf\n",dx,dy);
   new_x=PeriodicBoundary(x+dx);
   new_y=PeriodicBoundary(y+dy);
   j=0; crush=0;
   while(j<hstry){
      d2=(new_x-x_mmry[j])*(new_x-x_mmry[j])+(new_y-y_mmry[j])*(new_y-y_mmry[j]);
      // Self Avoiding
      if(d2<4.0*r*r){ crush=1; }
      j++;
   }// end of while(j)
   if(crush==0){
      hstry++;
      x=(new_x);
      y=(new_y);
      x_mmry[hstry]=x;
      y_mmry[hstry]=y;
   }// end of if(crush)

   // ++++ Set new gains to return ++++++++++++++++++++++++++
   gain.us=GAIN_US_MIN + new_x*(GAIN_US_MAX-GAIN_US_MIN);     //
   gain.dus=GAIN_DUS_MIN + new_y*(GAIN_DUS_MAX-GAIN_DUS_MIN);  //
   // +++++++++++++++++++++++++++++++++++++++++++++++++++++++

   gain.ay=gain.ax;
   gain.gy=gain.gx;

   return gain;

}
