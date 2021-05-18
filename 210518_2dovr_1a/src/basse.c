#include <time.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <pthread.h>
#include "pvm_4frobo.c"
#include "keyin.c"

// =========================================================

#define BASSE_PERIOD 50000 // 20Hz , 1000000: 1Hz
#define THROTTLE_MAX 300

#define NOMURA_PY "sub_data/nomura_files/recieve_readfile2.py"
#define NOMURA_MARKERS "sub_data/nomura_files/recieve_markerset.txt"

#define DATA_OUTPUT "/tmp/basse.dat"
// # mount -t tmpfs -o size=128m tmpfs /tmp を実行しておくと，/tmpがramdiskになって高速
#define GYRO_FACTOR 0.3 // is multiplied to gyro value to calculate angl2

struct threed_type norm_accel(struct threed_type, struct threed_type);
struct threed_type norm_gyro(struct threed_type, struct threed_type);


int main(int argc, char **argv){

   char line[256];
   double now,start;
   int avoid;
   int keep_height;
   int flag_height;
   int normal_throttle;
   int i;
   int q;
   int diff=0;
   int keyval=0;
   int tids[3];
   int info;
   int throttle;
   int mesure_count;
   FILE *fp,*fp_gain;

   double angl2;
   double sensitiv;
   double data[20];
   double tmp_data[10][20];
   struct gain_t gain;

   time_t timer;

   double timespend;
   double starttime;

   srand(783173);

   //Get Command Line Option
   int opt;
   int longindex;
   int opt_a = 0;
   int opt_c = 0;
   int opt_p = 0;
   int opt_x = 0;
   int opt_n = 0;
   int opt_s = 0;
   int opt_v = 0;
   int opt_h = 0;
   int opt_i = 0;
   char *opt_a_param = NULL;
   char *opt_c_param = NULL;
   char *opt_p_param = NULL;
   char *opt_i_param = NULL;
   struct option long_opts[] = {
     { "version", no_argument, NULL, 'v'},
     { "help", no_argument, NULL, 'h'},
     { "nametag", required_argument, NULL, 'c'},
     { "myip" , required_argument, NULL, 'p'},
     { "novisible", no_argument, NULL, 'n'},
     { "noservo", no_argument, NULL, 's'},
     { "nofile", no_argument, NULL, 'x'},
     { "alert", required_argument, NULL, 'a'}
   };
   while((opt = getopt_long(argc,argv,"xc:p:a:i:",long_opts,&longindex)) != -1){
     switch(opt){
       case 'a':
       opt_a = 1;
       opt_a_param = optarg;
       break;
       case 'c':
       opt_c = 1;
       opt_c_param = optarg;
       break;
       case 'p':
       opt_p = 1;
       opt_p_param = optarg;
       break;
       case 'x':
       opt_x = 1;
       break;
       case 'n':
       opt_n = 1;
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
       case 'i':
       opt_i = 1;
       opt_i_param = optarg;
       hostone_on = 1;
       strcpy(hostone,optarg);
       break;
     }
   }

   fp=fopen(DATA_OUTPUT,"a");
   /*
   if((fp_gain=fopen(GAINS,"r"))==NULL){
      printf("GAINS open error!!\n");
      exit(EXIT_FAILURE);
   }
   */

   // For making result filename
   FILE *fpxy;

   time(&timer);
   struct tm *local = localtime(&timer);
   char filename[100];
   sprintf(filename,"/tmp/%04d%02d%02d_%s_bassedat.xy",local->tm_year + 1900,\
                                      local->tm_mon + 1,\
                                      local->tm_mday,\
                                      opt_c_param);
   fpxy=fopen(filename,"w");


  /*
   while(fgets(line,sizeof(line),fp_gain)!=NULL){
      printf("%s",line);


   }
   sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf %d %lf %d %d",
      &gain.ax,&gain.gx,&gain.gz,&gain.z,&gain.dz,&gain.us,&gain.dus,&gain.direc,&avoid,&sensitiv,&keep_height,&normal_throttle);
   printf("avoid.dist =%d\n",avoid);

   gain.ay=gain.ax;
   gain.gy=gain.gx;
   fclose(fp_gain);
   fp_gain=fopen(GAINS,"a");
   */


   setlinebuf(stdout);
   signal(SIGINT,sigcatch);

   //printf("XX===========================================XX\n");
   //printf("X       Basse #2dOVR_B1 since 2016 12/9      X\n");
   //printf("XX=============================== (^.^)/ ====XX\n");
   printf("Starting Basse for 2dovr.\n");

   //load hosts
   read_hosts_file();

   pvm_init_4basse_multiple(argc,argv);  // haute is spawned in this init. function
   printf("PVM Init finish.\n");


   // Waiting for haute recv preparation
   printf("receiving fromt start signal from hautes.\n");
   while(pvm_recv_from_haute_to_basse_multiple_check(data,1)==0){}
   printf("Start signal is recieved from hautes.\n");

   keyval=keyin();
   data[0]=(double)keyval;
   pvm_send_from_basse_to_haute_multiple(data,1);

   //**** Start Nomura PY *****
   if(opt_p == 1){
     printf("Starting Nomura PY\n");
     char execute[200];
     sprintf(execute,"python %s %s %s %s &",NOMURA_PY,opt_c_param,opt_p_param,NOMURA_MARKERS);
     printf("%s\n",execute);
     system(execute);
   }
   //**** End *****

   while(keyval!=-999){

      starttime = gettimeofday_sec();
      //gain=new_gain(gain);

      //printf("======================\n");
      //printf("sensitiv =%lf\n",sensitiv);
      //printf("normal.throttle =%d\n",normal_throttle);
      //printf("======================\n");

      //data[0]=sensitiv;
      //data[1]=normal_throttle;

      pvm_send_from_basse_to_haute_multiple(data,2);
      //printf("[[ Please waite seconds to initialize SRF02. Thank you. ]]\n\n");

      init_keyboard();

      angl2=0.0;
      flag_height=0;
      start=gettimeofday_sec();
      mesure_count=0;
      throttle=0;
      diff=1;

      //usleep(2000000); // haute@bbbXXw でのタイムスタンプと秒を同期するため必要
      time(&timer);
      fprintf(fp,"\n# %s",ctime(&timer));
      fprintf(fp,"#sec ");
      fprintf(fp,"pulsewidth_r ");
      fprintf(fp,"pulsewidth_l ");
      fprintf(fp,"v_r ");
      fprintf(fp,"v_l ");
      fprintf(fp,"obj.x ");
      fprintf(fp,"obj_theta ");
      fprintf(fp,"obj_width ");
      fprintf(fp,"obj_distance ");
      fprintf(fp,"obj_visible\n");

      fprintf(fpxy,"#sec ");
      fprintf(fpxy,"pulsewidth_r ");
      fprintf(fpxy,"pulsewidth_l ");
      fprintf(fpxy,"v_r ");
      fprintf(fpxy,"v_l ");
      fprintf(fpxy,"obj.x ");
      fprintf(fpxy,"obj_theta ");
      fprintf(fpxy,"obj_width ");
      fprintf(fpxy,"obj_distance ");
      fprintf(fpxy,"obj_visible\n");

      //printf("[[ throttle : 0 -- 500 ]]\n\n");
      for(i=0;i<num_of_hautes;i++){
        printf("\n");
      }

      while(throttle>=0){

         if(diff==-999){
            throttle=-999; // to break this loop
            data[0]=(double)diff;
            pvm_send_from_basse_to_haute_multiple(data,1);
         }
         else if((throttle==0)&&(diff<0)){
            data[0]=0.0;
            pvm_send_from_basse_to_haute_multiple(data,1);
         }
         else if((throttle>=THROTTLE_MAX)&&(diff>0)){
            data[0]=0.0;
            pvm_send_from_basse_to_haute_multiple(data,1);
         }
         else{
            throttle+=diff;
            data[0]=(double)diff;
            pvm_send_from_basse_to_haute_multiple(data,1);
         }
         diff=mpwr_from_kybd();

         timespend = gettimeofday_sec() - starttime;
         printf("\033[42m"); // green background
         for(i=0;i<num_of_hautes;i++){
           printf("%6s %6.3lf %6.3lf %6.3lf %6.3lf",hautes[i],tmp_data[i][2],tmp_data[i][3],tmp_data[i][5],tmp_data[i][7]);
           printf("\033[%dA" ,1); // move one line upward
           printf("\r");
         }
         //printf("\033[%dA" ,1); // move one line upward
         printf("\r");
         printf("\033[44m"); // blue background
         printf("time %d min %.2lf sec",(int)timespend / 60,timespend - ((int)timespend / 60)*60 );
         printf("\033[%dA" ,1); // move one line upward
         printf("\r");
         printf("\033[41m"); // red background
         printf("\n");
         for(i=0;i<num_of_hautes;i++){
           printf("\n");
         }


         if( opt_a == 1 && timespend > atof(opt_a_param)){
           system("./alert.sh > /dev/null 2>&1 &");
           //puts("alert\n");
           opt_a = 0;
         }


         //Recieve from haute data
         if(pvm_recv_from_haute_to_basse_multiple_data(tmp_data,9)>0){
            now=gettimeofday_sec();

            for(i=0;i<num_of_hautes;i++){
              fprintf(fp,"%10s ",hautes[i]);
              fprintf(fp,"%8.3f ",now-start);
              fprintf(fp,"%8.3f ",tmp_data[i][0]);
              fprintf(fp,"%8.3f ",tmp_data[i][1]);
              fprintf(fp,"%8.3f ",tmp_data[i][2]);
              fprintf(fp,"%8.3f ",tmp_data[i][3]);
              fprintf(fp,"%8.3f ",tmp_data[i][4]);
              fprintf(fp,"%9.3f ",tmp_data[i][5]);
              fprintf(fp,"%9.3f ",tmp_data[i][6]);
              fprintf(fp,"%9.3f ",tmp_data[i][7]);
              fprintf(fp,"%9.3f ",tmp_data[i][8]);
              fprintf(fp,"\n");
              fflush(fp);
            }

            for(i=0;i<num_of_hautes;i++){
              fprintf(fpxy,"%s ",hautes[i]);
              fprintf(fpxy,"%.3f ",now-start);
              fprintf(fpxy,"%d ",(int)tmp_data[i][0]);
              fprintf(fpxy,"%d ",(int)tmp_data[i][1]);
              fprintf(fpxy,"%.3f ",tmp_data[i][2]);
              fprintf(fpxy,"%.3f ",tmp_data[i][3]);
              fprintf(fpxy,"%d ",(int)tmp_data[i][4]);
              fprintf(fpxy,"%.3f ",tmp_data[i][5]);
              fprintf(fpxy,"%d ",(int)tmp_data[i][6]);
              fprintf(fpxy,"%.3f ",tmp_data[i][7]);
              fprintf(fpxy,"%d ",(int)tmp_data[i][8]);
              fprintf(fpxy,"\n");
              //fflush(fp);
            }

            if((throttle>normal_throttle)&&(flag_height==1)){
               mesure_count++;
               angl2+=data[0]*data[0]+data[1]*data[1];
               angl2+=GYRO_FACTOR*GYRO_FACTOR*data[2]*data[2];
               angl2+=GYRO_FACTOR*GYRO_FACTOR*data[3]*data[3];
            }
         }


         usleep(BASSE_PERIOD); // this length of sleep determines period of reflesh diff
      } // end of throttle loop
      now=gettimeofday_sec();


      final_keyboard();
      printf("\033[49m"); // normal background

      while(pvm_recv_from_haute_to_basse_multiple_check(data,1)==0){}

//      if(mesure_count>0){

        /*
         angl2=angl2/((int)mesure_count);
         fprintf(fp_gain,"%8.1f ",gain.ax);
         fprintf(fp_gain,"%7.1f ",gain.gx);
         fprintf(fp_gain,"%7.1f ",gain.gz);
         fprintf(fp_gain,"%6.1f ",gain.z);
         fprintf(fp_gain,"%5.1f ",gain.dz);
         fprintf(fp_gain,"%6.1f ",gain.us);
         fprintf(fp_gain,"%4.1f ",gain.dus);
         fprintf(fp_gain,"%5.1f ",gain.direc);
         fprintf(fp_gain,"%4d ",avoid);
         fprintf(fp_gain,"%6.4f ",sensitiv);
         fprintf(fp_gain,"%3d ",keep_height);
         fprintf(fp_gain,"%3d ",normal_throttle);
         printf("angl2=%lf\n",angl2);
         fprintf(fp_gain,"%5.1f ",data[0]);
         fprintf(fp_gain,"%s",ctime(&timer));
         fflush(fp_gain);
         */

 //     }

      keyval=keyin();
      data[0]=(double)keyval;
      pvm_send_from_basse_to_haute_multiple(data,1);

   } // end of keyval loop

   sleep(2);
   pvm_kill_hautes();

   fclose(fp);
   //fclose(fp_gain);
   fclose(fpxy);

   if(opt_x == 1){
     int result = remove(filename);
     //printf("result=%d\n",result);
   }

   if(opt_p == 1){

     int nomura = system("ps -aux | grep nomura | grep -v grep | tr -s \" \" | cut -d \" \" -f 2");
     char kill[100];
     sprintf(kill,"kill -s 2 %d",nomura);
     system(kill);
   }

   return;
 } // END of MAIN
