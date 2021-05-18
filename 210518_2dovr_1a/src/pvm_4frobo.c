// Feeler by gpio 2016 12/17
// frb16.4.4(2016.2.1)
// v16.0.4(2015.1.11)hauteとsrf02通信関数
// ver.11.2.1 2014 5/29
// pvm for frobot (Since 2014. 1/19 Yasushi Honda)
// 2017/10/10 miyajima

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <pvm3.h>

#define BASSE  "" // 基地 Debian PC
#define HAUTE  ""   // 2dovrロボット BBB
#define OPEDEV ""         // 操作機
#define SRF02  "" // 超音波センサ(SRF02)を動かすマシン（ロボット）

#define PROGRAM_BASSE  "/tmp/basse"
#define PROGRAM_HAUTE  "/tmp/2dovr"
#define PROGRAM_OPEDEV ""
#define PROGRAM_SRF02 "/tmp/srf02_pvm"
#define PROGRAM_PIXY "/tmp/pixy"
#define PROGRAM_FEELER "/tmp/feeler"

// pvm TAG number to send data by fixing number for EASY
#define FROM_BASSE_TO_HAUTE_TAG   0
#define FROM_BASSE_TO_OPEDEV_TAG  1
#define FROM_OPEDEV_TO_BASSE_TAG  2
#define FROM_OPEDEV_TO_HAUTE_TAG  3
#define FROM_HAUTE_TO_BASSE_TAG   4
#define FROM_HAUTE_TO_OPEDEV_TAG  5
#define FROM_SRF02_TO_HAUTE_TAG   6
#define FROM_PIXY_TO_HAUTE_TAG    7
#define FROM_HAUTE_TO_PIXY_TAG    8
#define FROM_FEELER_TO_HAUTE_TAG  9

#define DNUM_BETWEEN_HAUTE_AND_SRF02 10

#define HOSTS_FILE "hosts"

struct obj_type{
   int signature;
   int x;
   int y;
   int width;
   int height;
}obj_type;

void sigcatch(int); // CTRL-Cで実行される関数

// tids as grobal variables
int basse_tid;
int haute_tid;
int opedev_tid;
int srf02_tid;
int pixy_tid;
int feeler_tid;

char hautes[10][20];
int haute_tids[10];
int num_of_hautes = 0;
char hostone_on = 0;
char hostone[10];

int read_hosts_file(){

  if(hostone_on == 1){
    strcpy(hautes[0],hostone);
    num_of_hautes = 1;
    return 0;
  }

  FILE *fp;
  fp = fopen(HOSTS_FILE,"r");
  if(fp == NULL){
    printf("hosts file open error\n");
    exit(0);
  }

  int i=0;
  char tmp[20];

  while(feof(fp) == 0){
		fscanf(fp,"%s\n",tmp);
    if(tmp[0] != '#'){
      strcpy(hautes[i],tmp);
      i++;
    }
	}
  num_of_hautes = i;

  fclose(fp);
  return 0;
}

// Time evaluation (In serial_accelerometer)
//double gettimeofday_sec();

// ------------------------------------------
int pvm_init_4basse(int argc, char **argv){
   int i;
   int info;
   int nhost, narch, infos;
   int tids[3];
   struct pvmhostinfo *hostp;

   char test[10];

   info = pvm_spawn(PROGRAM_HAUTE, argv, 1, HAUTE, 1, tids);
   printf("%d\n",info);
   haute_tid=tids[0];
   printf("%s is spawned.\n",PROGRAM_HAUTE);
   printf("tids[0]=%d\n",tids[0]);

   basse_tid=pvm_mytid();
   printf("my_tid(basse)=%d\n",basse_tid);

   info=pvm_config(&nhost, &narch, &hostp);
   for(i=0;i<nhost;i++){
      printf("host[%d]=%s\n",i,hostp[i].hi_name);
      printf("tid[%d]=%d\n",i,hostp[i].hi_tid);
   }
   printf("basse_tid=%d\n",basse_tid);
   printf("opedev_tid=%d\n",opedev_tid);
   printf("haute_tid=%d\n",haute_tid);

   return 0;
} // end of pvm_init_4basse

int pvm_init_4basse_multiple(int argc, char **argv){
   int i;
   int info;
   int nhost, narch, infos;
   int tids[3];
   struct pvmhostinfo *hostp;

   for(i=0;i<num_of_hautes;i++){
     info = pvm_spawn(PROGRAM_HAUTE, argv, 1, hautes[i], 1, tids);
     haute_tids[i] = tids[0];
     if(info > 0){
       printf("%s is spawned.\n",hautes[i]);
     }else{
       printf("%s is not spawned.\n",hautes[i]);
     }
   }

   //printf("num_of_hautes=%d\n",num_of_hautes);

   basse_tid=pvm_mytid();
   //printf("my_tid(basse)=%d\n",basse_tid);

   /*
   info=pvm_config(&nhost, &narch, &hostp);
   for(i=0;i<nhost;i++){
      printf("host[%d]=%s\n",i,hostp[i].hi_name);
      printf("tid[%d]=%d\n",i,hostp[i].hi_tid);
   }
   */

   //printf("basse_tid=%d\n",basse_tid);
   //printf("opedev_tid=%d\n",opedev_tid);
   //printf("haute_tid=%d\n",haute_tid);

   return 0;
} // end of pvm_init_4basse

int pvm_init_4haute(void){
   int tids[3];

   setlinebuf(stdout);
   basse_tid=pvm_parent();
   haute_tid=pvm_mytid();

   return 0;
}// end of pvm_init_4haute

int pvm_spawn_srf02(void){
   int tids[3];

   setlinebuf(stdout);

   pvm_spawn(PROGRAM_SRF02, (char**)0, 1, HAUTE, 1, tids);
   srf02_tid=tids[0];
   printf("%s is spawned.\n",PROGRAM_SRF02);
   //printf("tids[0]=%d\n",tids[0]);

   return 0;
}// end of pvm_spawn_srf02

int pvm_spawn_pixy(void){
   int tids[3];
   unsigned char myhostname[20];
   gethostname(myhostname,sizeof(myhostname));

   setlinebuf(stdout);

   pvm_spawn(PROGRAM_PIXY, (char**)0, 1, myhostname, 1, tids);
   pixy_tid=tids[0];
   printf("%s is spawned.\n",PROGRAM_PIXY);

   return 0;
}// end of pvm_spawn_pixy

int pvm_spawn_feeler(void){
   int tids[3];

   setlinebuf(stdout);

   pvm_spawn(PROGRAM_FEELER, (char**)0, 1, HAUTE, 1, tids);
   feeler_tid=tids[0];
   printf("%s is spawned.\n",PROGRAM_FEELER);

   return 0;
}// end of pvm_spawn_feeler

int pvm_init_4opedev(void){
   setlinebuf(stdout);
   haute_tid=pvm_parent();
   opedev_tid=pvm_mytid();

   return 0;
}// end of pvm_init_4opedev

int pvm_init_4srf02(void){
   setlinebuf(stdout);
   haute_tid=pvm_parent();
   srf02_tid=pvm_mytid();

   return 0;
}// end of pvm_init_4opedev

int pvm_init_4pixy(void){
   setlinebuf(stdout);
   haute_tid=pvm_parent();
   pixy_tid=pvm_mytid();

   return 0;
}// end of pvm_init_4pixy

/*
// Between feeler and haute
int pvm_send_from_feeler_to_haute(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(haute_tid,FROM_FEELER_TO_HAUTE_TAG);
   return info;
}
int pvm_recv_from_feeler_to_haute(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(feeler_tid,FROM_FEELER_TO_HAUTE_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}
*/

// Between basse and haute
int pvm_send_from_basse_to_haute(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(haute_tid,FROM_BASSE_TO_HAUTE_TAG);
   return info;
}

int pvm_send_from_basse_to_haute_multiple(double *data, int size){
   int info;
   int i;

   for(i=0;i<num_of_hautes;i++){
     pvm_initsend(0);
     pvm_pkdouble(data,size,1);
     info=pvm_send(haute_tids[i],FROM_BASSE_TO_HAUTE_TAG);
   }

   return 0;
}

int pvm_send_from_haute_to_basse(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(basse_tid,FROM_HAUTE_TO_BASSE_TAG);
   return info;
}

int pvm_recv_from_haute_to_basse(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(haute_tid,FROM_HAUTE_TO_BASSE_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}

int pvm_recv_from_haute_to_basse_multiple_check(double *data, int size){
   int bufid;
   int i;

   for(i=0;i<num_of_hautes;i++){
     bufid=pvm_recv(haute_tids[i],FROM_HAUTE_TO_BASSE_TAG);
     pvm_upkdouble(data,size,1);
   }

   return bufid;
}

int pvm_recv_from_haute_to_basse_multiple_data(double tmp_data[][20], int size){
   int bufid;
   int i;

   for(i=0;i<num_of_hautes;i++){
     bufid=pvm_nrecv(haute_tids[i],FROM_HAUTE_TO_BASSE_TAG);
     if(bufid>0){
        pvm_upkdouble(tmp_data[i],size,1);
     }
   }

   return 1;
}

int pvm_recv_from_basse_to_haute(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(basse_tid,FROM_BASSE_TO_HAUTE_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}

/*
// Between opedev and haute
int pvm_send_from_opedev_to_haute(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(haute_tid,FROM_OPEDEV_TO_HAUTE_TAG);
   return info;
}

int pvm_send_from_haute_to_opedev(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(opedev_tid,FROM_HAUTE_TO_OPEDEV_TAG);
   return info;
}

int pvm_recv_from_opedev_to_haute(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(opedev_tid,FROM_OPEDEV_TO_HAUTE_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}

int pvm_recv_from_haute_to_opedev(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(haute_tid,FROM_HAUTE_TO_OPEDEV_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}


// Between opedev and basse
int pvm_send_from_opedev_to_basse(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(basse_tid,FROM_OPEDEV_TO_BASSE_TAG);
   return info;
}

int pvm_send_from_basse_to_opedev(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(opedev_tid,FROM_BASSE_TO_OPEDEV_TAG);
   return info;
}

int pvm_recv_from_opedev_to_basse(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(opedev_tid,FROM_OPEDEV_TO_BASSE_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}

int pvm_recv_from_basse_to_opedev(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(basse_tid,FROM_BASSE_TO_OPEDEV_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}

int pvm_send_from_srf02_to_haute(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(haute_tid,FROM_SRF02_TO_HAUTE_TAG);
   return info;
}

int pvm_recv_from_srf02_to_haute(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(srf02_tid,FROM_SRF02_TO_HAUTE_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}
*/

int pvm_send_from_pixy_to_haute(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(haute_tid,FROM_PIXY_TO_HAUTE_TAG);
   return info;
}

int pvm_recv_from_pixy_to_haute(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(pixy_tid,FROM_PIXY_TO_HAUTE_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}

int pvm_send_from_haute_to_pixy(double *data, int size){
   int info;
   pvm_initsend(0);
   pvm_pkdouble(data,size,1);
   info=pvm_send(pixy_tid,FROM_HAUTE_TO_PIXY_TAG);
   return info;
}

int pvm_recv_from_haute_to_pixy(double *data, int size){
   int bufid;
   bufid=pvm_nrecv(haute_tid,FROM_HAUTE_TO_PIXY_TAG);
   if(bufid>0){
      pvm_upkdouble(data,size,1);
   }
   return bufid;
}

//kill hautes
int pvm_kill_hautes(){

  int i;
  for(i=0;i<num_of_hautes;i++){
    pvm_kill(haute_tids[i]);
  }
  return 0;
}

void sigcatch(int signal){
   printf("quit %d\n",opedev_tid);
   pvm_kill(opedev_tid);
   pvm_kill(srf02_tid);
   exit(1);
}

// Time evaluation (In serial_accelerometer)
double gettimeofday_sec(){
   struct timeval tv;
   gettimeofday(&tv,NULL);
   return ((double)tv.tv_sec + (double)tv.tv_usec*0.001*0.001);
}
