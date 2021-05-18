#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>


int main(int argc,char **argv){

  if(argc < 3){
    printf("gpioport pulsewidth clockinterval\n");
    exit(0);
  }

  gpioCfgClock(1,1,0); //ideal clock period is 1us
  gpioCfgBufferSize(200); //buffer size
  gpioInitialise();

  int gpio = atoi(argv[1]);
  int pulse = atoi(argv[2]);
  int clk = atoi(argv[3]);

  char c = 'r';
  char gomi;

  while(c != 'q'){
    printf("%d\n",pulse);
    gpioServo(gpio,pulse);
    scanf("%c",&c); //return code
    scanf("%c",&gomi);
    if(c == '[') pulse += clk;
    if(c == ']') pulse -= clk;
  }

  gpioTerminate();

  return 0;
}
