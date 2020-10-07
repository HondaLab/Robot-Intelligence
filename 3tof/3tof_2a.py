#!/usr/bin/python3
# CC BY-SA Yasushi Honda 2020 5/16

import modules.motor2 as mt # モーターを回転させるためのモジュール
import modules.vl53_4a as lidar
import modules.keyin as keyboard
import time
import math

PERIOD=0.5

mL=mt.Lmotor(17) # 左モーター(gpio17番)
mR=mt.Rmotor(18) # 右モーター(gpio18番)

tofR,tofL,tofC=lidar.start()
key=keyboard.Keyboard()

def tanh1(x):
    alpha=30.0
    alpha2=30.0
    beta=0.001 # 0.004
    beta2=10.00
    b=200  # 280
    c=0.0
    f=alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c
    return f
def tanh2(x):
    alpha=30.0
    alpha2=30.0
    beta=0.001 # 0.004
    beta2=10.00
    b=280  # 360
    c=0.0
    f=alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c
    return f

print("===============================")
print("Input 'q' to stop this program.")
print("===============================")
ch="c"
count=0
gamma=0.32 # Center weight
now=time.time()
start=now
init=now
print("  time rate distL distC distR areaL areaR   pwL    pwR")
while ch!="q":
    try:
        distanceL=tofL.get_distance()
        if distanceL>2000:
           distanceL=2000
        #time.sleep(0.02)
        distanceC=tofC.get_distance()
        if distanceC>2000:
           distanceC=2000
        #time.sleep(0.02)
        distanceR=tofR.get_distance()
        if distanceR>2000:
           distanceR=2000
        count+=1

        #areaL=math.sqrt(distanceL*distanceC)
        #areaR=math.sqrt(distanceR*distanceC)
        if distanceL>0 and distanceC>0:
           areaL=math.exp(gamma*math.log(distanceC))*math.exp((1-gamma)*math.log(distanceL))
        if distanceR>0 and distanceC>0:
           areaR=math.exp(gamma*math.log(distanceC))*math.exp((1-gamma)*math.log(distanceR))
    
        # Double tanh 非線形感覚運動写像
        powerL=tanh2(areaR)
        powerR=tanh1(areaL)
        

    except KeyboardInterrupt: #例外処理でプログラムを止める
        break

    now=time.time()
    if now-start>PERIOD:
       rate=count/PERIOD
       print("\r %5.1f" % (now-init),end='')
       print("%5.1f" % rate,end='')
       print(" %5d" % distanceL,end='')
       print(" %5d" % distanceC,end='')
       print(" %5d" % distanceR,end='') 
       print(" %5d" % areaL,end='')
       print(" %5d" % areaR,end='')
       print(" %5.2f" % powerL,end='')
       print(" %5.2f" % powerR,end='')
       count=0
       start=now

    mL.run(powerL)
    mR.run(powerR)
    time.sleep(0.013)
   
    ch=key.read()

print("\n Bye-bye")
mL.run(0)
mR.run(0)
