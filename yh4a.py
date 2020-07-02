#!/usr/bin/python3
# CC BY-SA Yasushi Honda 2020 5/16

import modules.motor as mt # モーターを回転させるためのモジュール
import modules.vl53_3a as lidar
import modules.keyin as keyboard
import time
import math

mL=mt.Lmotor(17) # 左モーター(gpio17番)
mR=mt.Rmotor(18) # 右モーター(gpio18番)

tofR,tofL=lidar.start()
key=keyboard.Keyboard()

def tanh1(x):
    alpha=6.0
    alpha2=alpha*0.7
    beta=0.008
    beta2=10.00
    b=200
    c=0.0*(alpha+alpha2)
    f=alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c
    return f
def tanh2(x):
    alpha=6.0
    alpha2=alpha*0.7
    beta=0.008
    beta2=10.00
    b=180
    c=0.0*(alpha+alpha2)
    f=alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c
    return f

print("===============================")
print("Input 'q' to stop this program.")
print("===============================")
ch="c"
while ch!="q":
    try:
        distanceR=tofR.get_distance()
        time.sleep(0.02)
        distanceL=tofL.get_distance()
    
        # Double tanh 非線形感覚運動写像
        powerL=tanh2(distanceR)
        powerR=tanh1(distanceL)
        
        mL.run(powerL)
        mR.run(powerR)

        print("\r %5d %5d %5.2f %5.2f" % (distanceL,distanceR,powerL,powerR),end="")
        time.sleep(0.10)
        
    except KeyboardInterrupt: #例外処理でプログラムを止める
        break
       
    ch=key.read()

print("\n Bye-bye")
mL.run(0)
mR.run(0)
