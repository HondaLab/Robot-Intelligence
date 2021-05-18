#!/usr/bin/python3

# motor1.py
# 2018-11-5
# Yasushi Honda

import pigpio
import time

MIN_WIDTH=1000
MID_WIDTH=1500
MAX_WIDTH=2000

class Motor:

   def __init__(self,gpio):
      self.gpio=gpio
      self.pi = pigpio.pi()
      if not self.pi.connected:
         exit()
      self.pi.set_servo_pulsewidth(gpio, MID_WIDTH)
      self.SLEEP=0.01
      #time.sleep(0.1)

   def move(self,power):
      self.pi.set_servo_pulsewidth(self.gpio, MID_WIDTH+power)
      time.sleep(self.SLEEP)    

   def stop(self):
      self.pi.stop()

class Lmotor(Motor):
   def run(self,power):
      self.move(-power*5)

class Rmotor(Motor):
   def run(self,power):
      self.move(power*5)
       
if __name__=='__main__':

   # モーター出力最小値と最大値
   MIN=-100
   MAX=100

   # rate調整用の待ち時間(秒)
   SLEEP=0.02

   # LeftモーターをGPIO=18に、Rightモーターを17につなぐ。
   motorL = Lmotor(18)
   motorR = Rmotor(17)
   
   # 停止の仕方のメッセージ表示
   print("Robot runs only 1sec.")

   # 左右のモーター出力をゼロに初期化する。
   motorL.run(0)
   motorR.run(0)
   time.sleep(1)

   # 各変数の初期化
   left=0
   right=0
   start=time.time()
   now=start
   # 1秒間だけモーターを回す
   while now-start<1 :
      now=time.time() 

      left=-20
      if left<MIN or left>MAX:
            left = MIN
      motorL.run(left)

      right=20
      if right<MIN or right>MAX:
            right = MIN
      motorR.run(right)

      # 更新をSLEEP秒だけ待って、rateを調節する。
      time.sleep(SLEEP)

   # モーター出力をゼロにもどして止める
   motorL.run(0)
   motorR.run(0)
   motorL.stop()
   motorR.stop()
