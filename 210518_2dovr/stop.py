#!/usr/bin/python3

# motor1.py
# 2018-11-5
# Yasushi Honda

import pigpio
import time

MID_WIDTH = 1500

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
      self.move(-5*power)

class Rmotor(Motor):
   def run(self,power):
      self.move(5*power)
       
if __name__=='__main__':

   # LeftモーターをGPIO=18に、Rightモーターを17につなぐ。
   motorL = Lmotor(18)
   motorR = Rmotor(17)
   
   # 左右のモーター出力をゼロに初期化する。
   motorL.run(0)
   motorR.run(0)

