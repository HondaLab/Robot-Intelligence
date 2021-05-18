#!/usr/bin/python3
# for 3 tofs
# vl53_4a.py  Yasushi Honda 2020 8.7
# This modeule code is to access trible vl53l0x connected GPIO of RasPi

# MIT License
# 
# Copyright (c) 2017 John Bryan Moore
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell # copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import time
import modules.VL53L0X as vl53
import RPi.GPIO as GPIO

def start():

   # GPIO for Sensor 1 shutdown pin
   sensor1_shutdown = 22
   # GPIO for Sensor 2 shutdown pin
   sensor2_shutdown = 23
   # GPIO for Sensor 3 shutdown pin
   sensor3_shutdown = 27

   GPIO.setwarnings(False)

   # Setup GPIO for shutdown pins on each VL53L0X
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(sensor1_shutdown, GPIO.OUT)
   GPIO.setup(sensor2_shutdown, GPIO.OUT)
   GPIO.setup(sensor3_shutdown, GPIO.OUT)

   # Set all shutdown pins low to turn off each VL53L0X
   GPIO.output(sensor1_shutdown, GPIO.LOW)
   GPIO.output(sensor2_shutdown, GPIO.LOW)
   GPIO.output(sensor3_shutdown, GPIO.LOW)

   # Keep all low for 500 ms or so to make sure they reset
   time.sleep(0.50)

   # Create one object per VL53L0X passing the address to give to each.
   tof1 = vl53.VL53L0X(address=0x2B)
   tof2 = vl53.VL53L0X(address=0x2D)
   tof3 = vl53.VL53L0X(address=0x2C)

   # Set shutdown pin high for the first VL53L0X then 
   # call to start ranging 
   GPIO.output(sensor1_shutdown, GPIO.HIGH)
   time.sleep(0.50)
   tof1.start_ranging(vl53.VL53L0X_HIGH_SPEED_MODE)
   #tof1.start_ranging(vl53.VL53L0X_BETTER_ACCURACY_MODE)
   #tof1.start_ranging(vl53.VL53L0X_LONG_RANGE_MODE)

   # Set shutdown pin high for the second VL53L0X then 
   # call to start ranging 
   GPIO.output(sensor2_shutdown, GPIO.HIGH)
   time.sleep(0.50)
   tof2.start_ranging(vl53.VL53L0X_HIGH_SPEED_MODE)

   # Set shutdown pin high for the second VL53L0X then 
   # call to start ranging 
   GPIO.output(sensor3_shutdown, GPIO.HIGH)
   time.sleep(0.50)
   tof3.start_ranging(vl53.VL53L0X_HIGH_SPEED_MODE)

   #timing = tof.get_timing()
   #if (timing < 20000):
   #   timing = 20000
   #print ("Timing %d ms" % (timing/1000))

   return tof1,tof2,tof3

def shutdown(tof1,tof2,tof3):
   tof1.stop_ranging()
   tof2.stop_ranging()
   tof3.stop_ranging()
   #GPIO.output(sensor2_shutdown, GPIO.LOW)
   #GPIO.output(sensor1_shutdown, GPIO.LOW)



if __name__=="__main__":
   import keyin

   tof1,tof2,tof3=start()

   period=0.05
   kbd=keyin.Keyboard()
   now=time.time()
   start=now
   init=now
   rate=0
   print("Input 'q' to stop this program")
   key='c'
   while key!='q':
      try:
         distance1 = tof1.get_distance()
         time.sleep(0.02)
         distance2 = tof2.get_distance()
         time.sleep(0.02)
         distance3 = tof3.get_distance()
         now = time.time()
         rate+=1
         #print (" %6.2f %d %d mm" % (now-start, distance, distance1) )
         #time.sleep(timing/1000000.00)
         #time.sleep(0.01)

       
         if now-init>period: 
            rate=rate/period
            print ("\r time=%6.2f %4d %4d %4d mm rate=%3d" % (now-start, distance1, distance2,distance3,rate), end=' ')
            rate=0
            init=now
        
      except :
         pass

      key=kbd.read()

   shutdown(tof1,tof2,tof3)
