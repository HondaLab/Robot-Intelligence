#!/usr/bin/python3
# CC BY-SA Yasushiu Honda 2021 2/4
# 検出座標の平均値から位置推定

# import the necessary packages
import datetime
import numpy as np
import heapq
import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from subprocess import Popen
import modules.sock4robot_7a as sk
import modules.keyin as keyin

# 分類器xmlデータ
classifier = cv2.CascadeClassifier('Debian60_5a.xml')

imshow='y' # y/n 結果画像表示
show_res='y' # 結果データ表示
PERIOD=0.50  # 表示間隔 (sec)

OUTPUT = datetime.datetime.now().strftime("%m%d-%Y_%H:%M:%S")+".xy"

# Resolution of camera ex. 640x480, 320x240
RES_X=int( 320 )
RES_Y=int( 320 )

# (3 area)
ULIMIT=int(RES_Y*0.55)
LLIMIT=int(RES_Y*0.80)
LMGL=int(RES_X*0.33)
RMGL=int(RES_X*0.66)

# initialize the camera and grab a reference to the raw camera capture
cam = PiCamera()
cam.framerate = 30
#cam.brightness = 50
#cam.saturation = 50

cam.awb_mode='auto'
#   #Auto White Balance :list_awb = ['off', 'auto', 'sunlight', 'cloudy', 'shade']
cam.iso=800
cam.shutter_speed=1000000
cam.exposure_mode = 'auto' # off, auto, fixedfps
time.sleep(1)
g = cam.awb_gains
cam.awb_mode = 'off'
cam.awb_gains = g

cam.resolution = (RES_X, RES_Y)
cam.rotation=0
cam.meter_mode = 'average' # average, spot, backlit, matrix
cam.exposure_compensation = 0
rawCapture = PiRGBArray(cam, size=(RES_X, RES_Y))

#動画出力用データ
fps    = 25 # 平均fps 
result = "output.mp4"
fourcc = cv2.VideoWriter_fourcc('H', '2', '6', '4')
out = cv2.VideoWriter(result, int(fourcc), fps, (RES_X, RES_Y))

rawCapture.truncate(0) # clear the stream for next frame


# UDP send object
udp=sk.UDP_Send(sk.ROBOT_ADDR, sk.PICAM_PORT)


data=[0,0,0,0,0] # For UDP socket 
key=keyin.Keyboard()
now=time.time()
init = now
start=now
count=0
rate=0
print(" rate    x    y    w    h  num") 
x=0
y=0
w=0
h=0
# camera capture loop
char='c'
for cap in cam.capture_continuous(rawCapture, format="bgr", use_video_port="True"):

   frame = cap.array
   # グレースケールに変換
   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

   # 検出
   detected_profiles = classifier.detectMultiScale(gray, scaleFactor=2.00, minNeighbors=0, minSize=(int(RES_X/30), int(RES_Y/30)))
   # scaleFactorを1に近づけると細かく拡大縮小して検出。精度向上＋速度低下 
   # minNeighborsを0に近づけると検出率が高くなる
   
   x_ave=0
   y_ave=0
   for (x,y,w,h) in detected_profiles:
      x_ave+=x+w/2
      y_ave+=y+h/2
      cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

   num=len(detected_profiles)
   try: # num=0を回避する
      x_ave=x_ave/num
      y_ave=y_ave/num
      w=8
      h=8
      x=int(x_ave-w/2)
      y=int(y_ave-h/2)
      #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
   except: 
      pass

   # 結果データをudpで配信
   data=[rate,x,y,w,h]
   udp.send(data)

   # 結果画像を表示
   if imshow=='y':
      #cv2.imshow('gray',gray)
      cv2.imshow('frame',frame)
      cv2.waitKey(1) 


   if show_res=='y':
      now = time.time()
      if now-start > PERIOD:
         rate=count/PERIOD
         #print("\r %4d %4d %4d %4d %4d" % (rate,x,y,w,h), end='') 
         print("\r %4d" % rate, end='') 
         print(" (%4d %4d %4d %4d)" % (x,y,w,h), end='') 
         print(" %4d" % num, end='') 
         count = 0
         start = now

   # 動画出力
   out.write(frame)

   #key=cv2.waitKey(1) & 0xFF
   char=key.read()
   rawCapture.truncate(0) # clear the stream for next frame
    
   if char == "q":
      print("\n")
      break

   count+=1

out.release()
cv2.destroyAllWindows()
