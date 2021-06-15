#!/usr/bin/python3
import cv2
import os
import time
import modules.keyin as keyin
import numpy as np

#(640x360),(1280x720),(1440x1440),... for SF360 4K
WIDTH=1440
HEIGHT=1440
size = (WIDTH, HEIGHT)
record_fps=12 # システムのスピードに応じて変更する必要あり
camera_dev='/dev/video0'

frame_rotation = 'y'
rotation_angle = int(180)

# open camera
cap = cv2.VideoCapture(camera_dev, cv2.CAP_V4L2)
if cap.isOpened(): # カメラが開けた場合
   record_frame=input('# 録画しますか(y/n default=y)')
   if record_frame=='':
      record_frame='y'
   if record_frame=='y':
      OUT_FILE='out.mp4'
      while os.path.exists(OUT_FILE):
         print("# %sはすでに存在しています．" % OUT_FILE)
         OUT_FILE=input('## 新しい出力ファイル名:')
         OUT_FILE=OUT_FILE+'.mp4'
      print('%s に動画を書き出します．' % OUT_FILE)

   # set dimensions
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)
   #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
   cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
   cap.set(cv2.CAP_PROP_BUFFERSIZE, 4) # BUFFERSIZE を小さくするとレートが速くなる
   cap.set(cv2.CAP_PROP_FPS, 30)

   if record_frame=='y':
      # 保存用
      fmt = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
      #frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
      frame_rate = 4
      width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
      height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
      size = (width, height)
      vw = cv2.VideoWriter(OUT_FILE, fmt, record_fps, size)

   count=0
   print("# Input 'q' to stop the camera.")
   now=time.time()
   start=now
   key=keyin.Keyboard()
   ch='c'  
   ch_im=cv2.waitKey(1)
   while not(ch=='q' or ch_im==ord('q') or ch=='Q' or ch_im==ord('Q')):
   #while now-start<10:
      # take frame
      ret, frame = cap.read()
      if frame_rotation == 'y':
          center2 = tuple(np.array([frame.shape[1] * 0.5, frame.shape[0] * 0.5]))
          rotation_matrix = cv2.getRotationMatrix2D(center2, rotation_angle, 1.0)
          frame = cv2.warpAffine(frame, rotation_matrix, size, flags=cv2.INTER_CUBIC)
          #frame=cv2.rotate(frame,cv2.ROTATE_90_COUNTERCLOCKWISE)
      cv2.imshow('frame',frame)
      ch_im=cv2.waitKey(1)
      ch=key.read()
      if record_frame=='y':
         vw.write(frame)
        
      count+=1
      now=time.time()
      print("\r %7.2f" % (now-start), end="")
      #time.sleep(0.1)

   now=time.time()
   rate=count/(now-start)
   speed=1.0/rate*1000
   print()
   print("rate=%5.2f (Hz)" % rate)
   print("speed=%5.2f (msec)" % speed)

   if record_frame=='y':
      print("%sに%7.2f(sec) 録画されました" % (OUT_FILE,now-start))
      vw.release()
   # release camera
   cap.release()

else: # カメラが開けなかった場合
   print("# Camera is NOT opened.")
   print("# Connect and turn on USB camera.")
