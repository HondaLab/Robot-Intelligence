#!/usr/bin/python3
# CC BY-SA Yasushiu Honda 2018 12/15

# import the necessary packages
import datetime
import numpy as np
import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from subprocess import Popen
import sock4robot_7a as sk

select_rect='y' # y/n
imshow='y' # y/n
show_res='y'

OUTPUT = datetime.datetime.now().strftime("%m%d-%Y_%H:%M:%S")+".ctrack"
PERIOD=1.0

# Resolution of camera ex. 640x480, 320x240
RES_X=int( 320 )
RES_Y=int( 320 )

# initialize the camera and grab a reference to the raw camera capture
cam = PiCamera()
cam.framerate = 30
#cam.brightness = 50
#cam.saturation = 50

cam.awb_mode='auto'
#   #Auto White Balance :list_awb = ['off', 'auto', 'sunlight', 'cloudy', 'shade']
cam.iso=800
cam.shutter_speed=1000000
cam.exposure_mode = 'off' # off, auto, fixedfps
time.sleep(3)
g = cam.awb_gains
cam.awb_mode = 'off'
cam.awb_gains = g

cam.resolution = (RES_X, RES_Y)
cam.rotation=0
cam.meter_mode = 'average' # average, spot, backlit, matrix
cam.exposure_compensation = 0
rawCapture = PiRGBArray(cam, size=(RES_X, RES_Y))


# 動画出力用
result = "output.m4v" 
fps    = 20 # 実際のfpsに合わせて調整必要
# MP4V形式で出力
fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
# 出力先のファイルを開く
out = cv2.VideoWriter(result, int(fourcc), fps, (RES_X, RES_Y))

rawCapture.truncate(0) # clear the stream for next frame

# トラッキングする物体を選ぶ
if select_rect=='y':
   cam.capture(rawCapture, format="bgr", use_video_port=True)
   frame = rawCapture.array
   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   bbox = (0,0,10,10)
   bbox = cv2.selectROI(frame, False)
   #ok = tracker.init(frame, bbox)
   #print(bbox)
   x=int(bbox[0]+bbox[2]/2)
   y=int(bbox[1]+bbox[3]/2)
   print("hsv: %4d %4d %4d" % (hsv[y,x,0],hsv[y,x,1],hsv[y,x,2]))
   rawCapture.truncate(0) # clear the stream for next frame
   cv2.destroyAllWindows()

   h_range=20 # 色相の許容範囲
   s_range=100 # 彩度の許容範囲
   v_range=80 # 明度の許容範囲
   #cvtColorでつくったhsv配列はy,xの順序なので注意
   hL=hsv[y,x,0]-h_range
   hU=hsv[y,x,0]+h_range
   sL=hsv[y,x,1]-s_range
   sU=hsv[y,x,1]+s_range
   vL=hsv[y,x,2]-v_range
   vU=hsv[y,x,2]+v_range
   lower_light=np.array([hL,sL,vL])
   upper_light=np.array([hU,sU,vU])
else:
   #Green
   #lower_light = np.array([30, 100, 50])
   #upper_light = np.array([50, 255, 180])
   #Pink ball
   lower_light = np.array([154, 70, 50])
   upper_light = np.array([194, 200, 200])
   #lower_light = np.array([0, 70, 50])
   #upper_light = np.array([15, 200, 200])
   # for awb_mode='sunlight'
   #lower_light = np.array([7, 163, 140])
   #upper_light = np.array([18,243, 220])
#print(lower_light)
#print(upper_light)

# UDP send object
udp=sk.UDP_Send(sk.ROBOT_ADDR, sk.PICAM_PORT)
udp2mpl53=sk.UDP_Send(sk.MLTPICAM_BASE_ADDR, sk.PICAM22MPL53_PORT)

data=[0,0,0,0,0] # For UDP socket 
now=time.time()
init = now
start=now
count=0
rate=0
# camera capture loop
for cap in cam.capture_continuous(rawCapture, format="bgr", use_video_port="True"):
   frame = cap.array
   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv, lower_light, upper_light)

   #res = cv2.bitwise_and(frame,frame, mask= mask)
    
   image, contours, hierarchy  = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   rects = []
   for contour in contours:
      approx = cv2.convexHull(contour)
      rect = cv2.boundingRect(approx)
      rects.append(np.array(rect))

   if len(rects) > 0:
      rect = max(rects, key=(lambda x: x[2] * x[3]))
      cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
      data=list(rect)
      data[0]=data[0] # レンズのズレ補正
      data.append(rate)
      #print(data) 
      udp.send(data)
      udp2mpl53.send(data)
      count = count + 1

   if imshow=='y':
      cv2.imshow('frame',frame)

   # Output to file
   #out.write(frame)

   key=cv2.waitKey(1) & 0xFF
   rawCapture.truncate(0) # clear the stream for next frame

   if show_res=='y':
      now = time.time()
      if now-start > PERIOD:
         rate=count/PERIOD
         px=data[0]
         py=data[1]
         print("\r %4d %4d %4d %4d %4d" % (rate,px,py,data[2],data[3]),end="") 
         count = 0
         start = now
    
   if key == ord("q"):
      print("\n")
      break

cv2.destroyAllWindows()
