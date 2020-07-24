#!/usr/bin/python3
# cf. https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
# import the necessary packages

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# 640 x 480
RESO_X=int( 320 )
RESO_Y=int( 240 )

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (RESO_X, RESO_Y)
camera.framerate = 50
camera.brightness = 50
camera.rotation=90
camera.exposure_mode = 'auto' # auto, fixedfps
camera.meter_mode = 'average' # averate, spot, backlit, matrix
camera.exposure_compensation = 0
rawCapture = PiRGBArray(camera, size=(RESO_X, RESO_Y))
 
# allow the camera to warmup
time.sleep(0.1)

# Boosting
#tracker = cv2.TrackerBoosting_create()

# MIL
#tracker = cv2.TrackerMIL_create()

# KCF 速い
#tracker = cv2.TrackerKCF_create()

# TLD 
#tracker = cv2.TrackerTLD_create()

# MedianFlow 速い
tracker = cv2.TrackerMedianFlow_create()

#tracker = cv2.TrackerGOTURN_create()

# トラッキングする物体を選ぶ
camera.capture(rawCapture, format="bgr")
frame = rawCapture.array
bbox = (0,0,10,10)
bbox = cv2.selectROI(frame, False)
ok = tracker.init(frame, bbox)
rawCapture.truncate(0) # clear the stream for next frame
cv2.destroyAllWindows()
 
# ウィンドウの名前、サイズ可変|アスペクト比維持
cv2.namedWindow("Tracking", cv2.WINDOW_KEEPRATIO | cv2.WINDOW_NORMAL )

now = time.time()
init = now
start = now
count = 0
# grab an image from the camera
camera.exposure_mode = 'fixedfps'
for cap in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
   frame = cap.array

   # トラッカーをアップデートする
   track, bbox = tracker.update(frame)

   # 検出した場所に四角を書く
   if track:
       # Tracking success
       p1 = (int(bbox[0]), int(bbox[1]))
       p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
       cv2.rectangle(frame, p1, p2, (0,255,0), 2, 1)
   else :
       # トラッキングが外れたら警告を表示する
       cv2.putText(frame, "Failure", (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA);

   # display the image on screen and wait for a keypress
   cv2.imshow("Tracking", frame)
   print("%5d %5d" % (bbox[0],bbox[1]))
   key=cv2.waitKey(1) & 0xFF

   rawCapture.truncate(0) # clear the stream for next frame

   count = count + 1
   now = time.time()
   if now-start > 1.0:
      print("rate = %d" % count)
      count = 0
      start = now
   

   if key == ord("q"):
      break



# キャプチャをリリースして、ウィンドウをすべて閉じる
#cap.release()
cv2.destroyAllWindows()