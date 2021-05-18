#! /usr/bin/python3
# CC BY-SA Yasushiu Honda 2018 12/15
# 変更者：山田将司 2021/04/25
# 各種パッケージ，ソースコードをインポート

import datetime
import numpy as np
import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from subprocess import Popen
#sokcet tuusinn kannkei
import socket
import socket1a as sk


#  定数を定義
MY_IP = "172.16.7.42"


a = 644.3
b = -25.19
c = 0.6182
d = -2.104
e = 0.2202

class PI_CAMERA_CLASS():
   def __init__(self):
      self.udp = sk.UDP_Send(MY_IP, sk.PICAM_PORT)
      self.select_rect='n' # y(es)/n(o) 検出対象を選ぶかどうか
      self.imshow='y'      # y(es)/n(o) カメラ映像を表示するかどうか，VNC必須
      self.show_res='y'    # y(es)/n(o) 結果を表示(print)するかどうか
      self.PERIOD=0.01 #FPSですね
      
      self.A = 644.3
      self.B = -25.19
      self.C = 0.6182
      self.D = -2.104
      self.E = 0.2202
      
      # カメラの解像度 例：640x480, 320x240
      self.RES_X=int( 320 )
      self.RES_Y=int( 320 )
      
      # initialize the camera and grab a reference to the raw camera capture
      #カメラを初期化，カメラへのアクセス？ルート？オブジェクト作成？
      self.cam = PiCamera()
      self.cam.framerate = 30  #フレームレート
      self.cam.brightness = 60 #明るさ
      #cam.saturation = 50
      #cam.exposure_compensation = 0
      #print(cam.exposure_compensation)

      self.cam.awb_mode='auto'
      
      self.cam.awb_mode='auto'
      #　コントラスト設定かな？
      #   #Auto White Balance :list_awb = ['off', 'auto', 'sunlight', 'cloudy', 'shade']
      self.cam.iso=800
      self.cam.shutter_speed=1000000
      self.cam.exposure_mode = 'off' # off, auto, fixedfps
      time.sleep(3)
      self.g = self.cam.awb_gains
      self.cam.awb_mode = 'off'
      self.cam.awb_gains = self.g

      self.cam.resolution = (self.RES_X, self.RES_Y)
      self.cam.rotation=0
      self.cam.meter_mode = 'average' # average, spot, backlit, matrix
      self.cam.exposure_compensation = 0
      self.rawCapture = PiRGBArray(self.cam, size=(self.RES_X, self.RES_Y))

      self.rawCapture.truncate(0) # clear the stream for next frame
      
      #Red Cup H:S:V=3:140:129
      # h,s,v = 171,106,138
      self.H = 174
      self.S = 138
      self.V = 144
      self.h_range = 10
      self.s_range = 100
      self.v_range = 80 # 明度の許容範囲
      self.lower_light = np.array([self.H-self.h_range, self.S-self.s_range, self.V-self.v_range])
      self.upper_light = np.array([self.H+self.h_range, self.S+self.s_range, self.V+self.v_range])


      #Pink ball
      #lower_light = np.array([154, 70, 50])
      #upper_light = np.array([194, 200, 200])

      # UDP send object
      #udp=sk.UDP_Send(sk.ROBOT_ADDR, sk.PICAM_PORT)
      #udp2mpl53=sk.UDP_Send(sk.MLTPICAM_BASE_ADDR, sk.PICAM22MPL53_PORT)

      self.data=[0,0,0,0,0] # For UDP socket

   def calc_dist_theta(self):
      tmp = self.cam.capture_continuous(self.rawCapture, format="bgr", use_video_port="True")
      cap = next(tmp)
      frame = cap.array
      #print(frame)
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, self.lower_light, self.upper_light)
      image, contours, hierarchy  = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      rects = []
      for contour in contours:
         approx = cv2.convexHull(contour)
         rect = cv2.boundingRect(approx)
         rects.append(np.array(rect))
    
      if len(rects) > 0:
         rect = max(rects, key=(lambda x: x[2] * x[3]))
         #cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
         self.data=list(rect)
         self.data[0]=self.data[0] # レンズのズレ補正
         #self.data.append(rate)
         #rint(self.data) 
         #self.udp.send(frame)
         #udp2mpl53.send(data)
         #rate=count/PERIOD
         px=self.data[0]
         py=self.data[1]
         width = self.data[2]
         height = self.data[3]
         cpx = px + (width/2)
         cpy = py + (height/2)
         rad = (160-cpx)*(52/160)*(np.pi/180)
         angle = np.rad2deg(rad)
         dis = (self.A/(width**self.C)) + self.B + (self.D*(abs(rad)**self.E))
         dis = float(dis/100)
         #print("\r %6.4f %6.4f" % (angle, dis ), end="" )
         self.rawCapture.truncate(0) # clear the stream for next frame
      else: #red cup not capture
         dis = None
         rad = None
      cv2.imshow("frame", frame)
      cv2.waitKey(3)
      self.rawCapture.truncate(0) # clear the stream for next frame
      return dis, rad

if __name__ == "__main__":
    picam = PI_CAMERA_CLASS()
    count = 0
    while 1:
        try:
            print(picam.calc_dist_theta())
            #dis, rad = picam.calc_dist_theta()
            #cap = next(tmp)
            #frame = cap.array
            #cv2.imshow("test", frame)
            #cv2.waitKey(3)
        except KeyboardInterrupt:
            print("ctrl + C ")
            cv2.destroyAllWindows()

