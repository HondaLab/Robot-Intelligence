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

udp = sk.UDP_Send(MY_IP, sk.PICAM_PORT)
select_rect='n' # y(es)/n(o) 検出対象を選ぶかどうか
imshow='y'# y(es)/n(o) カメラ映像を表示するかどうか，VNC必須
show_res='y'    # y(es)/n(o) 結果を表示(print)するかどうか
PERIOD=0.01 #FPSですね

# カメラの解像度 例：640x480, 320x240
RES_X=int( 320 )
RES_Y=int( 320 )

# initialize the camera and grab a reference to the raw camera capture
#カメラを初期化，カメラへのアクセス？ルート？オブジェクト作成？
cam = PiCamera()
cam.framerate = 30  #フレームレート
cam.brightness = 60 #明るさ
#cam.saturation = 50
#cam.exposure_compensation = 0
#print(cam.exposure_compensation)

cam.awb_mode='auto'
#　コントラスト設定かな？
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

rawCapture.truncate(0) # clear the stream for next frame

#Red Cup H:S:V=3:140:129
# h,s,v = 171,106,138
H = 174
S = 138
V = 144
h_range = 10
s_range = 100
v_range = 80 # 明度の許容範囲
lower_light = np.array([H-h_range, S-s_range, V-v_range])
upper_light = np.array([H+h_range, S+s_range, V+v_range])


#Pink ball
#lower_light = np.array([154, 70, 50])
#upper_light = np.array([194, 200, 200])

# UDP send object
#udp=sk.UDP_Send(sk.ROBOT_ADDR, sk.PICAM_PORT)
#udp2mpl53=sk.UDP_Send(sk.MLTPICAM_BASE_ADDR, sk.PICAM22MPL53_PORT)

data=[0,0,0,0,0] # For UDP socket
print("while before complete")
#while 1:
for cap in cam.capture_continuous(rawCapture, format="bgr", use_video_port="True"):
    #cap = cam.capture_continuous(rawCapture, format="bgr", use_video_port="True")
    frame = cap.array
    #print(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_light, upper_light)
    image, contours, hierarchy  = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        approx = cv2.convexHull(contour)
        rect = cv2.boundingRect(approx)
        rects.append(np.array(rect))
    
    if len(rects) > 0:
        rect = max(rects, key=(lambda x: x[2] * x[3]))
        #cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
        data=list(rect)
        data[0]=data[0] # レンズのズレ補正
        #data.append(rate)
        #print(data) 
        #udp.send(frame)
        #rate=count/PERIOD
        px=data[0]
        py=data[1]
        width = data[2]
        height = data[3]
        cpx = px + (width/2)
        cpy = py + (height/2)
        rad = (160-cpx)*(52/160)*(np.pi/180)
        angle = np.rad2deg(rad)
        dis = (a/(width**c)) + b + (d*(abs(rad)**e))
        dis = float(dis/100)
        #print("\r %6.4f %6.4f" % (angle, dis ), end="" )
        rawCapture.truncate(0) # clear the stream for next frame
        cv2.imshow('frame',frame)
    else: #red cup not capture
        cv2.imshow('frame',frame)
        rawCapture.truncate(0) # clear the stream for next frame

cv2.destroyAllWindows()
