#!/usr/bin/python3

# Observe under distance by Lidar LiteV3
# 'haute_lidarXX_H9T06b.py' CC-BY-SA Yasushi Honda Since 2019.3.13

# Robot Intelligence, Sensory Motor Mapping with self feedback
# パルス応答や軌道生成
# 安定化は modules/tanh1_H941.py の中で行っている。

# haute_ld1_H6T06a.py Since 2018 9.25
# 高度をLidarで制御 (2018 9/28未完成)
# ネットワークアドレスの構成など変更。また、sf4_h6t06bのなかで、ゲイン調整

# 2018 11/22
# haute_smem4_fPicam1_H7T06b.py for tracking object by front Picam+OpenCV

import numpy as np # smemに必要
import pigpio      # i2c, pwmに必要
import time,datetime # 時間計測など
import sys         # sys.stdout でその場文字出力
import socket      # soket通信の関数は下記sock4robotに含まれるが、
                   # noblocking recieve のためのexceptに使う
import os
from subprocess import Popen # 別のプロセスを生成して、それと通信（ポーリング）する。
# 以下自作モジュール
import modules.keyin as kbd  # modules/keyin.py をインポート
import modules.tanh1_H941 as tanh1 # 機体水平安定化のためのRobotクラス
import modules.sock4robot_7a as sk # socket通信のための関数群

color_track='n' # y/n
keep_height='n' # y/n
gains_output='n' # y/n

GAIN_FILE='gains_H941.dat'
BASE_HOST ="zelkova54w"
ROBOT_HOST ="rp220w"
CAMERA_HOST = "rp218w" # another raspi3 on the robot
LIDAR_HOST = "rp218w"
CAMERA_PASS = "xxxx"
OUT_PERIOD = 0.0333

LIDAR_PYTHON = "lidar_udp_2a.py"
CAMERA_PYTHON = "ctrack7a.py"
VIDEO_FILE = '/tmp/video'

F_CTRACK = datetime.datetime.now().strftime("%m%d-%Y_%H:%M:%S")+".ctrack"


# list size for short memmory
Nrange=70 

FLIGHT_START_HEIGHT=80.0 # cm

# 短期記憶(smem)を考慮して物体を追いかける(2018 11/22)
class Tracking():
   def __init__(self, gains):
      self.gain_x=gains[0]    #0.085
      self.gain_w=gains[1]    #0.0100
      self.gain_smem=gains[2] #0.0070
      self.amp=gains[3]       #5.0
      self.delta=gains[4]     #0.80  past in sec
      self.x_center=gains[5]  # 162
      self.normal_width=gains[6] #38

      print('gain_x=%7.4f' % self.gain_x)
      print('gain_w=%7.4f' % self.gain_w)
      print('gain_s=%7.4f' % self.gain_smem)
      print('amp=%7.4f' % self.amp)
      print('delta=%7.4f' % self.delta)
      print('x_c=%7.4f' % self.x_center)
      print('normal_w=%7.4f' % self.normal_width)

   def getpuls(self, bbox,width_smem,rate):
     
      x=bbox[0]
      y=bbox[1]
      width=bbox[2]
      height=bbox[3]
      x = x + width/2
      y = y + height/2

      # Z軸回り回転
      val=-self.gain_x*(x-self.x_center) 
      m0=+val
      m1=-val
      m2=+val
      m3=-val

      # 物体との距離維持
      past=Nrange-int(rate*self.delta)-1
      diff=width-self.normal_width
      try:
         val=self.gain_w*diff-self.gain_smem*(width-width_smem[past])
         #print(past,width_smem[past])
      except:
         print("%d is out of smem range" % past)
         val=0

      if(diff<0):
         val=self.amp*val

      m0=m0+val
      m1=m1+val
      m2=m2-val
      m3=m3-val

      return [m0,m1,m2,m3]

# 床から距離を保つ
def Keep_Height(dist,lidar_smem,rate):
   keep=120  # cm
   gain_k = 0.050  # 0.050
   gain_s = 0.050
   delta=0.2
   past=Nrange-int(rate*delta)-1
   try:
      val=gain_k*(dist-keep)+gain_s*(dist-lidar_smem[past])
   except:
      print("%d is out of smem range" % past)
      val=0

   m0=-val
   m1=-val
   m2=-val
   m3=-val

   return [m0,m1,m2,m3]

# 壁から距離を保つ
def Keep_Dist(dist,lidar_smem,rate):
   keep=150  # cm
   gain_k = 0.018
   gain_s = 0.020
   delta=0.8
   past=Nrange-int(rate*delta)-1
   val=gain_k*(dist-keep)-gain_s*(dist-lidar_smem[past])
   m0=-val
   m1=+val
   m2=+val
   m3=-val

   return [m0,m1,m2,m3]

# パルス発生関数
def puls_gen(robot,val,width):
   m0=val-5
   m1=val-5
   m2=val+5
   m3=val+5
   robot.m0.run(m0)
   robot.m1.run(m1)
   robot.m2.run(m2)
   robot.m3.run(m3)
   time.sleep(width)


# main
if __name__ == '__main__':

   # BASE_HOSTと時刻をntpdateで合わせる。
   NTPDATE_FILE = '/tmp/ntpdate'
   ADJUST_SEC = 7
   exist = os.path.exists(NTPDATE_FILE)
   if not exist:
      print("Now adjusting time with %s, wait %s sec" % (BASE_HOST,ADJUST_SEC))
      cmd = "sudo ntpdate %s" % BASE_HOST
      ntpdate = Popen( cmd .strip().split(" ") )
      cmd = "sshpass -p %s ssh pi@%s sudo ntpdate %s" % (CAMERA_PASS,CAMERA_HOST,BASE_HOST)
      ntpdate = Popen( cmd .strip().split(" ") )
      time.sleep(ADJUST_SEC)
      cmd = "touch %s" % NTPDATE_FILE
      touch = Popen( cmd .strip().split(" ") )

   # ゲインなどの読み込み
   data_size=8
   gain_list=np.empty((0,data_size),float)
   gains=[0 for i in range(data_size)]
   tmp_list=[0 for i in range(data_size)]
   gain_f=open(GAIN_FILE,'r')
   for line in gain_f:
      #print(line)
      if line[0]!='#':
         tmp_list=line.strip('\n').split(',')
         i=0
         while i<len(tmp_list)-1:
            gains[i]=float(tmp_list[i]) 
            i+=1
         #print(gains)
         #print(tmp_list[len(tmp_list)-1])
         gain_list=np.append(gain_list,np.array([gains]),axis=0)
         #print(gain_list)
   gain_f.close()
   tracking=Tracking(gains)

   # 慣性センサー読み取りプロセスの生成
   cmd = "modules/mpu9150.py"
   mpu9150 = Popen( cmd .strip().split(" ") )
   time.sleep(1)

   # トラッキングカメラプロセス生成
   trk = sk.UDP_Recv(ROBOT_HOST, sk.PICAM_PORT)
   width_smem=[0.0 for i in range(Nrange)] # 短期記憶用の配列
   cmd = "./camera/%s &" % CAMERA_PYTHON
   picam_pross = Popen( cmd .strip().split(" ") )

   # トラッキングカメラ(#2)プロセス生成
   trk2 = sk.UDP_Recv(ROBOT_HOST, sk.PICAM2_PORT)
   #width_smem=[0.0 for i in range(Nrange)] # 短期記憶用の配列
   #cmd = "./camera/%s &" % CAMERA_PYTHON
   cmd = "sshpass -p %s ssh pi@%s ./camera/ctrack6b.py &" % (CAMERA_PASS,CAMERA_HOST)
   picam2_pross = Popen( cmd .strip().split(" ") )


   # Lidarをつかう
   udp_lidar = sk.UDP_Recv(sk.ROBOT_ADDR, sk.LIDAR_PORT)
   lidar_smem=[0.0 for i in range(Nrange)] # 短期記憶用の配列
   lidar=[0]
   cmd = "sshpass -p fsr2 ssh pi@%s ./190119_Lidar/%s &" % (LIDAR_HOST,LIDAR_PYTHON)
   lidar_pross = Popen( cmd .strip().split(" ") )

   """
   # 録画カメラプロセス生成
   cmd = "sshpass -p %s ssh pi@%s ./camera/picamera2b.py &" % (CAMERA_PASS,CAMERA_HOST)
   movie = Popen( cmd .strip().split(" ") )
   cmd = "touch %s" % VIDEO_FILE
   touch = Popen( cmd .strip().split(" ") )
   print("Video capture by Picamera start.")
   """

   # スロットル step
   step=3

   m0_GPIO = 17
   m1_GPIO = 18
   m2_GPIO = 23
   m3_GPIO = 22

   robot = tanh1.Robot("H941")
   robot.m0.connect(m0_GPIO)
   robot.m1.connect(m1_GPIO)
   robot.m2.connect(m2_GPIO)
   robot.m3.connect(m3_GPIO)

   robot.zero()
   time.sleep(1)

   start = time.time()
   init = time.time()
   count=0
   lidar_count=0
   dist=150
   c1x=160
   c1y=160
   c2x=160
   c2y=160
   key = kbd.Keyboard()
   print('\n Input "q" to stop')
   print('[ftime rate 出力:  L( x, y), R( x, y): lidar rate]')
   f_ct=open(F_CTRACK,'w')
   f_ct.write('#[time ftime L( x, y) R( x, y): lidar]\n')
   throttle=0
   motor=[0,0,0,0] # m0 -- m3 に対応したリスト
   motor_lidar=[0,0,0,0]
   bbox=[0,0,0,0,0]
   motor_bbox=[0,0,0,0]
   ch='c'
   flight_start=0
   l_rate=0
   while(ch!='q'):

      try: # Tracking Picamから受信
         bbox=trk.recv()
         x=bbox[0]
         y=bbox[1]
         width=bbox[2]
         width_show=width
         height=bbox[3]
         c_rate=bbox[4]
         c1x = x + width/2
         c1y = y + height/2
         width_smem.pop(0)
         width_smem.append(width) 
         #print(bbox[0],bbox[1])
         motor_bbox=tracking.getpuls(bbox,width_smem,c_rate)
      except (BlockingIOError, socket.error):
         c_rate=-1

      try: # Tracking Picam2から受信
         bbox=trk2.recv()
         x=bbox[0]
         y=bbox[1]
         width=bbox[2]
         height=bbox[3]
         c_rate=bbox[4]
         c2x = x + width/2
         c2y = y + height/2
         #width_smem.pop(0)
         #width_smem.append(width) 
         #print(bbox[0],bbox[1])
         #motor_bbox=tracking.getpuls(bbox,width_smem,c_rate)
      except (BlockingIOError, socket.error):
         c_rate=-1

      try: # Lidarから
         lidar = udp_lidar.recv()
         lidar_count=lidar_count+1
         dist=lidar[0]
         lidar_smem.pop(0)
         lidar_smem.append(dist) 
        
         # Lidarを下面につけて高さキープに使う場合
         lidar_height=lidar[0]
         #print(lidar)
         if (lidar_height>FLIGHT_START_HEIGHT)&(flight_start==0) :
            flight_start=1
            flight_start_time=time.time()
            #print(flight_start_time,lidar_height,flight_start)
         motor_lidar=Keep_Height(dist,lidar_smem,l_rate)
      except (BlockingIOError, socket.error):
         pass

      try: # キーボードでスロットルコントロール
         ch=key.read()
         if(ch=='k'):
            throttle = throttle +1
         if(ch=='l'):
            throttle = throttle +step
         if(ch=='j'):
            throttle = throttle -1
         if(ch=='h'):
            throttle = throttle -step

         if(ch=='r'): #パルス生成
            puls_gen(robot,throttle,0.2)


      except KeyboardInterrupt:
         # 後処理
         robot.zero()
         time.sleep(1)
         mpu9150.terminate()
         print("\nmpu9150 is killed")

         """
         # カメラ録画プロセス終了
         movie.terminate()
         cmd = "sshpass -p fsr2 ssh pi@%s pkill picamera2.py" % CAMERA_HOST
         pkill = Popen( cmd .strip().split(" ") )
         print("picamera2.py is killed")
         """

         lidar_pross.terminate()
         # Lidarプロセス終了
         cmd = "sshpass -p fsr2 ssh pi@%s pkill %s" % (LIDAR_HOST,LIDAR_PYTHON)
         pkill = Popen( cmd .strip().split(" ") )
         print("lidar_pigpio.py is killed")

         time.sleep(1)
         break

      # rateなどの標準出力
      now=time.time()
      if now-start>OUT_PERIOD :
         rate = count/OUT_PERIOD
         l_rate = lidar_count/OUT_PERIOD
         # OUT_PERIOD secおきに出力(throttle)とrateをその場表示
         if flight_start==0:
            flight_start_time=now
         try:
            sys.stdout.write('\033[07m'+"\r %5.1f %4d %3d : (%3d,%3d,%3d),(%3d,%3d): %3d %3d:" % (now-flight_start_time, rate, throttle, c1x,c1y,width_show,c2x,c2y, dist, l_rate))
            f_ct.write("%6.2f, %6.2f, %3d, %3d, %3d, %3d, %3d \n" % (now-init, now-flight_start_time, c1x,c1y,c2x,c2y, dist ))
            sys.stdout.flush()
            f_ct.flush()
         except:
            sys.stdout.write('\033[07m'+"\r %6.1f %4d %3d " % (now-flight_start_time, rate, throttle))
            sys.stdout.flush()
         else:
            pass
         count=0
         lidar_count=0
         start=time.time()

      count=count+1

      # モーター出力値の合計
      # 基本スロットル
      motor[0]=throttle 
      motor[1]=throttle 
      motor[2]=throttle 
      motor[3]=throttle 

      # Front picameraカメラで物体追跡
      #print("%8.2f %8.2f %8.2f %8.2f" % (motor_bbox[0],motor_bbox[1],motor_bbox[2],motor_bbox[3]))
      if color_track=='y':
         motor[0]=motor[0] + motor_bbox[0]
         motor[1]=motor[1] + motor_bbox[1]
         motor[2]=motor[2] + motor_bbox[2]
         motor[3]=motor[3] + motor_bbox[3]

      # Lidarで壁から距離キープ
      if keep_height=='y':
         if flight_start==1:
            motor[0]=motor[0] + motor_lidar[0]
            motor[1]=motor[1] + motor_lidar[1]
            motor[2]=motor[2] + motor_lidar[2]
            motor[3]=motor[3] + motor_lidar[3]

      #機体の水平安定化を加えて，モーター出力する．
      robot.stb(motor)
      #time.sleep(0.001) # このsleepを無くすとrate=1000Hzほどになる

   # 後処理

   f_ct.close()

   if flight_start==1:
      flight_time = time.time()-flight_start_time
   else:
      flight_time = 0.000

   robot.zero()
   mpu9150.terminate()
   print("\nmpu9150 is terminated")

   #"""
   #movie.terminate()
   # カメラ録画プロセス終了
   if os.path.exists(VIDEO_FILE):
      cmd = "sshpass -p fsr2 ssh pi@%s pkill picamera2b" % CAMERA_HOST
      pkill = Popen( cmd .strip().split(" ") )
      print("picamera2.py is killed")
      cmd = "sshpass -p fsr2 ssh pi@%s ./h264.sh" % CAMERA_HOST
      rename = Popen( cmd .strip().split(" ") )
      cmd = "rm %s" % VIDEO_FILE
      rmfile = Popen( cmd .strip().split(" ") )
      print("video rename shell finished.")
   #"""

   # trackingカメラプロセス終了
   cmd = "pkill %s" % CAMERA_PYTHON
   pkill = Popen( cmd .strip().split(" ") )
   print("%s is killed" % CAMERA_PYTHON)
   cmd = "sshpass -p %s ssh pi@%s pkill %s" % (CAMERA_PASS,CAMERA_HOST,CAMERA_PYTHON)
   pkill = Popen( cmd .strip().split(" ") )
   print("%s@%s is killed" % (CAMERA_PYTHON,CAMERA_HOST))

   #lidar_pross.terminate()
   # Lidarプロセス終了
   cmd = "sshpass -p fsr2 ssh pi@%s pkill %s" % (LIDAR_HOST,LIDAR_PYTHON)
   pkill = Popen( cmd .strip().split(" ") )
   print("lidar_pigpio.py is killed")

   if gains_output=='y':
      gain_f=open(GAIN_FILE,"a")
      gain_f.write("%5.3f," % gains[0])
      gain_f.write("%7.4f," % gains[1])
      gain_f.write("%7.4f," % gains[2])
      gain_f.write("%4.1f," % gains[3])
      gain_f.write("%5.2f," % gains[4])
      gain_f.write("%4d," % gains[5])
      gain_f.write("%4d," % gains[6])
      gain_f.write("%9.2f," % (flight_time))
      import datetime
      gain_f.write(datetime.datetime.now().strftime(" %H:%M:%S(%Y %m/%d)")+'\n')
   
   gain_f.close()

   time.sleep(1)
   print('\033[0m'+'\nfinish')
