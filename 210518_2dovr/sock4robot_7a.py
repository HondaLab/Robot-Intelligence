#!/usr/bin/python3
# 2018 3/19 データをPC(rock7)にも送る
#
# sock4robot.py
# CC-BY-SA Yasushi Honda Since 2018 2/7
# UDP socketを使ってセンサー値(ex. Lidar)をRobotに送受信するためのクラス
# Robot とセンサー双方とも、このファイルをインポートして使う
# import sock4robot as sk
# センサーの種類を追加する際には、PORTおよび受信と送信用のクラスを
# それぞれここに追加していくことによってsocket通信を一元管理する。

import socket

# 受取側のアドレス
LOCAL_HOST = 'localhost' 
MPU9150_PORT = 50001 # 慣性センサ用のポート番号

ROBOT_ADDR = '172.16.7.101' # 
CAMERA_ADDR = '172.16.7.101' # 
BASE_ADDR = 'rock7w' # Base Debian PC
PIXY_BASE_ADDR = 'maple53w' # Base Debian PC
MLTPICAM_BASE_ADDR = 'maple53w' # Debian PC for multi-picamera data recieve

LIDAR_PORT = 50002 # PORTはセンサーごとに異る値を用いる
PIXY_PORT = 50003 #  Under PIXY <--> PIXY_BASE (for pixy data and MC data)
FPIXY_PORT = 50004 #  Front PIXY <--> BASE (Control PC)
FRPIXY_PORT = 50005 #  Front PIXY <--> Robot

PICAM_PORT = 50006 # Front(rp220) Piamera <--> Robot
PICAM2_PORT = 50008 # Front(rp218) Piamera <--> Robot

MPU2MPL53_PORT=50021     # MPU9150(rp220) <--> maple53w
PICAM2MPL53_PORT=50022   # Picam@rp220 <--> maple53w
PICAM22MPL53_PORT=50023  # Picam@rp218 <--> maple53w

class UDP_Send(): # send list data to (addr,port)
   def __init__(self,addr,port):
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.addr = addr
      self.port = port

   def send(self,list): 
      #print(list) 
      string=''
      num=len(list)
      i=0
      while i<num:
         string = string + str("{0:8.4f}".format(list[i])) 
         if i!=num-1:
           string=string+','
         i=i+1
      #print(string)

      self.sock.sendto(string.encode('utf-8'),(self.addr,self.port))
      return 0

class UDP_Recv(): # Recieve list data from (addr,port)
   def __init__(self,addr,port):
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.sock.bind((addr,port))
      self.sock.setblocking(0) # ポーリングのためのNoblocking受信

   def recv(self):
      message = self.sock.recv(1024).decode('utf-8')
      #print(message)
      slist=message.split(',')
      #print(slist)
      a=[float(s) for s in slist]
      #print(a) 
      return a

'''
 recv側のプログラム(Robot)はこの例のように
 exceptでBlockingIOErrorを処理しなければならない。
 recv側のloop rate が send側のloop rateよりも大きくなるようにtime.sleep
 を調節する。これにより、センサー値のリアルタイム性が確保される。
 recv側のtime.sleepを外すと、rateは最大となるが、ビジーループとなり、
 CPUを最大消費するので注意が必要。
'''

if __name__=='__main__':
   import time
   import socket

   udp=UDP_Recv()
   message='null'
   start=time.time()
   while time.time()-start<30 :
      try:
         message = udp.recv(LOCAL_HOST,PORT)
         print("{0:8.4f} {1:10s}".format(time.time()-start,message))
         time.sleep(0.05)
      except (BlockingIOError,  socket.error):
         # Noblockingなので、まだ届いてない場合の処理
         print("{0:8.4f} {1:10s}".format(time.time()-start,message))
         time.sleep(0.05)
         continue
      except OSError:
         break
