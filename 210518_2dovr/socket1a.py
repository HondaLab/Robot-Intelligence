#!/usr/bin/python3
# socket1a.py
# CC BY-SA Yasushi Honda Since 2018 12/25
# UDP socketを使ってデータの送受信をするためのクラス
# 使い方：送信側と受信側の双方で、このモジュールをインポートして使う。
# ex: import socket1a as sk
import socket

# アドレスとポート
# アドレスはIPアドレスを直接していしても良いが、
# /etc/hostsに登録してあるホスト名を用いることもできる
ROBOT_ADDR = '172.16.7.101' # 
# port番号は32768 -- 60999 がカスタム用途のプライベート番号(cf. /proc/sys/net/ipv4/ip_local_port_range)
PICAM_PORT = 50002 #piカメラのポート番号

class UDP_Send(): # send list data to (addr,port)
   def __init__(self,addr,port):
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # SOCK.DGRAMとすることで、UDPを指定
      self.addr = addr
      self.port = port

   def send(self,list): 
      #print(list) 
      string=''
      num=len(list)
      i=0
      while i<num:
         string = string + str("%12.8f" % list[i]) 
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
      #message = self.sock.recv(1024).decode('utf-8')
      message = self.sock.recv(1024).decode('utf-8')
      #print(message)
      slist=message.split(',')
      #print(slist)
      a=[float(s) for s in slist]
      #print(a) 
      return a
