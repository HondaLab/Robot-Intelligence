#!/usr/bin/python3
# CC-BY-SA 2018 2/10 Y.Honda
# mpu9150.py


class MPU9150():

   def __init__(self):
      import pigpio
      import time

      BUS=1
      ADD=0x68
      self.offset=32768
      self.norm=3.141592/(2*16000)
      self.gnorm=0.0000950

      self.pi=pigpio.pi()

      self.h=self.pi.i2c_open(BUS,ADD)
      if self.h<0:
         print('no handle')
         exit()

      who=self.pi.i2c_read_byte_data(self.h,0x75)
      #print('Who am I='+hex(who))

      self.pi.i2c_write_device(self.h,[0x6b,0x00])
      val=self.pi.i2c_read_byte_data(self.h,0x6b)
      #print('val(0x6b)='+hex(val))

      self.pi.i2c_write_device(self.h,[0x37,0x02])
      val=self.pi.i2c_read_byte_data(self.h,0x37)
      #print('val(0x37)='+hex(val))

      # Low Pass Filter 
      dlpf=self.pi.i2c_read_byte_data(self.h,0x1a)
      print('DLPF='+hex(dlpf))
      time.sleep(1)
      if dlpf != 0x05 :
         i=0
         while i<3:
            self.pi.i2c_write_device(self.h,[0x1a,0x05])
            time.sleep(1)
            dlpf=self.pi.i2c_read_byte_data(self.h,0x1a)
            print('DLPF='+hex(dlpf))
            i=i+1
      print("mpu9150 starts.")

   def getAccel(self):
      high=self.pi.i2c_read_byte_data(self.h,0x3b)
      low =self.pi.i2c_read_byte_data(self.h,0x3c)
      ax=high*256+low
      if ax>self.offset:
         ax=2*self.offset-ax
      else:
         ax=-ax
      ax=ax*self.norm

      high=self.pi.i2c_read_byte_data(self.h,0x3d)
      low =self.pi.i2c_read_byte_data(self.h,0x3e)
      ay=high*256+low
      if ay>self.offset:
         ay=2*self.offset-ay
      else:
         ay=-ay
      ay=ay*self.norm

      return [ax,ay]


   def getGyro(self):
      high=self.pi.i2c_read_byte_data(self.h,0x43)
      low =self.pi.i2c_read_byte_data(self.h,0x44)
      if high>127:
         high=high-255
      gx=high*256+low
      gx=gx*self.gnorm

      high=self.pi.i2c_read_byte_data(self.h,0x45)
      low =self.pi.i2c_read_byte_data(self.h,0x46)
      if high>127:
         high=high-255
      gy=high*256+low
      gy=gy*self.gnorm

      high=self.pi.i2c_read_byte_data(self.h,0x47)
      low =self.pi.i2c_read_byte_data(self.h,0x48)
      if high>127:
         high=high-255
      gz=high*256+low
      gz=gz*self.gnorm

      return [gx,gy,gz]


   #def __del__(self):
   #   self.pi.i2c_close(self.h)

if __name__=='__main__':
   import time
   import sock4robot_7a as sk

   print_period=0.50
   send_period=0.020
   udp=sk.UDP_Send(sk.LOCAL_HOST,sk.MPU9150_PORT)
   udp2mpl53=sk.UDP_Send(sk.MLTPICAM_BASE_ADDR,sk.MPU2MPL53_PORT)

   asens = MPU9150()
   start=time.time()
   send_start=start
   init=start
   count=0
   while 1:
      try:
         data=asens.getAccel()
         data.extend(asens.getGyro())
         """
         count=count+1
         if time.time()-start>print_period:
            sec=time.time()-init
            rate=count/print_period
            print("\r                             ",end="")
            print("{0:5.1f}{1:6.1f}{2:8.4f}{3:8.4f}{4:8.4f}".format(sec,rate,data[0],data[1],data[4]),end="")
            start=time.time()
            count=0
         """

         udp.send(data)

         if time.time()-send_start>send_period:
            udp2mpl53.send(data)
            send_start=time.time()

         #time.sleep(send_period) # このsleepを無くすと、rate=350Hzほど
      except:
         pass
