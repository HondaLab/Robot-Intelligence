#!/usr/bin/python3
# sound_and_speak_xx.py by Yasushi Honda 2019 1/3
#------------------------------------------------

import ev3dev.ev3 as ev3
import time
import random
import datetime

if __name__=='__main__':
   p=ev3.LegoPort(ev3.INPUT_4)
   p.set_device='lego-nxt-sound'
   ss = ev3.SoundSensor(ev3.INPUT_4)

   ts=ev3.TouchSensor(ev3.INPUT_3)
   sp=ev3.Sound()
   sp.speak('Hi. I can speak the current time.').wait()

   talk=[]
   talk.append('Yes. I can hear you, but speak more loudly.')

   leds = ev3.Leds()
   leds.set_color(ev3.Leds.LEFT, ev3.Leds.GREEN,pct=1.0)
   leds.set_color(ev3.Leds.RIGHT, ev3.Leds.YELLOW,pct=1.0)

   print('[[ Push touch sensor to stop.]]')
   #print('[[ snd ]]')
   while ts.value()==0:
      snd=967-ss.value()
      #print("\r %4d" % snd, end='')

      br=snd*0.040
      try:
         leds.set_color(ev3.Leds.LEFT,ev3.Leds.GREEN,pct=br)
         leds.set_color(ev3.Leds.RIGHT,ev3.Leds.ORANGE,pct=br)
      except:
         pass

      if 1<snd and snd<=7:
         #idx = random.randint(0,len(talk)-1)
         sp.speak(talk[0]).wait()
      elif 7<snd:
         date_now = datetime.datetime.now()
         talk_now = 'Now Its '
         weekday=date_now.weekday()
         if weekday==0:
            talk_now += 'Monday '
         elif weekday==1:
            talk_now += 'Tuesday '
         elif weekday==2:
            talk_now += 'Wednesday '
         elif weekday==3:
            talk_now += 'Thursday '
         elif weekday==4:
            talk_now += 'Friday '
         elif weekday==5:
            talk_now += 'Saturday '
         elif weekday==6:
            talk_now += 'Sunday '
         talk_now += str(date_now.hour) + ' '
         talk_now += str(date_now.minute) + ' '
         talk_now += str(date_now.second)
         print(talk_now)
         sp.speak(talk_now).wait()
 
      time.sleep(0.1)
   print('\n[[ See you again! ]]')
   sp.speak('See you again!').wait()


