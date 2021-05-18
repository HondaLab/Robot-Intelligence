# エンター押さなくても次に進むキーボード入力するためのコード
import sys
import termios
import fcntl 
import os

class Keyboard():
   def __init__(self):
      #標準入力のファイルディスクリプタを取得
      self.fd = sys.stdin.fileno()
   
      #fdの端末属性をゲットする
      #newに変更を加えて、適応する。oldは、後で元に戻すため
      self.old = termios.tcgetattr(self.fd)
      self.new = termios.tcgetattr(self.fd)
  
      #new[3]はlflags
      #ICANON(カノニカルモードのフラグ)を外す
      self.new[3] &= ~termios.ICANON
      #ECHO(入力された文字を表示するか否かのフラグ)を外す
      self.new[3] &= ~termios.ECHO
   
      # stdinをNONBLOCKに設定
      fcntl_old = fcntl.fcntl(self.fd, fcntl.F_GETFL)
      fcntl.fcntl(self.fd, fcntl.F_SETFL, fcntl_old | os.O_NONBLOCK)
   
   def __del__(self):
      termios.tcsetattr(self.fd, termios.TCSANOW, self.old)

   def read(self):
      # 書き換えたnewをfdに適応する
      termios.tcsetattr(self.fd, termios.TCSANOW, self.new)
      # キーボードから入力を受ける。
      # lfalgsが書き換えられているので、エンターを押さなくても次に進む。echoもしない
      return sys.stdin.read(1)         
   
if __name__=='__main__':
   key = Keyboard()
   ch="c"
   print("Input q to stop.")
   while(ch!="q"):
      try:
         ch = key.read() 
         print(ch)
      except:
         del key
