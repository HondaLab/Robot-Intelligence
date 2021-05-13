import numpy as np
import math

data = np.genfromtxt("ssr101_10_take1.dat") #datファイルを読み込む

xs = data[0,2] #xの始点
ys = data[0,3] #yの始点
xg = data[-1,2] #xの終点
yg = data[-1,3] #yの終点
xm = (xs+xg)/2 #xsとxgの中点
ym = (ys+yg)/2 #ysとygの中点
#print(xs, ys, xg, yg, xm, ym)

#２つの座標を通る直線の傾きと切片を求める
a0 = (yg-ys)/(xg-xs) #傾き
b0 = ys-a0*xs #切片
#print(a0,b0)

#スタートとゴールの垂直二等分線の傾きと切片
a = -1/a0 
b = ym-a*xm
#print(a, b)

#弦の直線の方程式を出す


#ここから下完成してない、xc,ycの計算式が正しくない
#xc = (xg**2-xs**2+2*ys*b+yg*(yg-2*b))/2*(xg+a*yg-xs-a*ys) #円の中心のx座標
#yc = a*xc+b #円の中心のy座標
print(xc,yc)

r = math.sqrt((xs-xc)**2+(ys-yc)**2) #半径


#ここより下、曲率を求める
#--------------------------------------------------------------------

CG =   #終点と中心を結ぶ弦
GS = math.sqrt((xg-xs)**2+(yg-ys)**2)  #始点と中心を結ぶ弦

ΔS = (yg-ys)/(xg-xs) #曲線に沿って始点から終点へ進んだ距離
Δα = math.acos((r**2+CG**2-GS**2)/2*r*CG) #始点と終点から円の中心に引いた直線が成す角度

R = abs(ΔS/Δα) #曲率半径
k = 1/R #曲率
#print("曲率半径>>>"R, "曲率>>>"k)
