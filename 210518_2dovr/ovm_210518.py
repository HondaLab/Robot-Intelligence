#!/usr/bin/python3
"""
作成日：2020/10/31
更新日：2021/04/16
作成者：20043060 山田 将司
###説明###
ssr系ロボット用2次元最適速度モデル
ovm.pyに関しては特に改造する必要無し．
"""
#モジュールインポート
import math

# 定数定義
MAX = 3 #自身を除いたロボット最大数

# 平方根計算，math.と書きたくないため，自作
def sqrt_1(x):
    return math.sqrt(x)
def sqrt_2(x,y):
    return math.sqrt(x*x + y*y)

# 絶対値計算、math.と書きたくないため，自作
def fabs(x):
    return math.fabs(x)

# sin計算，math.と書きたくないため，自作
def sin(x):
    return math.sin(x)

# cos計算，math.と書きたくないため，自作
def cos(x):
    return math.cos(x)

# arc-cos計算，math.と書きたくないため，自作
def acos(x):
    try:
        return math.acos(x)
    except:
        return 0.0

# hyperbolic-tan計算，math.と書きたくないため，自作
def tanh(x):
    return math.tanh(x)

# 外積計算
def outer_product(a1,a2,b1,b2):
    outer_z = (a1*b2 - a2*b1) #OK
    return outer_z

# シグナム関数(符号関数)
def sgn(x):
    if x > 0:
        sign = 1
    if x == 0:
        sign = 0
    if x < 0:
        sign = -1
    return sign

# Optimal Velocity model
class Optimal_Velocity_class:
    #インスタンス生成
    def __init__(self,parm):
        self.vs = parm[0]    #相互作用なしで動き続けるための項 
        self.a = parm[1]     #感応度
        self.alpha = parm[2] #最高速度決定
        self.beta = parm[3]  #αβで最適速度関数の変化率を決定
        self.b = parm[4]     #変曲点のx座標(ロボットの車頭距離にする)
        self.c = parm[5]     #前進後退の割合決定
        self.vx = 0.1
        self.vy = 0.1
        self.d = 0.06
        self.conversion_v = 1 / 0.525

    def calc(self,distance,theta,dt):

        f_rkj = self.alpha*(tanh(self.beta*(distance - self.b)) + self.c ) #(3)式
        
        #print("\r f_rkj=%7.3f" % f_rkj,end="")

        nx = cos(theta)
        ny = sin(theta)
        #print("nx=%7.3f" % nx,end="")
        #print(" ny=%7.3f" % ny,end="")

        large_v_x = (1+cos(theta)) * f_rkj * nx
        large_v_y = (1+cos(theta)) * f_rkj * ny
        #print("\r large_v_x=%7.3f" % large_v_x,end="")
        #print(" large_v_y=%7.3f" % large_v_y,end="")

        ax = self.a * (self.vs + large_v_x - self.vx)
        ay = self.a * (self.vs + large_v_y - self.vy)
        #print(" vx=%7.3f" % self.vx,end="")
        #print(" vy=%7.3f" % self.vy)

        vx_next = self.vx + dt * ax
        vy_next = self.vy + dt * ay

        v = sqrt_2(self.vx,self.vy) 
        v_next = sqrt_2(vx_next,vy_next)
        #print("\r              v=%7.3f" % v,end="")
        #print(" v_next=%7.3f" % v_next,end="")

        out_z = vx_next * self.vy - vy_next * self.vx
        inner_v = self.vx * vx_next + self.vy * vy_next

        in_acos = (inner_v/(v*v_next))
        #print("\r acosの内部",in_acos,end="")
        d_theta = sgn(out_z)*acos(inner_v/(v*v_next))
        #print("d_theta =  %6.4f" % d_theta)
        right = v - (self.d * ( d_theta / dt) ) #* self.conversion_v
        left = v + (self.d * ( d_theta / dt) )  #* self.conversion_v

        #print("left=%7.3f" % left,end="")
        #print(" right=%7.3f" % right)

        self.vx = vx_next #OK
        self.vy = vy_next #OK
        
        left = left/(2.0*self.alpha*(1+self.c))
        right = right/(2.0*self.alpha*(1+self.c))
        
        return left,right,d_theta
