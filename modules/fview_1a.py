def fview(func,x1,x2):
    #%matplotlib inline
    import matplotlib.pyplot as plt
    px=[]
    py=[]
    dx=(x2-x1)/400
    x=x1
    while x<x2:
        px.append(x)
        py.append(func(x))
        x+=dx
        
    plt.grid()
    # 関数を描画
    plt.plot(px,py) 
    plt.show()

if __name__=="__main__":
    from sympy import *    
    def ovf(r):

        # 係数を与える
        alpha=4
        beta=0.008
        alpha2=4
        beta2=10
        b=200
        c=0.0

        # 関数を決める
        f= alpha*tanh(beta*(r-b)) + alpha2*tanh(beta2*(r-b)) + c
    
        return f
    
    fview(ovf,-100,600)
