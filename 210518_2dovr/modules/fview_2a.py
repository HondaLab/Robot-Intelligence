def fview(func,xlabel,x1,x2,ylabel):
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
    #plt.show()
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.savefig('figure.png')

if __name__=="__main__":
    import math
    def ovf(r):

        # 係数を与える
        alpha=4
        beta=0.008
        alpha2=4
        beta2=10
        b=200
        c=0.0

        # 関数を決める
        f= alpha*math.tanh(beta*(r-b)) + alpha2*math.tanh(beta2*(r-b)) + c
    
        return f
    
    fview(ovf,'r',-100,600,'ovf')
