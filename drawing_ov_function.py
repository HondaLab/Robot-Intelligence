import sympy

sympy.var('x')

alpha=1
beta=5
b=1
c=0

f = alpha*(sympy.tanh(beta*(x-b))+c)

sympy.plot(f,(x,0,2*b))
sympy.show()
