import numpy as np
import math
import matplotlib.pyplot as plt
min = 0
max = 270


a = 120
b = 120
phi = 0
T = 1
omega = np.pi
#print("omega = ", omega)
#k = (max-min)/2000
#print("k = ",k)
#b = (5*min-max)/4
#print("b = ",b)
#print("test x = 500, y = 0:", k*500+b)
#print("test x = 2500, y = 270:", k*2500+b)
#y= 45
#print("y = 45, x = ", (y-b)/k)
#y= 135
#print("y = 135, x = ", (y-b)/k)

t = np.arange(0.0, 10.0, 0.01).reshape(1000,1)

Y = a*np.sin(phi+omega*t)+b

plt.plot(t, Y)
plt.show()


