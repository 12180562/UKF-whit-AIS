import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

Rf1, Ra1, Rs1, Rp1 = 6.359451948349684, 3.7398933899819955, 3.0497223301108316, 2.4372917475831235
Rf2, Ra2, Rs2, Rp2 = 5.0511100785740926, 3.7398933899819955, 4.282385556144601, 3.361789167108451


x1 = np.linspace(0, Rs1, 100)
y1 =(Rf1**2-(Rf1**2/Rs1**2)*x1**2)**0.5
y2 =-(Ra1**2-(Ra1**2/Rs1**2)*x1**2)**0.5

x2 = np.linspace(-Rp1, 0, 100)
y3 =(Rf1**2-(Rf1**2/Rp1**2)*x2**2)**0.5
y4 =-(Ra1**2-(Ra1**2/Rp1**2)*x2**2)**0.5

x3 = np.linspace(0, Rs2, 100)
y5 =(Rf2**2-(Rf2**2/Rs2**2)*x3**2)**0.5
y6 =-(Ra2**2-(Ra2**2/Rs2**2)*x3**2)**0.5

x4 = np.linspace(-Rp2, 0, 100)
y7 =(Rf2**2-(Rf2**2/Rp2**2)*x4**2)**0.5
y8 =-(Ra2**2-(Ra2**2/Rp2**2)*x4**2)**0.5

plt.figure(figsize=(8, 8))
plt.hlines(0,-10,+10,color='black',linestyle='--')##(y축 위치, x축 시작점, x축 끝점)
plt.vlines(0,-10,+10,color='black',linestyle='--')##(x축 위치, y축 시작점, y축 끝점)
plt.xticks(np.arange(-10, 12, 2))
plt.yticks(np.arange(-10, 12, 2))
plt.grid(linestyle='-.')

# plt.scatter(0,0,c='b')
# plt.text(-c-2.5,-2,'('+str(round(-c,1))+', 0)')
# plt.text(c-1.5,-2,'('+str(round(c,1))+', 0)')
# plt.text(-a,a-1,'e='+str(round(c/a,2)),size = 13)

plt.plot(x1,y1,c='r')
plt.plot(x1,y2,c='r')
plt.plot(x2,y3,c='r')
plt.plot(x2,y4,c='r')

plt.plot(x3,y5,c='b')
plt.plot(x3,y6,c='b')
plt.plot(x4,y7,c='b')
plt.plot(x4,y8,c='b')

plt.xlim(-11, 11)
plt.ylim(-11, 11)

plt.show()