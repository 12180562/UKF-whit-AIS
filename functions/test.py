import math
import numpy as np
from math import sqrt, sin, cos, atan2, pi
import CRI
# --- (질문에 주어진 CRI 클래스는 같다고 가정) ---

L    = 163.55
B    = 27.4
Xo   = 0
Yo   = 0
Xt   = 1000    # 1km
Yt   = 0
Co   = 0       # 동쪽
Ct   = math.pi # 서쪽(정면 대치)
Vo   = 5
Vt   = 5
scale = 70

my_cri_163 = CRI(L, B, Xo, Yo, Xt, Yt, Co, Ct, Vo, Vt, scale)
print("CRI =", my_cri_163.CRI())
