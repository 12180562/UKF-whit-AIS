import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

    #     static_obstacle = pd.read_csv('/home/hyoguen/catkin_ws/src/inha_modules/src/static_obstacle.csv',header = None)
    #     # header를 none으로 지정하면 칼럼도 숫자로 저장

static = pd.read_csv("/home/hyogeun/STOB_avoidance_data/square_obstacle/static_delta_t.csv",header=None)
variable = pd.read_csv("/home/hyogeun/STOB_avoidance_data/square_obstacle/variable_delta_t.csv",header=None)
opposite_variable = pd.read_csv("/home/hyogeun/STOB_avoidance_data/square_obstacle/opposite_variable_delta_t.csv",header=None)

static_x = np.array(static[1])
static_y = np.array(static[0])
variable_x = np.array(variable[1])
variable_y = np.array(variable[0])
oppossite_x = np.array(opposite_variable[1])
oppossite_y = np.array(opposite_variable[0])

figure = plt.figure()
plt.plot(static_x,static_y, c='g',label = 'static VFH')
plt.plot(variable_x,variable_y, c ='r', label = 'variable VFH')
plt.plot(oppossite_x,oppossite_y, c = 'b', label = 'opposite VFH')
plt.plot([-30,30,30,-30,-30],[80,80,120,120,80], c='black', label= 'obstacles')
plt.legend()
plt.axes().set_aspect('equal')
plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.show()