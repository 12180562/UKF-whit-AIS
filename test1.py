from numpy import deg2rad, rad2deg
from math import *
from numpy import rad2deg

import numpy as np
import json

def setParamUpdate():
    with open("parameter.json", 'r') as param:
        Update_parameter = json.load(param)
        
    parameter = Update_parameter
    
    # print(parameter)
    return parameter

a = setParamUpdate()

print(a)



# def waypoint_generator(OS):
#     wp_x = []
#     wp_y = []
#     OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])
#     for i in range(1, 16):
#         V_selected = [-0.28470095, -1.06251841]
#         wp = OS_X + V_selected * np.array([i])
#         wp_x.append(wp[0]) 
#         wp_y.append(wp[1])

#     return wp_x, wp_y

# def eta_eda_assumption(wp, OS, target_U):
#     eta = []
#     eda = []
#     target_U = 1.5
    
#     OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])
#     for i in range(15):
#         eda_x = wp[0][i] - OS['Pos_X']
#         eda_y = wp[1][i] - OS['Pos_Y']
#         distance = sqrt(eda_x**2 + eda_y**2)
#         eda.append(distance)
#         time = distance / target_U
#         eta.append(time)
        
#     return eta, eda


# def desired_value_assummption(wp):
#     desired_spd_list = []
#     desired_heading_list = []
#     for i in range(15):
#         U_des = sqrt(wp[0][i]**2 + wp[1][i]**2)
#         target_head = rad2deg(atan2(wp[1][i], wp[0][i])) 
        
#         desired_heading = target_head
#         desired_spd = U_des
#         desired_spd_list.append(desired_spd)
#         desired_heading_list.append(desired_heading)
        
#     return desired_spd_list, desired_heading_list
               

# OS = {'shipID': 1000, 'Pos_X': 3046.99512, 'Pos_Y': 12932.48609, 'Vel_U': 1.271798972336573, 'Heading': 210.0, 'V_x': -1.1014102185504147, 'V_y': -0.6358994861682866}
# target_U = 1.0
# wp = waypoint_generator(OS)
# print('wp:', wp)
# print(len(wp))


# eta, eda = eta_eda_assumption(wp, OS, target_U)
# print('eta:', eta)
# print('eda:', eda)

# desired_spd = desired_value_assummption(wp)

# print(desired_spd)









        