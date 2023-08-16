import kass_inha
from kass_inha import kass_inha
import matplotlib.pyplot as plt



from math import sqrt, atan2,cos,sin,pi
from numpy import rad2deg,deg2rad


import numpy as np
import json

import pymap3d as pm

def enu_convert(gnss,origin):
    e, n, u = pm.geodetic2enu(gnss[0], gnss[1], gnss[2], origin[0], origin[1], origin[2])
    return e, n, u

def gnss_convert(enu,origin):
    lat,long,alt = pm.enu2geodetic(enu[0], enu[1], enu[2], origin[0], origin[1], origin[2], ell = None)
    return lat, long, alt


with open("parameter.json", "r") as param:
    Update_parameter = json.load(param)
parameter = Update_parameter

origin = [35.497384,129.385912,0]
inha_input = {
  "cog": 1.4,
  "sog": 10.0,
  "degreeOfAxis" : [0, 0, 228.4],
  "latitude": 35.487445, #35.504516666667,  #35.504645,
  "longitude": 129.45815166666668, #129.36537166666668, #129.36543,
  "latOfWayPoint": [
    # 35.501226,
    # 35.502873
    35.503852,35.503795,35.494698,35.494797,35.485712,35.485712,35.476774,35.476823,35.50398,35.503782,35.494844,35.494813,35.485737,35.485798,35.476877,35.47689    
  ],
  "longOfWayPoint": [
    # 129.360741,
    # 129.357958
    129.458903,129.469974,129.470159,129.459028,129.459028,129.470099,129.470099,129.45921,129.458786,129.469978,129.470095,129.459016,129.459078,129.470098,129.470049,129.459173
  ],
  "numOfObject": 2,
  "idOfObject": [
    0.0,
    15001.0
  ],
  "cogOfObject": [
    270,
    270
  ],
  "sogOfObject": [
    0.0,
    0.0
  ],
  "latOfObject": [
    0.0,
    #35.5045180468848
    35.5027042410039,
  ],
  "longOfObject": [
    124.511256,
    #129.365460337617
    129.363478983698,
  ],

  "nWptsID" : 0
}




data = kass_inha(parameter)


OS_position_matrix = np.array([[inha_input["latitude"],inha_input["longitude"],inha_input["cog"]]])
# TS_position_matrix = np.array([[inha_input["latOfObject"][1],inha_input["longOfObject"][1],41.1659468231375]])

for i in range(1100):
    path_out_inha = data.kass_inha(inha_input)
    # print(path_out_inha["targetCourse"])
    move = [sin(deg2rad(path_out_inha["targetCourse"]))*path_out_inha["speedOfWayPoint"][0],cos(deg2rad(path_out_inha["targetCourse"]))*path_out_inha["speedOfWayPoint"][0],0]
    move_gnss = gnss_convert(move,[inha_input["latitude"],inha_input["longitude"],0])
    inha_input["latitude"] = move_gnss[0] 
    inha_input["longitude"] = move_gnss[1]
    inha_input["cog"] = path_out_inha["targetCourse"]

    # move_TS = [sin(deg2rad(inha_input["cogOfObject"][1]))*inha_input["sogOfObject"][1],cos(deg2rad(inha_input["cogOfObject"][1]))*inha_input["sogOfObject"][1],0]
    # move_gnss_TS = gnss_convert(move_TS,[inha_input["latOfObject"][1],inha_input["longOfObject"][1],0])
    # inha_input["latOfObject"][1] = move_gnss_TS[0] 
    # inha_input["longOfObject"][1] = move_gnss_TS[1]

    new_OS_position_matrix = np.array([[inha_input["latitude"],inha_input["longitude"],inha_input["cog"]]])
    # new_TS_position_matrix = np.array([[inha_input["latOfObject"][1],inha_input["longOfObject"][1],inha_input["cogOfObject"][1]]])
    OS_position_matrix = np.concatenate((OS_position_matrix, new_OS_position_matrix), axis = 0)
    # TS_position_matrix = np.concatenate((TS_position_matrix, new_TS_position_matrix))
    # print(inha_input["cogOfObject"][1])
    os_position = enu_convert([inha_input["latitude"],inha_input["longitude"],0],[inha_input["latitude"],inha_input["longitude"],0])
    wp_position = enu_convert([inha_input["latOfWayPoint"][inha_input["nWptsID"]],inha_input["longOfWayPoint"][inha_input["nWptsID"]],0],[inha_input["latitude"],inha_input["longitude"],0])
    local_goal_EDA = sqrt((os_position[0]-wp_position[0])**2+(os_position[1]-wp_position[1])**2)
    # print(local_goal_EDA)
    if local_goal_EDA < 44:
# 만약 `reach criterion`와 거리 비교를 통해 waypoint 도달하였다면, 
# 앞서 정의한 `waypint 도달 유무 확인용 flag`를 `True`로 바꾸어 `while`문 종료
      inha_input["nWptsID"] = (inha_input["nWptsID"] + 1) % len(inha_input["latOfWayPoint"])
      print(inha_input["nWptsID"])

    # print(path_out_inha["targetCourse"])

    # new_TS_position_matrix = np.array([])
# print(OS_position_matrix)
# print(TS_position_matrix)

plt.figure()
# print(OS_position_matrix)
plt.plot(OS_position_matrix[:,1],OS_position_matrix[:,0])
plt.plot(inha_input["longOfWayPoint"],inha_input["latOfWayPoint"])
plt.scatter(inha_input["longOfObject"][1],inha_input["latOfObject"][1])
# plt.plot(TS_position_matrix[:,1],TS_position_matrix[:,0])
plt.show()

