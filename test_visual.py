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
  "cog": 110.0,
  "sog": 5.0,
  "degreeOfAxis" : [0, 0, 228.4],
  "latitude": 35.504516666667,  #35.504645,
  "longitude": 129.36537166666668, #129.36543,
  "latOfWayPoint": [
    35.501226,
    35.502873    
  ],
  "longOfWayPoint": [
    129.360741,
    129.357958
  ],
  "numOfObject": 2,
  "idOfObject": [
    0.0,
    15001.0
  ],
  "cogOfObject": [
    0.0,
    41.1659468231375
  ],
  "sogOfObject": [
    0.0,
    9.88703295953533
  ],
  "latOfObject": [
    0.0,
    35.5045180468848
  ],
  "longOfObject": [
    124.511256,
    129.365460337617
  ],

  "nWptsID" : 0
}




data = kass_inha(parameter)


OS_position_matrix = np.array([[inha_input["latitude"],inha_input["longitude"],inha_input["cog"]]])
TS_position_matrix = np.array([[inha_input["latOfObject"][0],inha_input["longOfObject"][0],41.1659468231375]])
move_TS = np.array([[inha_input["latOfObject"][0],inha_input["longOfObject"][0],41.1659468231375]])

for i in range(50):
    path_out_inha = data.kass_inha(inha_input)
    # print(path_out_inha["targetCourse"])
    move = [cos(deg2rad(path_out_inha["targetCourse"]))*path_out_inha["speedOfWayPoint"][0],sin(deg2rad(path_out_inha["targetCourse"]))*path_out_inha["speedOfWayPoint"][0],0]
    move_gnss = gnss_convert(move,[inha_input["latitude"],inha_input["longitude"],0])
    inha_input["latitude"] = move_gnss[0] 
    inha_input["longitude"] = move_gnss[1]
    print(path_out_inha["targetCourse"])
    # if path_out_inha["targetCourse"] >= inha_input["cog"]+10:
    #   inha_input["cog"] = inha_input["cog"]+10
    # elif path_out_inha["targetCourse"] <= inha_input["cog"]-10:
    #   inha_input["cog"] = inha_input["cog"]-10
    # else:
    inha_input["cog"] = path_out_inha["targetCourse"]

    move_TS = np.array([[move_TS[0][0] + 5/111180, move_TS[0][1], 0]])

    new_OS_position_matrix = np.array([[inha_input["latitude"],inha_input["longitude"],inha_input["cog"]]])
    OS_position_matrix = np.concatenate((OS_position_matrix, new_OS_position_matrix), axis = 0)
    TS_position_matrix = np.concatenate((TS_position_matrix,move_TS))

    # new_TS_position_matrix = np.array([])

plt.figure()
# print(OS_position_matrix)
plt.plot(OS_position_matrix[:,1],OS_position_matrix[:,0])
plt.plot(TS_position_matrix[:,1],TS_position_matrix[:,0])
plt.show()

