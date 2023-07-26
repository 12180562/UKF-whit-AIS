import kass_inha
from kass_inha import kass_inha



from math import sqrt, atan2,cos,sin,pi
from numpy import rad2deg


import numpy as np
import json


with open("parameter.json", "r") as param:
    Update_parameter = json.load(param)
parameter = Update_parameter


inha_input = {
  "cog": 210.0,
  "sog": 8.2,
  "latitude": 3046.99512,
  "longitude": 12932.48609,
  "latOfWayPoint": [
    34.982813,
    34.929951,
    34.966123,
    34.865359,
    34.832428,
    34.895181,
    34.895181,
    35.716109
  ],
  "longOfWayPoint": [
    128.974371,
    129.09702,
    129.237384,
    129.196501,
    129.062349,
    128.92242,
    128.92242,
    162.581832
  ],
  "numOfObject": 13,
  "idOfObject": [
    1000.0,
    5001.0,
    5002.0,
    5003.0,
    5004.0,
    5005.0,
    5006.0,
    5007.0,
    5008.0,
    5009.0,
    5010.0,
    5011.0,
    5012.0
  ],
  "cogOfObject": [
    1.883908,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.0
  ],
  "sogOfObject": [
    5.756628,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  ],
  "latOfObject": [
    35.447928,
    35.446721,
    35.455861,
    35.461408,
    35.477105,
    35.445567,
    35.4324,
    35.465134,
    35.474048,
    35.45409,
    35.476987,
    35.447834,
    35.476608
  ],
  "longOfObject": [
    129.406891,
    129.366684,
    129.406173,
    129.37796,
    129.386703,
    129.413193,
    129.408081,
    129.391952,
    129.406143,
    129.368866,
    129.38559,
    129.404907,
    129.40155
  ]
}


data = kass_inha(parameter)

path_out_inha = data.kass_inha(inha_input)
print(path_out_inha)
# data.setParamUpdate(parameter)
# path_out_inha2 = data.kass_inha(inha_input)




