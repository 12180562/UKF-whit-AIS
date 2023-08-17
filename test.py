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
  "cog": 184.32,
  "sog": 9.37,
  "degreeOfAxis" : [0, 0, 228.4],
  "latitude": 35.585754214824256,  #35.504645,
  "longitude": 129.35889378536336, #129.36543,
  "latOfWayPoint": [
    35.477006,
    35.502873    
  ],
  "longOfWayPoint": [
    129.464499,
    129.357958
  ],
  "numOfObject": 2,
  "idOfObject": [
    0.0,
    15001.0
  ],
  "cogOfObject": [
    0.0,
    5.189
  ],
  "sogOfObject": [
    0.0,
    10.0
  ],
  "latOfObject": [ 
    0.0,
    #35.5045180468848
    35.4959599727472,
  ],
  "longOfObject": [
    124.511256,
    #129.365460337617
    129.465400152324,
  ],

  "nWptsID" : 0
}


# print(parameter)
data = kass_inha(parameter)

path_out_inha = data.kass_inha(inha_input)
print(path_out_inha)
# data.setParamUpdate(parameter)
# path_out_inha2 = data.kass_inha(inha_input)


