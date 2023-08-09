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
  "cog": 110.0,
  "sog": 0.0,
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
    40.053
  ],
  "sogOfObject": [
    0.0,
    9.96
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


# print(parameter)
data = kass_inha(parameter)

path_out_inha = data.kass_inha(inha_input)
print(path_out_inha)
# data.setParamUpdate(parameter)
# path_out_inha2 = data.kass_inha(inha_input)


