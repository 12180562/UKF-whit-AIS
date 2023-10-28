import kass_inha
from kass_inha import kass_inha



from math import sqrt, atan2,cos,sin,pi
from numpy import rad2deg


import numpy as np
import json
import pyproj


def latlon_to_utm(latitude, longitude, zone_number):
    utm_zone = pyproj.Proj(proj='utm', zone=zone_number, datum='WGS84')
    utm_easting, utm_northing = utm_zone(longitude, latitude)

    return utm_easting, utm_northing


with open("parameter.json", "r") as param:
    Update_parameter = json.load(param)
parameter = Update_parameter


inha_input = {
  "cog": 41.75,
  "sog": 15.43,
  "degreeOfAxis" : [0, 0, 90.72],
  "latitude": 33.9299805555,  #35.504645,
  "longitude": 17.5846405555, #129.36543,
  "latOfWayPoint": [
    33.93319666,
    # 35.502873    
  ],
  "longOfWayPoint": [
    127.588063888,
    # 129.357958
  ],
  "numOfObject": 1,
  "idOfObject": [
    4400
  ],
  "cogOfObject": [
    90.72
  ],
  "sogOfObject": [
    0.7
  ],
  "latOfObject": [ 
    33.9837977814753,
  ],
  "longOfObject": [
    127.627241224811
  ],

  "nWptsID" : 0
}


# print(parameter)
data = kass_inha(parameter)

path_out_inha = data.kass_inha(inha_input)
print(path_out_inha)
# data.setParamUpdate(parameter)
# path_out_inha2 = data.kass_inha(inha_input)


