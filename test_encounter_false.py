#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
  "nWptsID": 0,
  "cog": 213.69,
  "sog": 0.04,
  "degreeOfAxis": [
    0,
    0,
    230.4
  ],
  "latitude": 37.450167,
  "longitude": 126.653557, # 인하대학교
  "latOfWayPoint": [
    35.474149,
    35.474927,
    35.485253
  ],
  "longOfWayPoint": [
    129.427783,
    129.433516,
    129.444692
  ],
  "numOfObject": 6,
  "idOfObject": [
    0.0,
    102.0,  # 서울
    1031.0, # 경기
    1042.0, # 대전
    1051.0, # 부산
    1064.0  # 제주
  ],
  "cogOfObject": [
    0.0,
    247.5,
    17.0,
    158.899994,
    221.699997,
    56.4555
  ],
  "sogOfObject": [
    0.0,
    0.0,
    0.0,
    0.0,
    0.2,
    0.3
  ],
  "latOfObject": [
    0.0,
    37.586290,
    37.406663,
    36.327702,
    35.158111,
    33.361647
  ],
  "longOfObject": [
    0.0,
    126.974846,
    127.101871,
    127.427298,
    129.158212,
    126.529211
  ]
}

# print(parameter)
data = kass_inha(parameter)

path_out_inha = data.kass_inha(inha_input)
print(path_out_inha)
# data.setParamUpdate(parameter)
# path_out_inha2 = data.kass_inha(inha_input)