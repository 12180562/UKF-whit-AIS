import json
import pyproj

data = {'modifyWayPoint': False, 'numOfWayPoint': 15, 'latOfWayPoint': [35.30272631699444, 35.30275263398027, 35.30277895095748, 35.3028052679261, 35.3028315848861, 35.302857901837484, 35.30288421878027, 35.302910535714446, 35.30293685264001, 35.30296316955696, 35.30298948646531, 35.30301580336505, 35.30304212025617, 35.30306843713868, 35.303094754012584], 'longOfWayPoint': [129.2192619104422, 129.21929382090508, 129.21932573138864, 129.2193576418928, 129.2193895524177, 129.21942146296323, 129.2194533735294, 129.21948528411627, 129.2195171947238, 129.219549105352, 129.21958101600086, 129.2196129266704, 129.2196448373606, 129.21967674807144, 129.21970865880297], 'speedOfWayPoint': [0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04], 'ETAOfWayPoint': [6521.763, 6626.316, 6997.1], 'EDAOfWayPoint': [26838.358, 27268.618, 28794.467], 'error': False, 'errorCode': None, 'targetSpeed': 8.0, 'targetCourse': 44.701, 'cri': [], 'V_selected': [2.925023343770077, 2.8946691483484117]}
json_string = json.dumps(data)

decoded_data =json.loads(json_string)

# print(decodㄴed_data)








def latlong_to_utm(latitude, longitude, northern_hemisphere=True):
        utm_zone = pyproj.Proj(proj='utm', zone=52, datum='WGS84')
        utm_east,utm_north  = utm_zone(longitude, latitude)

        return utm_north, utm_east
    
def utm_to_latlong(utm_north, utm_east, northern_hemisphere=True):
    utm_zone = pyproj.Proj(proj='utm', zone=52, datum='WGS84', ellps='WGS84')
    longitude,latitude  = utm_zone(utm_east, utm_north, inverse=True)

    return latitude, longitude


a, b = latlong_to_utm(35.50450498164144, 129.365306637498)
print(a, b)
c, d = utm_to_latlong(a, b)

print(c, d)