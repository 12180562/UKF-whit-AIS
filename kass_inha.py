from functions.Inha_DataProcess import Inha_dataProcess
from functions.Inha_VelocityObstacle import VO_module

from math import sqrt, atan2
from numpy import rad2deg

import numpy as np
import time
import yaml

with open('/home/phlmy/catkin_ws/src/kass_inha/params/main_parameter.yaml') as f:
    parameter = yaml.full_load(f)
    
    

        
class data_inNout:
    def __init__(self):
        ###   frm_info 정의 ###
        self.m_nPacketCode = []
        self.m_nShipID = []
        self.m_fltHeading = []
        self.m_fltDriftangle = []
        self.m_fltShipTime = []
        self.m_fltRudderAngleFeedPORT = []
        self.m_fltRudderAngleFeedSTBD = []
        self.m_fltPropellerRPSFeedPORT = []
        self.m_fltPropellerRPSFeedSTBD = []
        self.m_fltFOGvel_rollx = []
        self.m_fltFOGvel_pitchy = []
        self.m_fltFOGvel_yawz = []
        self.m_fltFOGang_rollx = []
        self.m_fltFOGang_pitchy = []
        self.m_fltFOGang_yawz = []
        self.m_fltFOGvel_yawzG = []
        self.m_fltFOGang_yawzG = []
        self.m_fltIncl_heelx = []
        self.m_fltIncl_trimy = []
        self.m_fltVel_U = []
        self.m_fltPos_X = []
        self.m_fltPos_Y = []
        ### vessel_node init 정의 ###
        self.ship_ID = []
        self.waypoint_idx = 0
        self.len_waypoint_info = 0
        self.waypoint_dict = dict()
        self.ts_spd_dict = dict()
        self.TS_WP_idex = []
        self.target_heading_list = []
        
        self.static_obstacle_info = []
        self.static_point_info = []
        
        
        

        