#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.CRI import CRI
from numpy import deg2rad, rad2deg
from math import sin, cos, pi, sqrt, atan2
import numpy as np
import rospy

class Inha_dataProcess:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(
        self, 
        ship_ID, 
        Pos_X, 
        Pos_Y, 
        Vel_U, 
        Heading, 
        waypoint_dict, 
        TS_WP_index=[], 
        ):        

        self.ship = {
            'Ship_ID' : [],
            'Pos_X' : [],
            'Pos_Y' : [],
            'Vel_U' : [],
            'Heading' : [],
        }
        
        self.ship_dic= {}

        self.ship_ID = ship_ID
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Vel_U = Vel_U
        self.Heading = Heading

        self.waypoint_dict = waypoint_dict

        self.TS_WP_index = TS_WP_index
        
        self.pre_distance = {}
        self.pre_status = {}


    def ship_list_container(self, OS_ID):
        ''' 
            Subscribe한 선박의 운항정보를 dictionary로 저장 
        
            Return : 
                ship_list_dic
                ship_ID
        '''
        for i in range(len(self.ship_ID)):
            # TODO: Why separating...? 
            #       Just allocating has no difference with it at all in my head...
            index_ship = self.ship_ID[i]
            if index_ship == OS_ID:
                self.ship_dic[OS_ID]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Pos_X' : self.Pos_X[i],
                    'Pos_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    }

            else:
                self.ship_dic[index_ship]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Pos_X' : self.Pos_X[i],
                    'Pos_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    }

        return self.ship_dic, self.ship_ID


    def classify_OS_TS(self, ship_dic, ship_ID, OS_ID):
        ''' 자선과 타선의 운항정보 분리
        
        Return :
            OS_list, TS_list // (dataframe)
        '''
        if len(ship_ID) == 1:
            OS_list = ship_dic[OS_ID]

            TS_list = None

        else:
            for i in range(len(ship_ID)):
                if ship_ID[i] == OS_ID:
                    OS_list = ship_dic[OS_ID]

            
            TS_list = ship_dic.copy()
            # FIXME: `OS_ID` does not exist in `TS_list` when processing TSs? The `OS_ID` is not the "OS"????
            del(TS_list[OS_ID])

        return OS_list, TS_list
    
    def U_to_vector_V(self, U, deg):
        ''' Heading angle을 지구좌표계 기준의 속도벡터로 변환

        Return:
            Vx, Vy  [m/s]     
        '''
        psi = deg2rad(deg) ##강제로 xy좌표에서 NED좌표로 변환
        
        V_x = U * cos(psi)
        V_y = U * sin(psi)

        return V_x, V_y

    def waypoint_generator(self, OS, V_selected, dt):
        ''' `V_des` 방향 벡터를 기준으로 10초뒤 point를 waypoint 생성
        
        Return :
            wp_x, wp_y [m]
        '''

        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])

        wp = OS_X + V_selected * dt

        return wp

    def eta_eda_assumption(self, WP, OS, target_U):
        ''' 목적지까지 도달 예상 시간 및 거리
        
        Return:
            eta [t], eda [m]
        '''
        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])

        distance = np.linalg.norm(WP - OS_X)

        eta = distance/ target_U
        eda = distance

        return eta, eda

    def desired_value_assumption(self, V_des):
        ''' 목적지까지 향하기 위한 Desired heading angle
        
        Return :
            desired_speed [m/s], desired_heading [deg]
        '''
        U_des = sqrt(V_des[0]**2 + V_des[1]**2)
        target_head = rad2deg(atan2(V_des[1], V_des[0])) ## NED 좌표계 기준으로 목적지를 바라보는 방향각

        desired_heading = target_head        
        desired_spd = U_des

        return desired_spd, desired_heading

    def vectorV_to_goal(self, TS, ts_ID, goal, V_max):
        """ compute desired vel to goal
        
        Inputs :
            X = [OS_x, OS_y] / goal = [X, Y] / V = [uBody, vBody]        
        Returns:
            목표지점으로 가기 위한 속도(방향)벡터   [V_x, V_y]
        Notes :
            `pandas.to_dict()`호출 방법은 `a.['key'][loc]`임!
        """
        Pos_x = TS[ts_ID]['Pos_X']
        Pos_y = TS[ts_ID]['Pos_Y']
        dif_x = [goal[0] - Pos_x, goal[1] - Pos_y]
        norm = self.distance(dif_x, [0, 0])
        V_des = [dif_x[k] * V_max[k] / norm for k in range(2)]

        return V_des

    def distance(self, pose1, pose2):
        """ compute Euclidean distance for 2D 
        
        Input :
            pose1 = [x, y] , pose2 = [x, y]
        Note:
            거리 뒤에 `0.0001`뒤를 더하는 이유는, normalize 과정에서 `zero division error`를 없애기 위함임
            if) pose1 & pose2가 같다면 `0`이 나오므로 error 발생
        """
        
        return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2) + 0.0001 # = sqrt(np.dot(pose1, pose2))

    def CRI_cal(self, OS, TS):
        cri = CRI(rospy.get_param("shipInfo_all/OS_info/ship_L"), rospy.get_param("shipInfo_all/OS_info/ship_B"), OS['Pos_X'], OS['Pos_Y'], TS['Pos_X'], TS['Pos_Y'], deg2rad(OS['Heading']), deg2rad(TS['Heading']), OS['Vel_U'], TS['Vel_U'])
        Vtx = cri.Vtx()
        Vty = cri.Vty()
        RD = cri.RD()
        HAD = rad2deg(cri.HAD())
        RB = rad2deg(cri.RB())
        UDCPA = cri.UDCPA()
        UTCPA = cri.UTCPA()
        UD = cri.UD()
        UB = cri.UB()
        UK = cri.UK()
        enc = cri.encounter_classification()
        Rf = cri.Rf()
        Ra = cri.Ra()
        Rs = cri.Rs()
        Rp = cri.Rp()
        SD_dist = cri.SD_dist()
        result = cri.CRI()

        return Vtx, Vty, RD, HAD, RB, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, result

    def PARK_cal(self, OS, TS):
        cri = CRI(rospy.get_param("shipInfo_all/OS_info/ship_L"), rospy.get_param("shipInfo_all/OS_info/ship_B"), OS['Pos_X'], OS['Pos_Y'], TS['Pos_X'], TS['Pos_Y'], deg2rad(OS['Heading']), deg2rad(TS['Heading']), OS['Vel_U'], TS['Vel_U'])
        park = cri.PARK()
        return park

    def TS_info_supplement(self, OS_list, TS_list, ship_L, radius_scale, V_des_Heading):   
        """ TS에 대한 추가적인 information 생성 

        Returns:
            TS_info : pandas dataframe

        Note : 
            TS list =  {'Ship_ID': [], 'Pos_X' : [],  'Pos_Y' : [],   'Vel_U' : [],   'Heading' : [], 'V_x' : [], 'V_y' : [], 
                        'radius' : [], 'RD' : [], 'RB' : [],'RC' : [], 'local_rc' : [], 'status' : []}
        """
        
        TS_ID = TS_list.keys()
        
        for ts_ID in TS_ID:
            OS_i = OS_list
            TS_i = TS_list[ts_ID]
            Vtx, Vty, RD, HAD, RB, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri = self.CRI_cal(OS_i, TS_i)
            park = self.PARK_cal(OS_i, TS_list[ts_ID])
            #----------- Heading angle 바탕으로 속도 벡터 구하는 함수
            TS_list[ts_ID]['V_x'] = Vtx
            TS_list[ts_ID]['V_y'] = Vty
            #----------- ship domain은 임의로 원형으로 가정
            temp_r = ship_L  * radius_scale
            # temp_r = ship_L + TS_i['Vel_U'] * radius_scale            
            TS_list[ts_ID]['radius'] = temp_r
            #----------- 자선과 타선 사이의 거리 (relative distacne)
            TS_list[ts_ID]['RD'] = RD   
            #----------- relative bearing
            TS_list[ts_ID]['RB'] = RB
            #----------- relative course
            TS_list[ts_ID]['RC'] = HAD
            #----------- 조우 상황 판단
            TS_list[ts_ID]['status'] = enc
            #----------- 타선에 대한 CRI 계산
            TS_list[ts_ID]['PARK'] = park
            TS_list[ts_ID]['UDCPA'] = UDCPA
            TS_list[ts_ID]['UTCPA'] = UTCPA
            TS_list[ts_ID]['UD'] = UD
            TS_list[ts_ID]['UB'] = UB
            TS_list[ts_ID]['UK'] = UK
            #-----------cri coefficient
            TS_list[ts_ID]['Rf'] = Rf
            TS_list[ts_ID]['Ra'] = Ra
            TS_list[ts_ID]['Rs'] = Rs
            TS_list[ts_ID]['Rp'] = Rp
            TS_list[ts_ID]['SD'] = SD_dist
            #-----------ship domian
            TS_list[ts_ID]['CRI'] = cri
            
        return TS_list


    def TS_prediction(self, TS, ship_L, TS_target_spds_dic, TS_WP_index):   
        """ 타선에 대한 정보(pub_list)를 `/path_out_inha`에 담기 위한 함수

        Note:
            pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
        Return:
            TS_pub_list = [[TS1_pub_list], [TS2_pub_list], [TS3_pub_list], .....  ]
        """
        
        TS_ID = list(map(int, TS['Ship_ID'].index))

        i = 0
        ts_pub_list = []
        for ts_ID in TS_ID:
            wp_x = self.waypoint_dict['{}'.format(ts_ID)].wpts_x[TS_WP_index[i]]
            wp_y = self.waypoint_dict['{}'.format(ts_ID)].wpts_y[TS_WP_index[i]]
            speedWP = TS[ts_ID]['Vel_U']
            # !<<---------- TS의 전역경로 추종을 위한 eta, eda, V_des  계산
            eta_wp, eda_wp = self.eta_eda_assumption([wp_x, wp_y], [TS[ts_ID]['Pos_X'], TS[ts_ID]['Pos_Y']], TS[ts_ID]['Vel_U'])
            if eda_wp < 2 * ship_L:
                TS_WP_index[i] = (TS_WP_index[i] + 1) % len(self.waypoint_dict['{}'.format(ts_ID)].wpts_x)
            V_des = self.vectorV_to_goal(TS, ts_ID, [wp_x, wp_y], [TS_target_spds_dic[ts_ID] for i in range(2)])
            desired_spd, desired_heading = self.desired_value_assumption(V_des)

            isNeedCA = TS[ts_ID]['isNeedCA']
            ts_status = TS[ts_ID]['status']

            pub_list = [ts_ID, False, TS_WP_index[i], [wp_x], [wp_y], speedWP, eta_wp, eda_wp, 0.5, 0.0, False, [], desired_spd, desired_heading, isNeedCA, ts_status]
            # pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
            ts_pub_list.append(pub_list)
            i += 1
        # self.TS_WP_index = TS_WP_index

        return ts_pub_list, TS_WP_index