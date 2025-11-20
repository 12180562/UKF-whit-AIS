#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.Inha_VelocityObstacle import VO_module
from functions.Inha_DataProcess import Inha_dataProcess
# from functions.ukf_befor import UKF
from functions.ukf import UKF

from udp_col_msg.msg import col, vis_info, cri_info_yoo, VO_info
from udp_msgs.msg import frm_info, group_wpts_info
from ukf_ais.msg import ShipInfo, ResultInfo

from math import *
from numpy import rad2deg

import csv
import numpy as np
import rospy
import time
import rospkg

import copy

class data_inNout:
    def __init__(self):
        rospy.Subscriber('/frm_info', frm_info, self.OP_callback)
        rospy.Subscriber('/waypoint_info', group_wpts_info, self.wp_callback)

        self.WP_pub = rospy.Publisher('/vessel1_info', col, queue_size=0)
        self.cri_pub = rospy.Publisher('/cri1_info', cri_info_yoo, queue_size=10)
        self.VO_pub = rospy.Publisher('/VO1_info', VO_info, queue_size=10)
        self.Vis_pub = rospy.Publisher('/vis1_info', vis_info, queue_size=10)

        self.ori_pub = rospy.Publisher('TS_list', ShipInfo, queue_size=10)

        self.result_pub = rospy.Publisher('result_info', ResultInfo, queue_size=10)

        self.ship_ID = []
        self.waypoint_idx = 0
        self.len_waypoint_info = 0
        self.waypoint_dict = dict()
        self.ts_spd_dict = dict()

        self.TS_WP_index = []

        self.static_obstacle_info = []
        self.static_point_info = []

        self.target_heading_list = []

        self.start_time = time.time()
        self.ais_delay = rospy.get_param("ais_delay")

    def wp_callback(self, wp):
        self.len_waypoint_info = len(wp.group_wpts_info)
        wp_dic = dict()
        for i in range(self.len_waypoint_info):
            shipID = wp.group_wpts_info[i].shipID
            wp_dic['{}'.format(shipID)] = wp.group_wpts_info[i]

        self.waypoint_dict = wp_dic

    def OP_callback(self, operation):
        self.ship_ID = list(operation.m_nShipID)

        self.Pos_X  = operation.m_fltPos_X
        self.Pos_Y  = operation.m_fltPos_Y
        # self.Vel_U  = operation.m_fltVel_U
        self.Vel_U  = rospy.get_param("target_spd_List/target_speed_ship1")
        # U = 
        # self.Vel_U.append(U)
        # self.Vel_U.append(U)

        self.delta_deg = operation.m_fltRudderAngleFeedSTBD # deg.

        raw_psi = np.asanyarray(operation.m_fltHeading)
        self.Heading = raw_psi % 360

    def static_unavailable_callback(self, static_OB):
        self.len_static_obstacle_info = len(static_OB.group_boundary_info)
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
            static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
            
        static_ob_info = []
        
        for k in range(len(static_ob_list_x)):
            for l in range(len(static_ob_list_x[k])):
                if l == 0:
                    pass
                else:
                    static_ob_info.append(static_ob_list_x[k][l-1])
                    static_ob_info.append(static_ob_list_y[k][l-1])
                    static_ob_info.append(static_ob_list_x[k][l])
                    static_ob_info.append(static_ob_list_y[k][l])
                    
        self.static_unavailable_info = static_ob_info
        
    def static_available_callback(self, static_OB):
        self.len_static_obstacle_info = len(static_OB.group_boundary_info)
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
            static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
        
        static_ob_info = []
        
        for k in range(len(static_ob_list_x)):
            for l in range(len(static_ob_list_x[k])):
                if l == 0:
                    pass
                else:
                    static_ob_info.append(static_ob_list_x[k][l-1])
                    static_ob_info.append(static_ob_list_y[k][l-1])
                    static_ob_info.append(static_ob_list_x[k][l])
                    static_ob_info.append(static_ob_list_y[k][l])
                    
        self.static_available_info = static_ob_info

    def path_out_publish(self, pub_list):
        inha = col()
        inha.nship_ID = pub_list[0]
        inha.modifyWayPoint = pub_list[1]
        inha.numOfWayPoint  = pub_list[2]
        inha.latOfWayPoint = pub_list[3]
        inha.longOfWayPoint = pub_list[4]
        inha.speedOfWayPoint = pub_list[5]
        inha.ETAOfWayPoint = round(pub_list[6], 3)
        inha.EDAOfWayPoint = round(pub_list[7], 3)
        inha.error = pub_list[8]
        inha.errorCode = pub_list[9]
        inha.targetSpeed = round(pub_list[10], 3)
        inha.targetCourse = round(pub_list[11], 3)
        
        self.WP_pub.publish(inha)

    def vis_out(self, pub_list):
        vis = vis_info()
        vis.nship_ID = pub_list[0]
        vis.collision_cone = pub_list[1]
        vis.v_opt = pub_list[2]
        vis.local_goal = pub_list[3]

        self.Vis_pub.publish(vis)

    def cri_out(self, pub_list):
        cri = cri_info_yoo()
        cri.RD = pub_list[0]
        cri.DCPA = pub_list[1]
        cri.TCPA = pub_list[2]
        cri.UDCPA = pub_list[3]
        cri.UTCPA = pub_list[4]
        cri.UD = pub_list[5]
        cri.UB = pub_list[6]
        cri.UK = pub_list[7]
        cri.CRI = pub_list[8]
        cri.Rf = pub_list[9]
        cri.Ra = pub_list[10]
        cri.Rs = pub_list[11]
        cri.Rp = pub_list[12]
        cri.bx = pub_list[13]
        cri.by = pub_list[14]
        cri.encounter_classification = pub_list[15]
        # print(cri.encounter_classification)

        self.cri_pub.publish(cri)

    def vo_out(self, pub_list):
        vo = VO_info()
        vo.V_opt = pub_list[0]
        vo.Collision_cone = pub_list[1]

        self.VO_pub.publish(vo)

    def ts_out(self, ship_ID, Pos_X, Pos_Y, vel, psi_deg, cpa_x, cpa_y, cpa_r):
        message = ShipInfo()
        message.Ship_ID = ship_ID
        message.Pos_X = Pos_X
        message.Pos_Y = Pos_Y
        message.Vel_U = vel
        message.Heading = psi_deg
        message.cpa_x = cpa_x
        message.cpa_y = cpa_y
        message.cpa_r = cpa_r

        self.ori_pub.publish(message)

    def result_out(self, pos_err, cov, CRP, DI):
        ukf_result = ResultInfo()
        for ship_id in pos_err.keys():                           # ① id 하나씩
            ukf_result = ResultInfo()                            #   결과 새로 만들기

            # ─── 필드 채우기 ──────────────────────────────
            ukf_result.Ship_ID  = ship_id                        # (msg에 필드 있을 때)
            ukf_result.Pos_Err  = pos_err[ship_id]

            # cov가 없으면 기본 0으로
            cov_data = cov.get(ship_id, (0.0, 0.0, 0.0, 0.0))
            ukf_result.Cov_XX, \
            ukf_result.Cov_YY, \
            ukf_result.Cov_UU, \
            ukf_result.Cov_Heading = cov_data

            ukf_result.CRP = CRP.get(ship_id, 0.0)               # 없으면 0
            ukf_result.DI  = DI.get(ship_id, 0.0)

            # ─── 퍼블리시 ────────────────────────────────
            self.result_pub.publish(ukf_result)

def main():  
    rospack = rospkg.RosPack()  
    package_path = rospack.get_path('kass_inha')
    VO_operate = rospy.get_param("shipInfo_all/ship1_info/include_inha_modules")

    update_rate = rospy.get_param("update_rate")
    dt = rospy.get_param("mmg_dt")
    detecting_distance = rospy.get_param("detecting_distance")

    timestr = time.strftime("%Y%m%d-%H%M%S")
    # path = "/home/phl/문서/" + timestr + ".csv"
    # path = "/home/phlyoo/Documents/" + timestr + ".csv"
    # header = ['ShipID', 'Pos_X', 'Pos_Y', 'wp_x', 'wp_y', 'Vel_U', 'Vx', 'Vy', 'Heading', 'desired_heading', 'encounter', 'encounterMMSI']
    # header = ['RD','RC', 'K', 'DCPA','TCPA', 'UDCPA', 'UTCPA', 'UD', 'UB', 'UK', 'CRI', 'Rf', 'Ra', 'Rs', 'Rp', 'ENC', 'V_opt', 'pub_collision_cone', 'VO_operate']

    # file = open(path, 'a', newline='')
    # writer = csv.writer(file)
    # writer.writerow(header)

    node_Name = "vessel_node1"
    rospy.init_node("{}".format(node_Name), anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    OS_ID = rospy.get_param("shipInfo_all/ship1_info/ship_ID")
    TS_ID = []
    desired_spd_list = []
    pub_collision_cone = []
    V_opt = []

    # 자선의 정보
    OS_scale = rospy.get_param("shipInfo_all/ship1_info/ship_scale")
    target_speed = (rospy.get_param("shipInfo_all/ship1_info/target_speed")  * 0.5144) / sqrt(OS_scale)
    ship_L = rospy.get_param("shipInfo_all/ship1_info/ship_L")
    ship_B = rospy.get_param("shipInfo_all/ship1_info/ship_B")
    ship_T = rospy.get_param("shipInfo_all/ship1_info/ship_T")

    ship_L_scaled = ship_L / OS_scale
    ship_B_scaled = ship_B / OS_scale
    ship_T_scaled = ship_T / OS_scale

    data = data_inNout()
    
    t = 0
    waypointIndex = 0
    targetspdIndex = 0    

    encounter = None
    encounterMMSI = []

# UKF Declare Variables Part
#####################################################################################################################
    
    ukf_dt = rospy.get_param('ukf_dt')

    last_publish_time = rospy.Time.now()  # 마지막으로 발행한 시간을 초기화
    AIS_delay = rospy.get_param('ais_delay')
    publish_interval = rospy.Duration(AIS_delay)  # 발행 주기를 5초로 설정
    
    ukf_instance = {}
    TS_list_ori={}  
    TS_list_del={}
    TS_list_pre = {}


    predicted_state = []
    AIS_previous_input_list = {}
    radar_previous_input_list = {}

    X_diff = {}
    Y_diff = {}
    U_diff = {}
    H_diff = {}
    pos_err = {}
    cov = {}

    pos_err_list = {}
    relative_distance_list = {}
    relative_bearing_list = {}

    first_loop = True
    first_publish = True
    heading_diff = 0.0

    radar_delay = rospy.get_param('radar_delay')
    radar_update_interval = rospy.Duration(radar_delay)  # 2.5초
    radar_last_update_time = rospy.Time.now()

    start_time = time.perf_counter()    # 권장: 높은 해상도의 경과 시간 전용 타이머
    avoide_start = 0
    avoide_cri = 0
    # 2) 메인 루프 ― 예: 10 Hz(0.1 s마다)로 도는 시뮬레이션
    dt = 0.1           # 한 주기 간격 [초]
#####################################################################################################################

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()  # 현재 시간을 계속 추적
        Local_PP = VO_module()

        if len(data.ship_ID) == 0:
            print("========= Waiting for `/frm_info` topic subscription in {}=========".format(node_Name))
            rate.sleep()
            continue

        if data.len_waypoint_info == 0:
            print("========= Waiting for `/waypoint_info` topic subscription in {} =========".format(node_Name))
            rate.sleep()
            continue

        inha = Inha_dataProcess(
            data.ship_ID,
            data.Pos_X, 
            data.Pos_Y, 
            data.Vel_U, 
            data.Heading, 
            data.waypoint_dict
            )

        wpts_x_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_x)
        wpts_y_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_y)
        Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]

        ship_list, ship_ID = inha.ship_list_container(OS_ID)
        OS_list, TS_list_ori = inha.classify_OS_TS(ship_list, ship_ID, OS_ID)
        TS_ID = TS_list_ori.keys()
        # TODO : why do this?

# UKF Main part
#####################################################################################################################   

        # print("TS_list_ori: ", TS_list_ori)
        if first_loop:
            for ts_ID in TS_ID:
                ukf_instance[ts_ID] = UKF()

            first_loop = False

        if (current_time - radar_last_update_time >= radar_update_interval) or first_publish:
            for ts_ID in TS_ID:
                relative_distance = sqrt((TS_list_ori[ts_ID]["Pos_X"]-OS_list["Pos_X"])**2 + \
                                (TS_list_ori[ts_ID]["Pos_Y"]-OS_list["Pos_Y"])**2)
                relative_bearing = rad2deg(atan2(TS_list_ori[ts_ID]["Pos_Y"]-OS_list["Pos_Y"], \
                                            TS_list_ori[ts_ID]["Pos_X"]-OS_list["Pos_X"]))

                radar_last_update_time = current_time
                # print("-------------Radar Infromation Update-------------")
                relative_distance_list[ts_ID] = relative_distance
                relative_bearing_list[ts_ID] = relative_bearing

        if(current_time - last_publish_time >= publish_interval) or first_publish:
            for ts_ID in TS_ID:   
                TS_list_del[ts_ID] = TS_list_ori[ts_ID]
                last_publish_time = current_time
                # print("-------------AIS Infromation Update-------------")

            
            heading_diff = TS_list_del[ts_ID]['Heading'] - TS_list_ori[ts_ID]['Heading']

            if heading_diff < 0:
                heading_diff += 360
            elif heading_diff >= 360:
                heading_diff -= 360

            if abs(heading_diff) >= 5:
                TS_list_del[ts_ID] = TS_list_ori[ts_ID]

            else:
                TS_list_del[ts_ID] = TS_list_del[ts_ID]

        first_publish = False
        # print("delay: ",TS_list_del)


        TS_list_pre = copy.deepcopy(TS_list_ori)

        os_pos = np.array([OS_list["Pos_X"], OS_list["Pos_Y"]])
        
        for ts_ID in TS_ID:
            AIS_input_list = []
            radar_input_list = []
            AIS_input_list.append(TS_list_del[ts_ID]['Pos_X'])
            AIS_input_list.append(TS_list_del[ts_ID]['Pos_Y'])
            AIS_input_list.append(TS_list_del[ts_ID]['Vel_U'])
            AIS_input_list.append(TS_list_del[ts_ID]['Heading'])
            radar_input_list.append(relative_distance_list[ts_ID])
            radar_input_list.append(relative_bearing_list[ts_ID])
            # print("relative_bearing ", relative_bearing_list[ts_ID])

# --------------------------------------- use AIS and Radar change import----------------------------------------------------------
            predicted_state, covariance = ukf_instance[ts_ID].predict(ukf_dt)


            if ts_ID in AIS_previous_input_list and AIS_previous_input_list[ts_ID] == AIS_input_list:
                pass
            else:
                predicted_state, covariance= ukf_instance[ts_ID].update_AIS(AIS_input_list)

            if ts_ID in radar_previous_input_list and radar_previous_input_list[ts_ID] == radar_input_list:
                pass
            else:
                predicted_state, covariance= ukf_instance[ts_ID].update_Radar(radar_input_list, os_pos)

# --------------------------------------- Only AIS and change import----------------------------------------------------------
            # if ts_ID in AIS_previous_input_list and AIS_previous_input_list[ts_ID] == AIS_input_list:
            #     predicted_state, covariance = ukf_instance[ts_ID].predict(ukf_dt)

            # else:
            #     predicted_state, covariance= ukf_instance[ts_ID].update(AIS_input_list, ukf_dt)
# -----------------------------------------------------------------------------------------------------------

            AIS_previous_input_list[ts_ID] = AIS_input_list.copy()
            radar_previous_input_list[ts_ID] = radar_input_list.copy()

            update_keys_state = ['Pos_X', 'Pos_Y', 'Vel_U', 'Heading']

            for key, value in zip(update_keys_state, predicted_state):
                if key in TS_list_pre[ts_ID]:
                    TS_list_pre[ts_ID][key] = value
            
            cov[ts_ID] = list(np.diagonal(covariance))
            update_keys_var = ['x_var', 'y_var', 'U_var', 'theta_var']

            for key, value in zip(update_keys_var, cov[ts_ID]):
                TS_list_pre[ts_ID][key] = value

            X_diff[ts_ID] = TS_list_pre[ts_ID]["Pos_X"] - TS_list_ori[ts_ID]["Pos_X"]
            Y_diff[ts_ID] = TS_list_pre[ts_ID]["Pos_Y"] - TS_list_ori[ts_ID]["Pos_Y"]
            U_diff[ts_ID] = TS_list_pre[ts_ID]["Vel_U"] - TS_list_ori[ts_ID]["Vel_U"]
            H_diff[ts_ID] = TS_list_pre[ts_ID]["Heading"] - TS_list_ori[ts_ID]["Heading"]
        
            pos_err[ts_ID] = np.sqrt(X_diff[ts_ID]**2 + Y_diff[ts_ID]**2)

            pos_err_list[ts_ID] = round(pos_err[ts_ID],3)
            # print(X_diff[2001],Y_diff[2001])
            # print("\n")
            # print(cov[ts_ID])
            # print(type(cov[ts_ID]))
            
            # TS_list = TS_list_ori
            # TS_list = TS_list_del
            TS_list = TS_list_pre
            # print("x_var : ",TS_list[ts_ID]['x_var'])
            # print("y_var : ",TS_list[ts_ID]['y_var'])
#####################################################################################################################
        
        # print("\n")
        # print("pos_err :    ", pos_err_list)
        # print(TS_list)
        print("\n")
        print("x_var 2001 : ", round(TS_list[2001]['x_var']))
        print("y_var 2001 : ", round(TS_list[2001]['y_var']))
        print("\n")

        print("x_var 2002 : ", round(TS_list[2002]['x_var']))
        print("y_var 2002 : ", round(TS_list[2002]['y_var']))
        print("\n")

        print("x_var 2003 : ", round(TS_list[2003]['x_var']))
        print("y_var 2003 : ", round(TS_list[2003]['y_var']))
        # print("\n")

        OS_Vx, OS_Vy = inha.U_to_vector_V(OS_list['Vel_U'], OS_list['Heading'])

        OS_list['V_x'] = OS_Vx
        OS_list['V_y'] = OS_Vy

        _, local_goal_EDA = inha.eta_eda_assumption(Local_goal, OS_list, target_speed)

        V_des = Local_PP.vectorV_to_goal(OS_list, Local_goal, target_speed)

        TS_list = inha.TS_info_supplement(OS_list, TS_list)
        
        TS_RD_temp = []
        TS_RC_temp = []
        TS_K_temp = []
        TS_DCPA_temp = []
        TS_TCPA_temp = []
        TS_UDCPA_temp = []
        TS_UTCPA_temp = []
        TS_UD_temp = []
        TS_UB_temp = []
        TS_UK_temp = []
        TS_CRI_temp = []
        TS_Rf_temp = []
        TS_Ra_temp = []
        TS_Rs_temp = []
        TS_Rp_temp = []
        TS_bx_temp = []
        TS_by_temp = []
        TS_ENC_temp = []
        distance = {}

        for ts_ID in TS_ID:
            temp_RD = TS_list[ts_ID]['RD']
            TS_RD_temp.append(temp_RD)
            
            temp_RC = TS_list[ts_ID]['RC']
            TS_RC_temp.append(temp_RC)

            temp_K = TS_list[ts_ID]['K']
            TS_K_temp.append(temp_K)

            temp_DCPA = TS_list[ts_ID]['DCPA']
            TS_DCPA_temp.append(temp_DCPA)

            temp_TCPA = TS_list[ts_ID]['TCPA']
            TS_TCPA_temp.append(temp_TCPA)

            temp_UDCPA = TS_list[ts_ID]['UDCPA']
            TS_UDCPA_temp.append(temp_UDCPA)
            
            temp_UTCPA = TS_list[ts_ID]['UTCPA']
            TS_UTCPA_temp.append(temp_UTCPA)

            temp_UD = TS_list[ts_ID]['UD']
            TS_UD_temp.append(temp_UD)

            temp_UB = TS_list[ts_ID]['UB']
            TS_UB_temp.append(temp_UB)

            temp_UK = TS_list[ts_ID]['UK']
            TS_UK_temp.append(temp_UK)

            temp_cri = TS_list[ts_ID]['CRI']
            TS_CRI_temp.append(temp_cri)

            temp_Rf = TS_list[ts_ID]['Rf']
            TS_Rf_temp.append(temp_Rf)

            temp_Ra = TS_list[ts_ID]['Ra']
            TS_Ra_temp.append(temp_Ra)

            temp_Rs = TS_list[ts_ID]['Rs']
            TS_Rs_temp.append(temp_Rs)

            temp_Rp = TS_list[ts_ID]['Rp']
            TS_Rp_temp.append(temp_Rp)

            temp_point = TS_list[ts_ID]['SD_point']
            for (by, bx) in temp_point:
                TS_bx_temp.append(by)
                TS_by_temp.append(bx)

            temp_enc = TS_list[ts_ID]['status']
            TS_ENC_temp.append(temp_enc)
            # print(temp_enc)

            distance[ts_ID] = sqrt((OS_list["Pos_X"]-TS_list[ts_ID]["Pos_X"])**2+(OS_list["Pos_Y"]-TS_list[ts_ID]["Pos_Y"])**2)
            # print("distance :   ", round(distance[ts_ID],3))
            # print("CRI :        ", temp_cri)
        # print("tcpa :        ", temp_TCPA)
        # print("dcpa :        ", temp_DCPA)
        # print(temp_point)
        
        # for ts_ID in list(TS_ID):
        #     if distance[ts_ID] > detecting_distance:
        #         del TS_list[ts_ID]

# TS_list include CPA information 
#################################################################
        damage_index = {ts_id: 0.0 for ts_id in TS_ID}

        TS_list_cpa = copy.deepcopy(TS_list)

        for ts_ID in TS_ID:
            cpa_id = ts_ID + 1000
            cpa_cri = 0
            cpa_status = 'cpa'

            TS_list_cpa.setdefault(cpa_id, {})
            # print("TCPA : ",TS_list[ts_ID]["TCPA"])
            # print("DCAP : ",TS_list[ts_ID]["DCPA"])
            # print("mapped : ",TS_list[ts_ID]["mapped_radius"])
            cpa_x, cpa_y, cpa_vx, cpa_vy, cpa_mapped_radius, brg_rb, brg_lb = 0,0,0,0,0,0,0
            # 겹치지 않으면 
            if (TS_list[ts_ID]["DCPA"] >= TS_list[ts_ID]["mapped_radius"]) or TS_list[ts_ID]["TCPA"]<0:
                TS_list_cpa[cpa_id].update({
                    'Ship_ID'           : cpa_id,            # 1000 + ts_ID
                    'Pos_X'             : cpa_x,             # CPA 중심 X
                    'Pos_Y'             : cpa_y,             # CPA 중심 Y
                    'V_x'               : cpa_vx,            # CPA 속도 X (지금은 0)
                    'V_y'               : cpa_vy,            # CPA 속도 Y (지금은 0)
                    'mapped_radius'     : cpa_mapped_radius, # CPA 원 반지름
                    'right_boundary'    : brg_rb,            # 우현(오른쪽) 접선 각(라디안)
                    'left_boundary'     : brg_lb,            # 좌현(왼쪽)  접선 각(라디안)
                    'CRI'               : cpa_cri,           # 위험도 등급 (예: 0)
                    'status'            : cpa_status         # 상태 문자열 'cpa'
                })

            # 겹치면
            else:
                vox = OS_list["Vel_U"]*np.cos(np.deg2rad(OS_list["Heading"]))
                voy = OS_list["Vel_U"]*np.sin(np.deg2rad(OS_list["Heading"]))

                vtx = TS_list[ts_ID]["Vel_U"]*np.cos(np.deg2rad(TS_list[ts_ID]["Heading"]))
                vty = TS_list[ts_ID]["Vel_U"]*np.sin(np.deg2rad(TS_list[ts_ID]["Heading"]))

                # vtx = 1.3*np.cos(np.deg2rad(TS_list[ts_ID]["Heading"]))
                # vty = 1.3*np.sin(np.deg2rad(TS_list[ts_ID]["Heading"]))

                cpa_x = ((OS_list["Pos_X"]+vox*TS_list[ts_ID]['TCPA'])+(TS_list[ts_ID]["Pos_X"]+vtx*TS_list[ts_ID]['TCPA']))/2
                cpa_y = ((OS_list["Pos_Y"]+voy*TS_list[ts_ID]['TCPA'])+(TS_list[ts_ID]["Pos_Y"]+vty*TS_list[ts_ID]['TCPA']))/2

                cpa_vx = 0
                cpa_vy = 0
                cpa_mapped_radius = (TS_list[ts_ID]["DCPA"]/2)+(TS_list[ts_ID]["mapped_radius"]-TS_list[ts_ID]["DCPA"])

                # ① 필요한 값들 ─ (예시: 이미 계산해 둔 값이라고 가정)
                xo, yo = OS_list["Pos_X"], OS_list["Pos_Y"]     # 자선 위치
                xc, yc = cpa_x, cpa_y                           # CPA 원 중심
                R = cpa_mapped_radius                           # CPA 원 반지름

                # ② 자선→원 중심까지 거리 d
                dx, dy = xc - xo, yc - yo
                d = hypot(dx, dy)                          # 피타고라스 √(dx²+dy²)

                if d <= R:                                      # d ≤ R이면 원 안에 있음 → 접선 불가
                    raise ValueError("접선을 그릴 수 없습니다.")

                # ── 2. 접점 좌표 (해석식)
                #    v⊥ = ( -vy, vx )  (90° 회전)
                dpx, dpy = -dy, dx
                k1 = (R**2) / (d**2)
                k2 = R * sqrt(d**2 - R**2) / (d**2)

                T1x = xc + k1 * dx + k2 * dpx   # LB  (우선 가정)
                T1y = yc + k1 * dy + k2 * dpy
                T2x = xc + k1 * dx - k2 * dpx   # RB
                T2y = yc + k1 * dy - k2 * dpy

                # ── 3. 방위각(북=0, 시계+) 계산
                def bearing(dx, dy):
                    return (atan2(dx, dy) + 2 * pi) % (2 * pi)

                brg_lb = bearing(T1y - yo, T1x - xo)
                brg_rb = bearing(T2y - yo, T2x - xo)

                TS_list_cpa[cpa_id].update({
                    'Ship_ID'           : cpa_id,            # 1000 + ts_ID
                    'Pos_X'             : cpa_x,             # CPA 중심 X
                    'Pos_Y'             : cpa_y,             # CPA 중심 Y
                    'V_x'               : cpa_vx,            # CPA 속도 X (지금은 0)
                    'V_y'               : cpa_vy,            # CPA 속도 Y (지금은 0)
                    'mapped_radius'     : cpa_mapped_radius, # CPA 원 반지름
                    'right_boundary'    : brg_rb,            # 우현(오른쪽) 접선 각(라디안)
                    'left_boundary'     : brg_lb,            # 좌현(왼쪽)  접선 각(라디안)
                    'CRI'               : cpa_cri,           # 위험도 등급 (예: 0)
                    'status'            : cpa_status         # 상태 문자열 'cpa'
                })
            # print("cpa x,y : ", cpa_x,cpa_y)
            # print("cpa left right : ",rad2deg(brg_lb), rad2deg(brg_rb))
        # print(TS_list_cpa)
################################################################

        V_selected, pub_collision_cone, collision_risk_vo = Local_PP.VO_update(
            OS_list, 
            # TS_list,
            TS_list_cpa,
            V_des, 
            data.static_obstacle_info,
            data.static_point_info
            )

# Estimate Collision Damage
########################################################################################################
        # print("CRP : ",collision_risk_vo, "%")

        rho = 1025  # 해수 밀도 kg/m³
        # 선종 별 평균 값 벌크 0.85, 탱커 0.83, 컨테이너 0.7
        Cb_o = 0.7
        Cb_t = 0.7

        M_o = ship_L * ship_B * ship_T * Cb_o * rho   # 자선 kg
        M_t = ship_L * ship_B * ship_T * Cb_t * rho    # 타선 kg
        mu  = (M_o * M_t) / (M_o + M_t)
        # print("OS_MU : ", M_o)
        for ts_ID in TS_ID:
            vox = OS_list["Vel_U"]*np.cos(np.deg2rad(OS_list["Heading"]))*sqrt(31.65)
            voy = OS_list["Vel_U"]*np.sin(np.deg2rad(OS_list["Heading"]))*sqrt(31.65)

            vtx = TS_list[ts_ID]["Vel_U"]*np.cos(np.deg2rad(TS_list[ts_ID]["Heading"]))*sqrt(31.65)
            vty = TS_list[ts_ID]["Vel_U"]*np.sin(np.deg2rad(TS_list[ts_ID]["Heading"]))*sqrt(31.65)
            VA = np.array([vox, voy], dtype=float)   # 자선 [Vx, Vy]  (m/s)
            VB = np.array([vtx, vty], dtype=float)   # 타선 [Vx, Vy]  (m/s)

            Vr = VB - VA                 # 상대 속도 벡터
            RV = np.linalg.norm(Vr)
            damage_index[ts_ID] = (0.5 * mu * RV**2 * (collision_risk_vo[ts_ID]/100))/10**6
        # print("damage_index : ",damage_index, "MJ")

#########################################################################################################
        
        desired_spd_list = []
        desired_heading_list = []

        wp = inha.waypoint_generator(OS_list, V_selected, dt)
        wp_x = wp[0]
        wp_y = wp[1]

        if VO_operate:
            eta, eda = inha.eta_eda_assumption(wp, OS_list, target_speed)            
            temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
            desired_spd_list.append(temp_spd)
            desired_heading_list.append(temp_heading_deg)
            desired_spd = desired_spd_list[0]
            desired_heading = desired_heading_list[0]
        
        else:
            V_selected = V_des
            eta, eda = inha.eta_eda_assumption(wp, OS_list, target_speed)            
            temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
            desired_spd_list = list(data.waypoint_dict['{}'.format(OS_ID)].target_spd)
            desired_heading_list.append(temp_heading_deg)
            desired_spd = desired_spd_list[targetspdIndex]
            desired_heading = desired_heading_list[0]
        if t%10 ==0:
            pass

        t += 1

        if len(data.target_heading_list) != rospy.get_param('filter_length'):
            data.target_heading_list.append(desired_heading)
        
        else:
            del data.target_heading_list[0]

        sum_of_heading = 0
        real_target_heading = 0
        for i in data.target_heading_list:
            sum_of_heading = sum_of_heading + i

        if len(data.target_heading_list) >= 2:
            if data.target_heading_list[len(data.target_heading_list)-1]*data.target_heading_list[len(data.target_heading_list)-2] < 0:
                data.target_heading_list = [data.target_heading_list[-1]]
                real_target_heading = desired_heading
            else:
                real_target_heading = sum_of_heading/len(data.target_heading_list)

        a = (real_target_heading + 360) % 360

        if a<=3:
            avoide_start = round(distance[ts_ID],3)
            avoide_cri = temp_cri
        # print("avoide_start :   ", avoide_start)
        # print("n*L :            ", avoide_start/(ship_L/OS_scale))
        # print("avoide_cri :     ", avoide_cri)

        OS_pub_list = [
            int(OS_ID), 
            False,
            waypointIndex,
            # int(data.waypoint_idx), # 부경대 i_way
            # data.waypoint_idx, # kriso
            [wp_x], 
            [wp_y],  
            desired_spd_list, 
            eta, 
            eda, 
            False, 
            0, 
            desired_spd, 
            # desired_heading
            a,
            ]

        vis_pub_list = [
            int(OS_ID), 
            pub_collision_cone,
            V_opt,
            Local_goal
        ]

        cri_pub_list = [
            TS_RD_temp,
            TS_DCPA_temp,
            TS_TCPA_temp,
            TS_UDCPA_temp,
            TS_UTCPA_temp,
            TS_UD_temp,
            TS_UB_temp,
            TS_UK_temp,
            TS_CRI_temp,
            TS_Rf_temp,
            TS_Ra_temp,
            TS_Rs_temp,
            TS_Rp_temp,
            TS_bx_temp,
            TS_by_temp,
            TS_ENC_temp,
        ]

        vo_pub_list = [
            V_selected,
            pub_collision_cone
        ]

        ship_dic2list = list(OS_list.values())

        """
        savedata_list = [
            TS_RD_temp,
            TS_RC_temp,
            TS_K_temp,
            TS_DCPA_temp,
            TS_TCPA_temp,
            TS_UDCPA_temp,
            TS_UTCPA_temp,
            TS_UD_temp,
            TS_UB_temp,
            TS_UK_temp,
            TS_CRI_temp,
            TS_Rf_temp,
            TS_Ra_temp,
            TS_Rs_temp,
            TS_Rp_temp,
            TS_ENC_temp,
            V_selected,
            pub_collision_cone,
            VO_operate
        ]

        savedata_list = [
            int(OS_ID),
            ship_dic2list[1],
            ship_dic2list[2],
            wp_x,
            wp_y,
            ship_dic2list[3],
            OS_Vx,
            OS_Vy,
            ship_dic2list[4],
            desired_heading,
            encounter,
            encounterMMSI
        ]

        writer.writerow(savedata_list)
        """

        data.path_out_publish(OS_pub_list)
        data.vis_out(vis_pub_list)
        data.cri_out(cri_pub_list)
        data.vo_out(vo_pub_list)

        shipID_all = [ship_info['Ship_ID'] for ship_info in TS_list.values()]
        Pos_X_all = [ship_info['Pos_X'] for ship_info in TS_list.values()]
        Pos_Y_all = [ship_info['Pos_Y'] for ship_info in TS_list.values()]
        Vel_U_all = [ship_info['Vel_U'] for ship_info in TS_list.values()]
        Heading_deg_all = [ship_info['Heading'] for ship_info in TS_list.values()]

        data.ts_out(
                shipID_all, 
                Pos_X_all, 
                Pos_Y_all, 
                Vel_U_all, 
                Heading_deg_all, 
                cpa_x,
                cpa_y,
                cpa_mapped_radius
            )

        data.result_out(pos_err, cov, collision_risk_vo, damage_index)

        if local_goal_EDA < 2 * (ship_L_scaled) :
            waypointIndex = (waypointIndex + 1) % len(wpts_x_os)
            targetspdIndex = waypointIndex

        rate.sleep()
        
        # print("Loop end time: ", time.time() - startTime)
        # print("================ Node 1 loop end ================\n")

    # file.close()

    rospy.spin()

if __name__ == '__main__':
    main()