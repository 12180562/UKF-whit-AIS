from functions.Inha_DataProcess import Inha_dataProcess
from functions.Inha_VelocityObstacle import VO_module

from math import sqrt, atan2,cos,sin,pi
from numpy import rad2deg

import numpy as np
# import matplotlib.pyplot as plt
import time
import yaml

with open('./etc/main_parameter.yaml') as f:
    parameter = yaml.full_load(f)
    
    
class data_inNout:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(self):

        self.available_info = dict()
        self.unavailable_info = dict()
        self.waypoint_info = dict()
        self.frm_info = dict()

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
        self.TS_WP_index = []
        self.target_heading_list = []
        
        self.static_obstacle_info = []
        self.static_point_info = []


    def wp_callback(self, wp):
        ''' subscribe `/waypoint_info`

        Example:
            OS_wpts_x = self.waypoint_dict['2000'].wpts_x
        '''
        self.len_waypoint_info = len(wp["group_wpts_info"])
        wp_dic = dict()
        for i in range(self.len_waypoint_info):
            shipID = wp["group_wpts_info"][i]["shipID"]
            wp_dic['{}'.format(shipID)] = wp["group_wpts_info"][i]
            ## 위 처럼 표현할 경우, dictionary 안의 message 변수명을 알고 있어야 호출이 가능함!
            ## 따라서, 새로운 임의의 key Value로 바꾸서 저장하고 싶다면 아래와 같이 새로운 dictionary를 만들어도 됨. (이중 dictionary 구조)
            # wp_dic2 = dict()
            # wp_dic2['waypoint_x'] = wp.group_wpts_info[i].wpts_x
            # wp_dic2['waypoint_y'] = wp.group_wpts_info[i].wpts_y
            
            # wp_dic[f'{shipID}'] = wp_dic2

        self.waypoint_dict = wp_dic
        

    def OP_callback(self, operation):
        ''' subscribe `/frm_info` 
        
        params : 
            `frm_info` 변수명은 입출력관계도 KRISO 참조

        Note :
            `psi`값이 [-2pi, 2pi]값으로 들어오므로, 편의상 강제로 [0, 2pi]로 변경
        '''
        self.Pos_X = operation["m_fltPos_X"] 
        self.Pos_Y = operation["m_fltPos_Y"] 
        self.Vel_U = operation["m_fltVel_U"] 
        self.ship_ID = list(operation["m_nShipID"])
        raw_psi = np.asanyarray(operation["m_fltHeading"])
        self.Heading = raw_psi % 360

        ############################ for connect with KRISO format ##################################

    def static_unavailable_callback(self, static_OB):
        self.len_static_obstacle_info = len(static_OB["group_boundary_info"])
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB["group_boundary_info"][i]["area_x"]))
            static_ob_list_y.append(list(static_OB["group_boundary_info"][i]["area_y"]))
            
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
        self.len_static_obstacle_info = len(static_OB["group_boundary_info"])
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB["group_boundary_info"][i]["area_x"]))
            static_ob_list_y.append(list(static_OB["group_boundary_info"][i]["area_y"]))
        
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

        ############################ for connect with KRISO format ##################################

    def path_out_publish(self, pub_list):
        ''' publish `/path_out_inha`
        
        Note :
            pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
        '''
        path_out_inha = dict()
        path_out_inha['nship_ID'] = pub_list[0]
        path_out_inha['modifyWayPoint'] = pub_list[1]
        path_out_inha['numOfWayPoint']  = pub_list[2]
        path_out_inha['latOfWayPoint'] = pub_list[3]
        path_out_inha['longOfWayPoint'] = pub_list[4]
        path_out_inha['speedOfWayPoint'] = pub_list[5]
        path_out_inha['ETAOfWayPoint'] = round(pub_list[6], 3)
        path_out_inha['EDAOfWayPoint'] = round(pub_list[7], 3)
        path_out_inha['error'] = pub_list[8]
        path_out_inha['errorCode'] = pub_list[9]
        path_out_inha['targetSpeed'] = round(pub_list[10], 3)
        path_out_inha['targetCourse'] = round(pub_list[11], 3)
        
        return path_out_inha
        # print(self.WP_pub.publish(inha))

    def path_out(self,waypoint_info,frm_info,unavailable_info,available_info):  

        self.available_info = available_info
        self.unavailable_info = unavailable_info
        self.waypoint_info = waypoint_info
        self.frm_info = frm_info

        ###   frm_info 정의 ###
        self.m_nPacketCode = self.frm_info["m_nPacketCode"]
        self.m_nShipID = self.frm_info["m_nShipID"]
        self.m_fltHeading = self.frm_info["m_fltHeading"]
        self.m_fltDriftangle = self.frm_info["m_fltDriftangle"]
        self.m_fltShipTime = self.frm_info["m_fltShipTime"]
        self.m_fltRudderAngleFeedPORT = self.frm_info["m_fltRudderAngleFeedPORT"]
        self.m_fltRudderAngleFeedSTBD = self.frm_info["m_fltRudderAngleFeedSTBD"]
        self.m_fltPropellerRPSFeedPORT = self.frm_info["m_fltPropellerRPSFeedPORT"]
        self.m_fltPropellerRPSFeedSTBD = self.frm_info["m_fltPropellerRPSFeedSTBD"]
        self.m_fltFOGvel_rollx = self.frm_info["m_fltFOGvel_rollx"]
        self.m_fltFOGvel_pitchy = self.frm_info["m_fltFOGvel_pitchy"]
        self.m_fltFOGvel_yawz = self.frm_info["m_fltFOGvel_yawz"]
        self.m_fltFOGang_rollx = self.frm_info["m_fltFOGang_rollx"]
        self.m_fltFOGang_pitchy = self.frm_info["m_fltFOGang_pitchy"]
        self.m_fltFOGang_yawz = self.frm_info["m_fltFOGang_yawz"]
        self.m_fltFOGvel_yawzG = self.frm_info["m_fltFOGvel_yawzG"]
        self.m_fltFOGang_yawzG = self.frm_info["m_fltFOGang_yawzG"]
        self.m_fltIncl_heelx = self.frm_info["m_fltIncl_heelx"]
        self.m_fltIncl_trimy = self.frm_info["m_fltIncl_trimy"]
        self.m_fltVel_U = self.frm_info["m_fltVel_U"]
        self.m_fltPos_X = self.frm_info["m_fltPos_X"]
        self.m_fltPos_Y = self.frm_info["m_fltPos_Y"]

        self.static_available_callback(self.available_info)
        self.static_unavailable_callback(self.unavailable_info)
        self.wp_callback(self.waypoint_info)
        self.OP_callback(self.frm_info)
        # get an instance of RosPack with the default search paths
        OS_ID = '{}'.format(parameter["shipInfo_all"]["ship1_info"]["ship_ID"])
        TS_ID = []
        desired_spd_list = []

        # 자선의 정보
        OS_scale = parameter["shipInfo_all"]["ship1_info"]["ship_scale"]
        target_speed = parameter["shipInfo_all"]["ship1_info"]["target_speed"]  * 0.5144 / sqrt(OS_scale)
        ship_L = parameter["shipInfo_all"]["ship1_info"]["ship_L"]
        
        
        t = 0
        waypointIndex = 0
        targetspdIndex = 0    

        Local_PP = VO_module()

        startTime = time.time()

        inha = Inha_dataProcess(
            self.ship_ID,
            self.Pos_X, 
            self.Pos_Y, 
            self.Vel_U, 
            self.Heading, 
            self.waypoint_dict,
            )


        ## <======== 서울대학교 전역경로를 위한 waypoint 수신 및 Local path의 goal로 처리
        wpts_x_os = list(self.waypoint_dict['{}'.format(OS_ID)]["wpts_x"])
        wpts_y_os = list(self.waypoint_dict['{}'.format(OS_ID)]["wpts_y"])
        Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]   
        # Local_goal = [wpts_x_os[data.waypoint_idx], wpts_y_os[data.waypoint_idx]]          # kriso
        # Local_goal = [wpts_x_os[int(data.waypoint_idx)], wpts_y_os[int(data.waypoint_idx)]]          # 부경대
        ## <========= `/frm_info`를 통해 들어온 자선 타선의 데이터 전처리
        ship_list, ship_ID = inha.ship_list_container(OS_ID)
        OS_list, TS_list = inha.classify_OS_TS(ship_list, ship_ID, OS_ID)

        TS_ID = ship_ID[:]  ## 리스트 복사
        TS_ID.remove(OS_ID)
        # TODO : why do this?

        OS_Vx, OS_Vy = inha.U_to_vector_V(OS_list['Vel_U'], OS_list['Heading'])

        OS_list['V_x'] = OS_Vx
        OS_list['V_y'] = OS_Vy

        _, local_goal_EDA = inha.eta_eda_assumption(Local_goal, OS_list, target_speed)

        # <=========== VO 기반 충돌회피를 위한 경로 생성
        # !--- 1) `Local goal`으로 향하기 위한 속도벡터 계산
        V_des = Local_PP.vectorV_to_goal(OS_list, Local_goal, target_speed)

        '''
            NOTE: 
                `OS_list` and `TS_list`:
                    {
                        'Ship_ID': [], 
                        'Pos_X' : [],  
                        'Pos_Y' : [],   
                        'Vel_U' : [],   
                        'Heading_deg' : [], 
                        'V_x' : [], 
                        'V_y' : [], 
                        'radius' : []
                    }
        '''

        TS_list = inha.TS_info_supplement(
            OS_list, 
            TS_list,
            )
        
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
        TS_ENC_temp = []

        for ts_ID in TS_ID:

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

            temp_enc = TS_list[ts_ID]['status']
            TS_ENC_temp.append(temp_enc)

            distance = sqrt((OS_list["Pos_X"]-TS_list[ts_ID]["Pos_X"])**2+(OS_list["Pos_Y"]-TS_list[ts_ID]["Pos_Y"])**2)
            
            if distance <= 20:
                print(ts_ID,":",temp_enc, distance)

        # print(TS_ENC_temp)


        # NOTE: `VO_update()` takes the majority of the computation time
        # TODO: Reduce the computation time of `VO_update()`
        # V_opt, VO_BA_all = Local_PP.VO_update(OS_list, TS_list_sort, static_OB, V_des, v_min)

        ############################ for connect with KRISO format ##################################

        # data.static_obstacle_info = data.static_available_info + data.static_unavailable_info

        ############################ for connect with KRISO format ##################################

        V_selected, pub_collision_cone = Local_PP.VO_update(
            OS_list, 
            TS_list, 
            V_des, 
            self.static_obstacle_info,
            self.static_point_info
            )

        # V_selected2 = Local_PP2.RVO_update(
        #     OS_list,
        #     TS_list,
        #     V_des,
        # )

        # print(V_selected2)

        # TODO: Reduce the computation time for this part (~timeChckpt4_vesselNode)
        desired_spd_list = []
        desired_heading_list = []

        # NOTE: Only one step ahead
        wp = inha.waypoint_generator(OS_list, V_selected, 1)
        wp_x = wp[0]
        wp_y = wp[1]


        eta, eda = inha.eta_eda_assumption(wp, OS_list, target_speed)            
        temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
        desired_spd_list.append(temp_spd)
        desired_heading_list.append(temp_heading_deg)
        desired_spd = desired_spd_list[0]
        desired_heading = desired_heading_list[0]


        V_selected = V_des
        eta, eda = inha.eta_eda_assumption(wp, OS_list, target_speed)            
        temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
        desired_spd_list = list(self.waypoint_dict['{}'.format(OS_ID)]["target_spd"])
        desired_heading_list.append(temp_heading_deg)
        desired_spd = desired_spd_list[targetspdIndex]
        desired_heading = desired_heading_list[0]

        if t%10 ==0:
            pass

        t += 1

        if len(self.target_heading_list) != parameter['filter_length']:
            self.target_heading_list.append(desired_heading)
        
        else:
            del self.target_heading_list[0]

        sum_of_heading = 0
        real_target_heading = 0
        for i in self.target_heading_list:
            sum_of_heading = sum_of_heading + i

        if len(self.target_heading_list) >= 2:
            if self.target_heading_list[len(self.target_heading_list)-1]*self.target_heading_list[len(self.target_heading_list)-2] < 0:
                self.target_heading_list = [self.target_heading_list[-1]]
                real_target_heading = desired_heading
            else:
                real_target_heading = sum_of_heading/len(self.target_heading_list)

        

        # # < =========  인하대 모듈에서 나온 데이터를 최종적으로 송신하는 부분
        # OS_pub_list = [int(OS_ID), False, waypointIndex, [wp_x], [wp_y],desired_spd, eta, eda, 0.5, 0.0, False, [], desired_spd, desired_heading, isNeedCA, ""]
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
            real_target_heading, 
            ]

        path_out_inha = self.path_out_publish(OS_pub_list)  


        if local_goal_EDA < 2 * ship_L :
        # 만약 `reach criterion`와 거리 비교를 통해 waypoint 도달하였다면, 
        # 앞서 정의한 `waypint 도달 유무 확인용 flag`를 `True`로 바꾸어 `while`문 종료
            waypointIndex = (waypointIndex + 1) % len(wpts_x_os)
            targetspdIndex = waypointIndex
            # data.waypoint_idx = (data.waypoint_idx + 1) % len(wpts_x_os)  # kriso 
            # data.waypoint_idx = (int(data.waypoint_idx) + 1) % len(wpts_x_os) # 부경대

            # targetspdIndex = data.waypoint_idx
            # waypointIndex = (waypointIndex + 1) % len(wpts_x_os)
            # targetspdIndex = waypointIndex
        
        print("Loop end time: ", time.time() - startTime)
        print("================ Node 1 loop end ================\n")

        return path_out_inha


        