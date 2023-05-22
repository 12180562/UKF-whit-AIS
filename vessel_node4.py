#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.Inha_VelocityObstacle import VO_module
from functions.Inha_DataProcess import Inha_dataProcess

from udp_col_msg.msg import col, vis_info, cri_info, VO_info, static_OB_info
from udp_msgs.msg import frm_info, group_wpts_info

from math import sqrt, atan2
from numpy import rad2deg

import csv
import numpy as np
import rospy
import time
import rospkg

class data_inNout:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(self):
        # Subscriber = input
        rospy.Subscriber('/frm_info', frm_info, self.OP_callback) 
        rospy.Subscriber('/waypoint_info', group_wpts_info, self.wp_callback)
        # rospy.Subscriber('/static_OB_info', static_OB_info, self.static_OB_callback)
        # rospy.Subscriber('/wpts_idx_os_kriso', wpt_idx_os, self.wp_idx_callback)
        # rospy.Subscriber('/ctrl_info_pknu', ctrl_output_pknu, self.wp_idx_callback)

        ############################ for connect with KRISO format ##################################

        # rospy.Subscriber('/Unavailiable_Area_info', group_boundary_info, self.static_unavailable_callback)
        # rospy.Subscriber('/Availiable_Area_info', group_boundary_info, self.static_available_callback)

        ############################ for connect with KRISO format ##################################

        self.WP_pub = rospy.Publisher('/vessel4_info', col, queue_size=10)
        self.cri_pub = rospy.Publisher('/cri4_info', cri_info, queue_size=10)
        self.VO_pub = rospy.Publisher('/VO4_info', VO_info, queue_size=10)
        self.Vis_pub = rospy.Publisher('/vis4_info', vis_info, queue_size=10)
        self.ship_ID = []
        self.waypoint_idx = 0
        self.len_waypoint_info = 0
        self.waypoint_dict = dict()
        self.ts_spd_dict = dict()
        # self.ship1_index = rospy.get_param('ship1_index')
        # self.index = rospy.get_param('index')

        self.TS_WP_index = []
        ############################ for connect with KRISO format ##################################

        # self.static_unavailable_info =[]
        # self.static_available_info =[]

        ############################ for connect with KRISO format ##################################
        self.static_obstacle_info = []
        self.static_point_info = []

        self.target_heading_list = []

    def wp_callback(self, wp):
        ''' subscribe `/waypoint_info`

        Example:
            OS_wpts_x = self.waypoint_dict['2000'].wpts_x
        '''
        self.len_waypoint_info = len(wp.group_wpts_info)
        wp_dic = dict()
        for i in range(self.len_waypoint_info):
            shipID = wp.group_wpts_info[i].shipID
            wp_dic['{}'.format(shipID)] = wp.group_wpts_info[i]
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
        self.ship_ID = list(operation.m_nShipID)

        self.Pos_X  = operation.m_fltPos_X
        self.Pos_Y  = operation.m_fltPos_Y
        self.Vel_U  = operation.m_fltVel_U

        self.delta_deg = operation.m_fltRudderAngleFeedSTBD # deg.

        raw_psi = np.asanyarray(operation.m_fltHeading)
        self.Heading = raw_psi % 360


    # def static_OB_callback(self, static_OB):

    #     self.static_obstacle_info = static_OB.data
    #     self.static_point_info = static_OB.point

        ############################ for connect with KRISO format ##################################

    # def static_unavailable_callback(self, static_OB):
    #     self.len_static_obstacle_info = len(static_OB.group_boundary_info)
    #     static_ob_list_x = []
    #     static_ob_list_y = []
    #     for i in range(self.len_static_obstacle_info):
    #         static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
    #         static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
            
    #     static_ob_info = []
        
    #     for k in range(len(static_ob_list_x)):
    #         for l in range(len(static_ob_list_x[k])):
    #             if l == 0:
    #                 pass
    #             else:
    #                 static_ob_info.append(static_ob_list_x[k][l-1])
    #                 static_ob_info.append(static_ob_list_y[k][l-1])
    #                 static_ob_info.append(static_ob_list_x[k][l])
    #                 static_ob_info.append(static_ob_list_y[k][l])
                    
    #     self.static_unavailable_info = static_ob_info
        
    # def static_available_callback(self, static_OB):
    #     self.len_static_obstacle_info = len(static_OB.group_boundary_info)
    #     static_ob_list_x = []
    #     static_ob_list_y = []
    #     for i in range(self.len_static_obstacle_info):
    #         static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
    #         static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
        
    #     static_ob_info = []
        
    #     for k in range(len(static_ob_list_x)):
    #         for l in range(len(static_ob_list_x[k])):
    #             if l == 0:
    #                 pass
    #             else:
    #                 static_ob_info.append(static_ob_list_x[k][l-1])
    #                 static_ob_info.append(static_ob_list_y[k][l-1])
    #                 static_ob_info.append(static_ob_list_x[k][l])
    #                 static_ob_info.append(static_ob_list_y[k][l])
                    
    #     self.static_available_info = static_ob_info

        ############################ for connect with KRISO format ##################################

    def path_out_publish(self, pub_list):
        ''' publish `/path_out_inha`
        
        Note :
            pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
        '''
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
        cri = cri_info()
        cri.DCPA = pub_list[0]
        cri.TCPA = pub_list[1]
        cri.UDCPA = pub_list[2]
        cri.UTCPA = pub_list[3]
        cri.UD = pub_list[4]
        cri.UB = pub_list[5]
        cri.UK = pub_list[6]
        cri.CRI = pub_list[7]
        cri.Rf = pub_list[8]
        cri.Ra = pub_list[9]
        cri.Rs = pub_list[10]
        cri.Rp = pub_list[11]
        cri.encounter_classification = pub_list[12]

        self.cri_pub.publish(cri)

    def vo_out(self, pub_list):
        vo = VO_info()
        vo.V_opt = pub_list[0]
        vo.Collision_cone = pub_list[1]

        self.VO_pub.publish(vo)

def main():  
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()  
    package_path = rospack.get_path('kass_inha')
    VO_operate = rospy.get_param("shipInfo_all/ship4_info/include_inha_modules")

    update_rate = rospy.get_param("update_rate")
    dt =  rospy.get_param("mmg_dt")

    timestr = time.strftime("%Y%m%d-%H%M%S")
    # path = "/home/phl/문서/" + timestr + ".csv"
    path = "/home/phlmy/Documents/" + timestr + ".csv"
    header = ['ShipID', 'Pos_X', 'Pos_Y', 'wp_x', 'wp_y', 'Vel_U', 'Vx', 'Vy', 'Heading', 'desired_heading']
    file = open(path, 'a', newline='')
    writer = csv.writer(file)
    writer.writerow(header)

    node_Name = "vessel_node1"
    rospy.init_node("{}".format(node_Name), anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    OS_ID = rospy.get_param("shipInfo_all/ship4_info/ship_ID")
    TS_ID = []
    desired_spd_list = []
    pub_collision_cone = []
    V_opt = []

    # 자선의 정보
    OS_scale = rospy.get_param("shipInfo_all/ship4_info/ship_scale")
    target_speed = rospy.get_param("shipInfo_all/ship4_info/target_speed")  * 0.5144 / sqrt(OS_scale)
    ship_L = rospy.get_param("shipInfo_all/ship4_info/ship_L") ## 향후 이부분은, 1) ship domain과 2) AIS data의 선박의 길이 부분으로 나누어 고려 및 받아야 함!
    
    # # !----- 설문조사에서는 충돌회피 시점은 12m 어선 기준으로 일반적으론 HO 3nm/ CS & OT 2nm을 기준으로 하고 있으며, 최소 안전 이격거리는 0.5~ 1nm으로 조사됨
    # # 다만, 2m급 모형선 테스트에서는 협소한 부분이 있으므로 스케일 다운(1/200)을 시켜서, "회피시점: 0.0015nm(27.78m) / 최소 안전 이격거리는 9.26m"가 되게끔 할 예정
    # # 참고 논문: https://www.koreascience.or.kr/article/JAKO201427542599696.pdf 

    data = data_inNout()
    
    t = 0
    waypointIndex = 0
    targetspdIndex = 0    

    while not rospy.is_shutdown():
        Local_PP = VO_module()
        
        if len(data.ship_ID) == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/frm_info` topic subscription in {}=========".format(node_Name))
            rate.sleep()
            continue

        if data.len_waypoint_info == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/waypoint_info` topic subscription in {} =========".format(node_Name))
            rate.sleep()
            continue

        startTime = time.time()

        inha = Inha_dataProcess(
            data.ship_ID,
            data.Pos_X, 
            data.Pos_Y, 
            data.Vel_U, 
            data.Heading, 
            data.waypoint_dict,
            )                       # inha_module의 data 송신을 위해 필요한 함수들이 정의됨

        ## <======== 서울대학교 전역경로를 위한 waypoint 수신 및 Local path의 goal로 처리
        wpts_x_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_x)
        wpts_y_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_y)
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
            data.static_obstacle_info,
            data.static_point_info
            )

        # TODO: Reduce the computation time for this part (~timeChckpt4_vesselNode)
        desired_spd_list = []
        desired_heading_list = []

        # NOTE: Only one step ahead
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

        # # < =========  인하대 모듈에서 나온 데이터를 최종적으로 송신하는 부분
        # OS_pub_list = [int(OS_ID), False, waypointIndex, [wp_x], [wp_y],desired_spd, eta, eda, 0.5, 0.0, False, [], desired_spd, desired_heading, isNeedCA, ""]
        OS_pub_list = [
            int(OS_ID), 
            False, 
            waypointIndex, 
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

        vis_pub_list = [
            int(OS_ID), 
            pub_collision_cone,
            V_opt,
            Local_goal
        ]

        cri_pub_list = [
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
        ]

        vo_pub_list = [
            V_selected,
            pub_collision_cone
        ]

        ship_dic2list = list(OS_list.values())

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
        ]

        writer.writerow(savedata_list)

        data.path_out_publish(OS_pub_list)   
        data.vis_out(vis_pub_list)
        data.cri_out(cri_pub_list)
        data.vo_out(vo_pub_list)

        if local_goal_EDA < 2 * ship_L :
        # 만약 `reach criterion`와 거리 비교를 통해 waypoint 도달하였다면, 
        # 앞서 정의한 `waypint 도달 유무 확인용 flag`를 `True`로 바꾸어 `while`문 종료
            waypointIndex = (waypointIndex + 1) % len(wpts_x_os)
            targetspdIndex = waypointIndex

        rate.sleep()
        
        print("Loop end time: ", time.time() - startTime)
        print("================ Node 1 loop end ================\n")

    file.close()

    rospy.spin()

if __name__ == '__main__':
    main()