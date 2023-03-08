#!/usr/bin/env python
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.Inha_VelocityObstacle import VO_module
from functions.Inha_DecisionMaking import decision_making
from functions.data_vis import history
from functions.Inha_DataProcess import Inha_dataProcess
from functions.ShipSimulation import ShipSimulation

from inha_msg.msg import inha_topic
from udp_msgs.msg import frm_info, group_wpts_info 
# import imp
# decision_making = imp.load_compiled("decision_making", "src/Inha_DecisionMaking.cpython-37.pyc" )
# VO_module = imp.load_compiled("VO", "src/Inha_VelocityObstacle.cpython-37.pyc")
# INHA = imp.load_compiled("INHA", "src/Inha_DataProcess.cpython-37.pyc")

from numpy import rad2deg
import numpy as np
from math import sqrt, atan2
import rospy
import time
import rospkg
import yaml

class data_inNout:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(self):
        # Subscriber = input
        # rospy.Subscriber('/frm_info', frm_info, self.OP_callback) 
        # rospy.Subscriber('/waypoint_info', group_wpts_info, self.wp_callback)
        # self.WP_pub = rospy.Publisher('/vessel1_info', inha_topic, queue_size=10)
        totalDataSet = \
        {     
            "utcTime":1636622054,

            "localTime":1636622054,

            "latitude":801.7644,

            "directionOfLat":"N",

            "longitude":3230.5278,

            "directionOfLong":"W",

            "reliability":"V",

            "cog":56.6,

            "sog":88.5,

            "degreeOfAxis":[
                85.2,
                202,
                50.9
            ],

            "accelerationOfAxis":[
                338,
                172,
                197
            ],

            "angularVelocityOfAxis":"",

            "latOfNavigationArea":[
                4329.8088,
                26134.5409,
                1213.1299
            ],

            "longOfNavigationArea":[
                21842.1288,
                4403.191,
                29154.4987
            ],

            "latOfDepthOfWater":[
                21415.3584,
                35519.6891,
                2541.65
            ],

            "longOfDepthOfWater":[
                14556.5544,
                28848.9249,
                11148.4192
            ],

            "depthOfWater":5971,

            "idOfLimitedArea":769,

            "latOfLimitedArea":[
                27431.94,
                8349.797,
                31150.2704
            ],

            "longOfLimitedArea":[
                15045.7278,
                30646.555,
                30006.6574
            ],

            "idOfLandArea":"",

            "latOfLandArea":"",

            "longOfLandArea":"",

            "numOfObject":7,

            "idOfObject":[
                99,
                882,
                84,
                6,
                825,
                845,
                47
            ],

            "latOfObject":[
                -3942.3188,
                -4425.8525,
                -6334.8199,
                -5148.4418,
                5921.2571,
                -6824.8445,
                -6652.581
            ],

            "longOfObject":[
                -13401.5521,
                -10343.291,
                12715.8744,
                -14423.3403,
                12435.6026,
                13941.2942,
                -6520.7448
            ],

            "cogOfObject":[
                193,
                224.7,
                155.7,
                153.2,
                144,
                17.5,
                42.7
            ],

            "sogOfObject":[
                55.3,
                21.6,
                63.8,
                48.1,
                92.9,
                32.1,
                57.2
            ],

            "cpaOfObject":[
                56.6,
                78.5,
                41.8,
                73.2,
                38.5,
                81.4,
                91.7
            ],

            "dcpaOfObject":[
                51.4,
                30.9,
                4.3,
                11.2,
                18.3,
                2.5,
                23.5
            ],

            "tcpaOfObject":[
                3.7,
                4.2,
                56.5,
                10.2,
                39.3,
                19.5,
                10.7
            ],

            "collisionRiskIndex":"",

            "modifyWayPoint":"true",

            "numOfWayPoint":3,

            "latOfWayPoint":[
                22.222,
                33.333,
                44.444
            ],

            "longOfWayPoint":[
                22.222,
                33.333,
                44.444
            ],

            "speedOfWayPoint":[
                22.222,
                33.333,
                44.444
            ],

            "ETAOfWayPoint":[
                22.222,
                33.333,
                44.444
            ],

            "EDAOfWayPoint":[
                22.222,
                33.333,
                44.444
            ],

            "PDOfWayPoint":[
                22.222,
                33.333,
                44.444
            ],

            "reliabilityOfNewWayPoint":[
                22.222,
                33.333,
                44.444
            ],
            "error":True,
            "errorCode":"00",

            "lat_ts_wp":[
                51.4,
                30.9,
                4.3,
                11.2,
                18.3,
                2.5,
                23.5
            ],

            "long_ts_wp":[
                3.7,
                4.2,
                56.5,
                10.2,
                39.3,
                19.5,
                10.7
            ]
            }




        self.ship_ID = []
        self.len_waypoint_info = 0
        self.waypoint_dict = dict()
        self.TS_WP_index = []

        # for 유엔젤
        # OP_callback
        self.OS_ID = 2000
        self.ship_ID = totalDataSet['idOfObject']
        self.TS_ID = self.ship_ID[:]
        self.ship_ID.insert(0, self.OS_ID)

        self.os_Pos_X = totalDataSet['latitude']
        self.os_Pos_Y = totalDataSet['longitude']
        self.os_Vel_U = totalDataSet['sog']
        self.os_Heading = totalDataSet['cog'] % 360

        self.Pos_X = totalDataSet['latOfObject']
        self.Pos_Y = totalDataSet['longOfObject']
        self.Vel_U = totalDataSet['sogOfObject']
        self.Heading = totalDataSet['cogOfObject']
        self.TS_Pos_X = self.Pos_X[:]
        self.TS_Pos_Y = self.Pos_Y[:]
        self.TS_Vel_U = self.Vel_U[:]
        self.TS_Heading = self.Heading[:]

        self.Pos_X.insert(0, self.os_Pos_X)
        self.Pos_Y.insert(0, self.os_Pos_Y)
        self.Vel_U.insert(0, self.os_Vel_U)
        self.Heading.insert(0, self.os_Heading)

        # wp_callback
        self.len_waypoint_info = len(self.TS_ID)   # WP 딕셔너리에 TS먼저저장하기 위함
        wp_dic = dict()
        for i in range(self.len_waypoint_info):   
            shipID = self.TS_ID[i]
            wp_dic[f'{shipID}'] = {'wpts_x' : [totalDataSet['lat_ts_wp'][i]], 'wpts_y' : [totalDataSet['long_ts_wp'][i]]}
        wp_dic[f'{self.OS_ID}'] = {'wpts_x' : totalDataSet['latOfWayPoint'], 'wpts_y' : totalDataSet['latOfWayPoint']}
        self.waypoint_dict = wp_dic


    def wp_callback(self, wp):
        ''' subscribe `/waypoint_info`

        Example:
            OS_wpts_x = self.waypoint_dict['2000'].wpts_x
        '''
        # self.len_waypoint_info = len(wp.group_wpts_info)
        # wp_dic = dict()
        # for i in range(self.len_waypoint_info):
        #     shipID = wp.group_wpts_info[i].shipID
        #     wp_dic[f'{shipID}'] = wp.group_wpts_info[i]
        #     ## 위 처럼 표현할 경우, dictionary 안의 message 변수명을 알고 있어야 호출이 가능함!
        #     ## 따라서, 새로운 임의의 key Value로 바꾸서 저장하고 싶다면 아래와 같이 새로운 dictionary를 만들어도 됨. (이중 dictionary 구조)
        #     # wp_dic2 = dict()
        #     # wp_dic2['waypoint_x'] = wp.group_wpts_info[i].wpts_x
        #     # wp_dic2['waypoint_y'] = wp.group_wpts_info[i].wpts_y
            
        #     # wp_dic[f'{shipID}'] = wp_dic2

        # self.waypoint_dict = wp_dic
        

    def OP_callback(self, operation):
        ''' subscribe `/frm_info` 
        
        params : 
            `frm_info` 변수명은 입출력관계도 KRISO 참조

        Note :
            `psi`값이 [-2pi, 2pi]값으로 들어오므로, 편의상 강제로 [0, 2pi]로 변경
        '''
        # self.ship_ID = list(operation.m_nShipID)
        # self.Pos_X  = operation.m_fltPos_X
        # self.Pos_Y  = operation.m_fltPos_Y
        # self.Vel_U  = operation.m_fltVel_U
        # # self.Heading = operation.m_fltFOGang_yawzG
        # raw_psi = np.asanyarray(operation.m_fltFOGang_yawzG)
        # self.Heading = raw_psi % 360

    def path_out_publish(self, pub_list):
        ''' publish `/path_out_inha`
        
        Note :
            pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
        '''
        inha = inha_topic()                # 이건 모지? -> 커스텀메시지명
        inha.m_nShipID = pub_list[0]
        inha.isUpdateWP = pub_list[1]
        inha.numWP  = pub_list[2]
        inha.WP_x = pub_list[3]
        inha.WP_y = pub_list[4]
        inha.speedWP = round(pub_list[5], 4)
        inha.ETA_WP = round(pub_list[6], 3)
        inha.EDA_WP = round(pub_list[7], 3)
        inha.RI = pub_list[8]
        inha.CRI = pub_list[9]
        inha.isError = pub_list[10]
        # inha.errors\Code = ['0x00', '0x01']
        inha.errors = pub_list[11]
        inha.desired_spd = round(pub_list[12], 4)
        inha.desired_heading = round(pub_list[13], 3)
        inha.isNeedCA = pub_list[14]
        inha.status_btw_OS = pub_list[15]

        # self.WP_pub.publish(inha)
        print("==========================")


def main():  
    with open('./params/KASS_Coefficient.yaml') as f:
        KASS_coefficient = yaml.safe_load(f)

    with open('./params/main_parameter.yaml') as f:
        params = yaml.safe_load(f)

    Model = KASS_coefficient['Model']
    Coefficient = KASS_coefficient['Coefficient']     # 혜리 수정

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()  
    package_path = rospack.get_path('inha_modules')

    update_rate = params["update_rate"]
    dt = params["Ship_prediction"]["prediction_interval"] 
    break_prediction = params["Ship_prediction"]["prediction_end"]   

    node_Name = "vessel_node1"
    rospy.init_node(f"{node_Name}", anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    OS_ID = params["OS_info"]["ship_ID"]
    TS_ID = []

    # 자선의 정보
    OS_scale = params["OS_info"]["ship_scale"]
    target_spd = params["OS_info"]["target_speed"] * 0.5144 / sqrt(OS_scale)
    target_speed = [target_spd for i in range(2)]      
    radius_scale = params["OS_info"]["ship_domain_radius"]
    # ship_L 대상선의 길이! 
    ship_L = params["OS_info"]["ship_L"] ## 향후 이부분은, 1) ship domain과 2) AIS data의 선박의 길이 부분으로 나누어 고려 및 받아야 함!
    include_inha_modules = params["OS_info"]["include_inha_modules"]

    #### <<<<<<<<<<<<<<<< Check later !!! <<<<<<<<<<<<<<<<<<<< ####
    CA_starts = dict()
    CA_starts['DCPA'] = params["CA_starts"]["DCPA"]
    CA_starts['TCPA'] = params["CA_starts"]["TCPA"]
   
    
    # # !----- 설문조사에서는 충돌회피 시점은 12m 어선 기준으로 일반적으론 HO 3nm/ CS & OT 2nm을 기준으로 하고 있으며, 최소 안전 이격거리는 0.5~ 1nm으로 조사됨
    # # 다만, 2m급 모형선 테스트에서는 협소한 부분이 있으므로 스케일 다운(1/200)을 시켜서, "회피시점: 0.0015nm(27.78m) / 최소 안전 이격거리는 9.26m"가 되게끔 할 예정
    # # 참고 논문: https://www.koreascience.or.kr/article/JAKO201427542599696.pdf 
            
    vector_scale = 2.0  ## `data visualization(static plot)`에서 사용함

    #### <<<<<<<<<<<<<<<< Check later !!! <<<<<<<<<<<<<<<<<<< ####    
    # 향후 이 부분은 ECDIS 등 전자해도지도를 바탕으로 정적장애물 정보를 불러와야 함!!
    static_OB = dict()
    static_OB['circular_obstacles'] = []
    static_OB['robot_radius'] = 0.2
    #### >>>>>>>>>>>>>>>> Check later !!! >>>>>>>>>>>>>>>>>>>> ####    

    data = data_inNout()
    
    # inha = INHA.INHA(data.ship_ID, data.Pos_X, data.Pos_Y, data.Vel_U, data.Heading)                       # inha_module의 data 송신을 위해 필요한 함수들이 정의됨
    # decision_make = decision_making.decision_making()   # 충돌회피를 위한 의사 결정에 필요한 함수들이 정의됨
    # Local_PP = VO_module.VO_module()              # 충돌회피 경로생성을 위한 Velocity Obstacle 기법에 관련된 함수가 정의됨
    decision_make = decision_making()
    Local_PP = VO_module()
    history_collector = history()
    
    # data visualization(static plot)에 사용됨
    t =0
    Time = 0
    map_size = [-30.0, 60.0, -30.0, 30.0] # [x_min, x_max, y_min, y_max]    
    waypointIndex = 0   

    # while not rospy.is_shutdown():
        # if len(data.ship_ID) == 0:
        #     ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
        #     print(f"========= Waiting for `/frm_info` topic subscription in {node_Name}=========")
        #     rate.sleep()
        #     continue

        # if data.len_waypoint_info == 0:
        #     ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
        #     print(f"========= Waiting for `/waypoint_info` topic subscription in {node_Name} =========")
        #     rate.sleep()
        #     continue


        # rospy.loginfo("=============================================================================================")

    inha = Inha_dataProcess(data.ship_ID, data.Pos_X, data.Pos_Y, data.Vel_U, data.Heading, data.waypoint_dict)                       # inha_module의 data 송신을 위해 필요한 함수들이 정의됨

    ## <======== 서울대학교 전역경로를 위한 waypoint 수신 및 Local path의 goal로 처리
    wpts_x_os = list(data.waypoint_dict[f'{OS_ID}']['wpts_x'])
    wpts_y_os = list(data.waypoint_dict[f'{OS_ID}']['wpts_y'])

    Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]           # waypoint list에서 1개의 waypoint 만을 추출

    ## <========= `/frm_info`를 통해 들어온 자선 타선의 데이터 전처리
    ship_list, ship_ID = inha.ship_list_container(OS_ID)
    OS_list, TS_list = inha.classify_OS_TS(ship_list, ship_ID, OS_ID)   
    
    TS_ID = ship_ID[:]  ## 리스트 복사
    TS_ID.remove(OS_ID)

    #----------- OS -> vx, vy 추가
    ## Heading angle 바탕으로 속도 벡터 구하는 함수
    OS_Vx, OS_Vy = inha.U_to_vector_V(OS_list.loc[OS_ID, 'Vel_U'], OS_list.loc[OS_ID, 'Heading'])
    OS_list.loc[OS_ID, 'V_x'] = OS_Vx
    OS_list.loc[OS_ID, 'V_y'] = OS_Vy
    # OS_list['radius'] = ship_L + OS_list.loc[OS_ID]['Vel_U'] * radius_scale
    OS_list.loc[OS_ID, 'radius'] = ship_L  * radius_scale

    OS_V_current = [OS_Vx, OS_Vy]
    OS_X_current = [OS_list.loc[OS_ID, 'Pos_X'], OS_list.loc[OS_ID, 'Pos_Y']]

    _, local_goal_EDA = inha.eta_eda_assumption(Local_goal, OS_X_current, target_speed[0])   

    # OS_dic = OS_list.to_dict()     
    # <=========== VO 기반 충돌회피를 위한 경로 생성
    # !--- 1) `Local goal`으로 향하기 위한 속도벡터 계산
    V_des = Local_PP.vectorV_to_goal(OS_list, Local_goal, target_speed)    
    V_des_Heading = rad2deg(atan2(V_des[1], V_des[0]))

    '''
    OS list =  {'Ship_ID': [], 'Pos_X' : [],  'Pos_Y' : [],   'Vel_U' : [],   'Heading' : [], 
                'V_x' : [], 'V_y' : [], 'radius' : []}
    '''

    # !--- 2) CRI를 기준으로 충돌회피 시점을 정의
    # <<<!------충돌회피 모듈이 필요한 시점 인지 판단 --->>>  
    # `if isNeedCA == True: 충돌회피가 필요함`
    TS_list = inha.TS_info_supplement(OS_list, TS_list, ship_L, radius_scale, V_des_Heading, CA_starts, local_goal_EDA)
    '''
    TS list =  {'Ship_ID': [], 'Pos_X' : [],  'Pos_Y' : [],   'Vel_U' : [],   'Heading' : [], 'V_x' : [], 'V_y' : [], 
                'radius' : [], 'RD' : [], 'RB' : [],'RC' : [], 'local_rc' : [], 'status' : []}
    '''
    TS_list = decision_make.getting_farther_away(TS_list)
    isNeedCA = decision_make.isNeedCA(TS_list)
    isNeedCA_original = [isNeedCA][:]

    TS_sort_condition = (TS_list['isNeedCA'] == True) 
    TS_list_sort = TS_list[TS_sort_condition]  ## 전체 TS List들 중에 충돌회피가 필요로 하는 TS만 추출함

    # rospy.loginfo(TS_list)

    if include_inha_modules == False:
        ## 만약, `include_inha_modules`이 false 라면, 충돌회피가 작동하지 않고, 전역경로만을 추종하게끔 구성
        isNeedCA = False
    
    TS_status_temp = []                
    for ts_ID in TS_ID:
        temp_status = TS_list.loc[ts_ID,'status']
        TS_status_temp.append(temp_status)

    if isNeedCA == False:
        # !--- 충돌위험이 없다면 전역경로 추종      
        wp_x = Local_goal[0] ; wp_y = Local_goal[1]
        eta, eda = inha.eta_eda_assumption([wp_x, wp_y], OS_X_current, target_speed[0])
        desired_spd, desired_heading = inha.desired_value_assumption(V_des)
        VO_BA_all = None
        wp_x_list = [wp_x]
        wp_y_list = [wp_y]

    if isNeedCA:                    
        # !--- 3) VO기반 충돌회피를 위한 속도벡터 선정
        # !--- 4) 속도벡터 기반 waypoint 선정  
        # !--- 5)  `ETA_WP` & `EDA_WP` 계산 + 현 위치를 기준으로 `desired_spd` & `desired_heading` 계산
        V_opt, VO_BA_all = Local_PP.VO_update(OS_list, TS_list_sort, static_OB, V_des, target_speed)

        wp_x_list_temp = []
        wp_y_list_temp = []
        pos_x_list = []
        pos_y_list = []
        desired_spd_list = []
        desired_heading_list = []
        OS_list_copy = OS_list.copy()
        TS_list_copy = TS_list.copy()

        os_dynamics = ShipSimulation(OS_list.loc[OS_ID, 'Pos_X'], OS_list.loc[OS_ID, 'Pos_Y'], OS_list.loc[OS_ID, 'Vel_U'], \
                                        OS_list.loc[OS_ID, 'Heading'], 0.0, OS_scale, dt)

        start_while_break_time = time.time()
        while isNeedCA == True:
            wp_x, wp_y = inha.waypoint_generator(OS_X_current, V_opt)
            wp_x_list_temp.append(wp_x)
            wp_y_list_temp.append(wp_y)

            eta, eda = inha.eta_eda_assumption([wp_x, wp_y], OS_X_current, target_speed[0])            
            temp_spd, temp_heading = inha.desired_value_assumption(V_opt)
            desired_spd_list.append(temp_spd)
            desired_heading_list.append(temp_heading)

            # rospy.loginfo(TS_list_copy)

            temp_x, temp_y, temp_U, temp_psi, _ = os_dynamics.moving_ships(temp_heading, temp_spd)
            pos_x_list.append(temp_x)
            pos_y_list.append(temp_y)
            OS_list_copy.loc[OS_ID, 'Pos_X'] = temp_x
            OS_list_copy.loc[OS_ID, 'Pos_Y'] = temp_y
            OS_list_copy.loc[OS_ID, 'Vel_U'] = temp_U
            OS_list_copy.loc[OS_ID, 'Heading'] = temp_psi

            TS_list_copy = inha.nextStep_TSprediction(OS_list_copy, TS_list_copy, ship_L, radius_scale, V_des_Heading, CA_starts, local_goal_EDA, dt)
            # rospy.loginfo(TS_list_copy)
            isNeedCA = decision_make.isNeedCA(TS_list_copy)
            
            end_while_break_time = time.time()       
            break_time = end_while_break_time - start_while_break_time
            
            '''
            # if isNeedCA == False or break_time >= 2.0:
            '''
            if isNeedCA == False or break_time >= break_prediction:
                announce_break_time = f"When does the estimated loop end? ? : {break_time} sec" 
                rospy.loginfo(announce_break_time)
                break
            TS_sort_copy_cond = (TS_list_copy['isNeedCA'] == True) 
            TS_list_copy_sort = TS_list_copy[TS_sort_copy_cond]  ## 전체 TS List들 중에 충돌회피가 필요로 하는 TS만 추출함
            V_opt, VO_BA_all = Local_PP.VO_update(OS_list_copy, TS_list_copy_sort, static_OB, V_des, target_speed)

        desired_spd = desired_spd_list[0]
        desired_heading = desired_heading_list[0]

        wp_x_list = [wp_x_list_temp[0]] + pos_x_list
        wp_y_list = [wp_y_list_temp[0]] + pos_y_list
                        
        # CC_loginfo_msg = f"Collision cone?? : {VO_BA_all}" 
        # rospy.loginfo(CC_loginfo_msg)
        # # VO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]            

        # wp_x, wp_y = inha.waypoint_generator(OS_X_current, V_opt)
        # eta, eda = inha.eta_eda_assumption([wp_x, wp_y], OS_X_current, target_speed[0])            
        # desired_spd, desired_heading = inha.desired_value_assumption(V_opt)

    # !--- 6) 생성경로 신뢰성 지수 계산
    if isNeedCA :

        # RI 계산하는 코드 추가 예정 
        pass

    # !--- data visualization
    if t%10 ==0:
        # history_collector.visualize_traj_dynamic(static_OB, OS_list, TS_list_sort, Local_goal, [wp_x, wp_y],
        #                              vector_scale, VO_BA_all, map_size, Time, 
        #                              name=f'{package_path}/Fig/snap%s.png'%str(t/10))
        pass

    t += 1
    Time = t/update_rate    

    # # < =========  인하대 모듈에서 나온 데이터를 최종적으로 송신하는 부분
    # OS_pub_list = [int(OS_ID), False, waypointIndex, [wp_x], [wp_y],desired_spd, eta, eda, 0.5, 0.0, False, [], desired_spd, desired_heading, isNeedCA, ""]
    OS_pub_list = [int(OS_ID), False, waypointIndex, wp_x_list, wp_y_list, desired_spd, eta, eda, 0.5, 0.0, False, [], desired_spd, desired_heading, isNeedCA_original[0], f"{TS_status_temp}"]
    
    # if len(data.TS_WP_index) == 0:
    #     data.TS_WP_index = [0 for i in range(len(TS_ID))]
    # TS_WP_index = data.TS_WP_index
    
    # TS_pub_list, TS_WP_index = inha.TS_prediction(TS_list, ship_L, TS_target_spds_dic, TS_WP_index)
    # pub_list = [OS_pub_list] + TS_pub_list
    # # pub_list : [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
    # data.path_out_publish(pub_list, start_time)     
    # data.TS_WP_index = TS_WP_index
    data.path_out_publish(OS_pub_list)   

    if local_goal_EDA < 2 * ship_L :
    # 만약 `reach criterion`와 거리 비교를 통해 waypoint 도달하였다면, 
    # 앞서 정의한 `waypint 도달 유무 확인용 flag`를 `True`로 바꾸어 `while`문 종료
        waypointIndex = (waypointIndex + 1) % len(wpts_x_os)

    # rate.sleep()
    print(OS_pub_list)

    # rospy.spin()


if __name__ == '__main__':
    main()
