#!/usr/bin/env python
import sys, os

from numpy import deg2rad
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.ShipSimulation import ShipSimulation
from functions.InfoLoader import InfoLoader
from udp_col_msg.msg import path_output
from udp_msgs.msg import frm_info
import rospy
from math import sqrt

class KRISO:
    """22m급의 KASS MMG 운항 모델 반영"""
    def __init__(self):
        rospy.Subscriber('/path_out_inha2', path_output, self.inha_callback)
        self.OS_pub = rospy.Publisher('/AIS_data', frm_info, queue_size=10)

        self.len_path_out_inha = 0
        self.path_out_inha_dic = dict()

    def inha_callback(self, WP):
        """`/path_out_inha`의 데이터를 모두 저장하고 있음
        
        Note :
            inha_dic[f'{shipID}'] = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desired_spd, desired_heading, isNeedCA, "status_btw_OS"]
        Example:
            `desired_spd = self.path_out_inha_dic[f'{ship_ID}'].desired_spd`
        """

        self.len_path_out_inha = len(WP.pathData)

        inha_dic = dict()
        for i in range(self.len_path_out_inha):
            shipID = WP.pathData[i].nship_ID
            inha_dic[f'{shipID}'] = WP.pathData[i]

        self.path_out_inha_dic = inha_dic
        
    def frm_info_publish(self, ship_ID, Pos_X, Pos_Y, vel, psi_deg, delta_deg, start_time):
        """ 전체 `/frm_info`중 인하대 node에서 필요한 위치 및 속도 정보 생성 """
        kriso = frm_info()
        kriso.header.stamp = rospy.Time.now() - start_time
        kriso.header.frame_id = "ship_info"

        kriso.m_nShipID  = ship_ID
        kriso.m_fltPos_X = Pos_X
        kriso.m_fltPos_Y = Pos_Y
        kriso.m_fltVel_U = vel
        kriso.m_fltHeading = psi_deg
        kriso.m_fltRudderAngleFeedSTBD = delta_deg

        self.OS_pub.publish(kriso)
        # rospy.loginfo(kriso)

def main():    
    rospy.init_node('KRISO', anonymous=False)   
    update_rate = rospy.get_param("update_rate")
    rate = rospy.Rate(update_rate) # 10 Hz renew
    dt = rospy.get_param("mmg_dt")
    kriso = KRISO()
    shipsInfo = InfoLoader(rospy.get_param("shipInfo_all"))

    # Initialize the ship instances to simulate
    shipState_all = dict()
    shipInstance_all = dict()
    latest_shipState_all = dict()

    for shipName in shipsInfo.shipName_all:
        # Get the initial states
        paramStr_shipID = "shipInfo_all/" + shipName + "_info/ship_ID"
        paramStr_shipScale = "shipInfo_all/" + shipName + "_info/ship_scale"
        paramStr_initStartX = "shipInfo_all/" + shipName + "_info/initial_start_x"
        paramStr_initStartY = "shipInfo_all/" + shipName + "_info/initial_start_y"
        paramStr_initStartU = "shipInfo_all/" + shipName + "_info/initial_start_U"
        paramStr_initStartPsi = "shipInfo_all/" + shipName + "_info/initial_start_psi"
        paramStr_LBP = "shipInfo_all/" + shipName + "_info/ship_L"

        shipState_all[shipName] = dict()
        shipState_all[shipName]["shipID"] = rospy.get_param(paramStr_shipID)
        shipState_all[shipName]["scale"] = rospy.get_param(paramStr_shipScale)
        shipState_all[shipName]["X"] = rospy.get_param(paramStr_initStartX)
        shipState_all[shipName]["Y"] = rospy.get_param(paramStr_initStartY)
        shipState_all[shipName]["U"] = rospy.get_param(paramStr_initStartU) * 0.5144 / sqrt(shipState_all[shipName]["scale"])
        shipState_all[shipName]["u"] = shipState_all[shipName]["U"]
        shipState_all[shipName]["v"] = 0
        shipState_all[shipName]["psi_deg"] = rospy.get_param(paramStr_initStartPsi)
        shipState_all[shipName]["delta_deg"] = 0.0
        shipState_all[shipName]["LBP"] = rospy.get_param(paramStr_LBP)

        # Get the ships instances
        shipInstance_all[shipName] = ShipSimulation(
            shipState_all[shipName]["X"],
            shipState_all[shipName]["Y"],
            shipState_all[shipName]["U"],
            shipState_all[shipName]["u"],
            shipState_all[shipName]["v"],
            shipState_all[shipName]["psi_deg"],
            shipState_all[shipName]["delta_deg"],
            shipState_all[shipName]["LBP"],
            shipState_all[shipName]["scale"],
            dt,
            )
        
    start_time = rospy.Time.now()

    last_publish_time = rospy.Time.now()  # 마지막으로 발행한 시간을 초기화
    delay=rospy.get_param('ais_delay')
    publish_interval = rospy.Duration(delay)  # 발행 주기를 5초로 설정
    # publish_interval = rospy.Duration(random.uniform(1.0, 5.0))  # 발행 주기 랜덤
    # print("발행 주기(초):", publish_interval.to_sec())
    # publish_interval = rospy.Duration(random.uniform(1.0, 5.0))  # 발행 주기 랜덤

    latest_desired_Heading = shipState_all["ship2"]["psi_deg"]
    latest_shipState_all[shipName] = []
    first_publish = True
    diff = 0.0

    while not rospy.is_shutdown():  
        current_time = rospy.Time.now()  # 현재 시간을 계속 추적

        # Pack the state information to publish
        shipID_all = [shipState_all[shipName]['shipID'] for shipName in shipsInfo.shipName_all]
        Pos_X_all = [shipState_all[shipName]['X'] for shipName in shipsInfo.shipName_all]
        Pos_Y_all = [shipState_all[shipName]['Y'] for shipName in shipsInfo.shipName_all]
        Vel_U_all = [shipState_all[shipName]['U'] for shipName in shipsInfo.shipName_all]
        Heading_deg_all = [shipState_all[shipName]['psi_deg'] for shipName in shipsInfo.shipName_all]
        delta_deg_all = [shipState_all[shipName]['delta_deg'] for shipName in shipsInfo.shipName_all]

        kriso.frm_info_publish(
                shipID_all, 
                Pos_X_all, 
                Pos_Y_all, 
                Vel_U_all, 
                Heading_deg_all, 
                delta_deg_all, 
                start_time, 
            )
            
        if kriso.len_path_out_inha == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/path_out_inha` topic subscription =========")
            rate.sleep()
            continue

        # shipsInfo.shipName_all의 총 길이
        total_ships = len(shipsInfo.shipName_all)

        # 현재 인덱스를 추적하기 위한 변수
        current_index = 0

        # Update the ship state
        for shipName in shipsInfo.shipName_all:
            current_index += 1

            if shipName == "ship1":
                shipID = shipState_all[shipName]['shipID']
                desired_Heading = kriso.path_out_inha_dic[f'{shipID}'].targetCourse
                desired_spd = kriso.path_out_inha_dic[f'{shipID}'].targetSpeed
                shipState_all[shipName] = {**{'shipID': shipID} , **shipInstance_all[shipName].moving_ships(desired_Heading, desired_spd)}
            else:
                shipID = shipState_all[shipName]['shipID']
                desired_Heading = kriso.path_out_inha_dic[f'{shipID}'].targetCourse
                desired_spd = kriso.path_out_inha_dic[f'{shipID}'].targetSpeed
                shipState_all[shipName] = {**{'shipID': shipID} , **shipInstance_all[shipName].moving_ships(desired_Heading, desired_spd)}
                # print(desired_Heading)

                if (current_time - last_publish_time >= publish_interval) or first_publish:
                    # if current_index - 1 >= len(latest_desired_Heading):
                    #     latest_desired_Heading.append(desired_Heading)
                    # else:
                    #     latest_desired_Heading[current_index - 1] = desired_Heading
                    # latest_shipState_all[shipName] = shipState_all[shipName]
                    latest_shipState_all[shipName] = shipState_all[shipName]
                    # print("갱신")
                    if current_index == total_ships:
                        last_publish_time = current_time  # 마지막 발행 시간을 현재 시간으로 업데이트
                # print(latest_desired_Heading)     
                # 차이 계산
                    latest_desired_Heading = desired_Heading
                diff = latest_desired_Heading - desired_Heading
                if diff > 180:
                    diff -= 360
                elif diff < -180:
                    diff += 360
                else:
                    diff = diff
                # print(diff)

                if abs(diff) >= 0.7:
                    shipID = shipState_all[shipName]['shipID']
                    desired_Heading = kriso.path_out_inha_dic[f'{shipID}'].targetCourse
                    desired_spd = kriso.path_out_inha_dic[f'{shipID}'].targetSpeed
                    shipState_all[shipName] = {**{'shipID': shipID} , **shipInstance_all[shipName].moving_ships(desired_Heading, desired_spd)}
                    # latest_desired_Heading = desired_Heading
                    # print("헤딩 변화")
                else:
                    shipState_all[shipName] = latest_shipState_all[shipName]
                    # print("지연")

        first_publish = False  # 첫 번째 발행이 끝났으니 플래그를 False로 설정
        
        rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()
