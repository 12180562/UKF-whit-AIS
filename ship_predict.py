#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from functions.Inha_DataProcess import Inha_dataProcess

from udp_msgs.msg import frm_info

import numpy as np
import rospy
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
    
class UKF:
    '''
    혹시나 나중에 csv파일로 뽑아야 한다면 이걸로 하기
    class UKFNode:
        def __init__(self):
            rospy.init_node('ukf_node')

            # CSV 파일 초기화
            current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.file_name = f"ukf_results_{current_time}.csv"
            self.csv_file = open(self.file_name, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Time', 'Ship ID', 'Latitude', 'Longitude'])
            
            self.subscriber = rospy.Subscriber('AIS_data', String, self.sensor_callback)
            self.ships = {}

        def sensor_callback(self, data):
            ship_id, lat, lon, speed, heading = data.data.split(',')
            ship_id = ship_id
            current_measurement = np.array([float(lat), float(lon), float(speed), float(heading)])

            if ship_id not in self.ships:
                self.ships[ship_id] = UKF()

            predicted_state = self.ships[ship_id].update_ukf(current_measurement)

            # 현재 시간, 선박 ID, 위도, 경도 추출 및 CSV 파일에 기록
            time = rospy.get_time()
            self.csv_writer.writerow([time, ship_id, predicted_state[0], predicted_state[1]])
            self.csv_file.flush()
        
            rospy.on_shutdown(self.on_shutdown())

        def on_shutdown(self):
            self.csv_file.close()
    '''
    def __init__(self):
        rospy.Subscriber('/AIS_data', frm_info, self.OP_callback)
        self.OS_pub = rospy.Publisher('/frm_info', frm_info, queue_size=10)

        self.dt = rospy.get_param('ukf_dt')  # 샘플링 시간

        self.last_measurement = None
        self.last_heading = None
        self.ukf_initialized = False

        self.initialize_ukf()
        # print("durldurl")
        self.waypoint_idx = 0
        self.len_waypoint_info = 0
        self.waypoint_dict = dict()

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

    def initialize_ukf(self):
        '''Alpha (α):
        alpha는 시그마 포인트의 분포를 결정하는 스케일링 파라미터입니다. alpha는 0과 1 사이의 작은 양수로 설정되며, 일반적으로 매우 작은 값(예: 1e-3)을 사용합니다.
        alpha가 작을수록 시그마 포인트들은 평균 근처에 더 가깝게 위치하며, alpha가 커지면 시그마 포인트들은 멀리 퍼지게 됩니다. 이는 곧 필터의 추정치가 측정치에 대해 얼마나 신뢰를 두는지에 영향을 미칩니다.
        alpha를 너무 크게 설정하면 필터가 불안정해질 수 있으며, 너무 작게 설정하면 필터가 새로운 측정치에 너무 느리게 반응할 수 있습니다.

        Beta (β):
        beta는 사전 분포에 대한 지식을 통합하는 파라미터입니다. 특히, beta는 상태 변수의 분포가 가우시안 분포에서 벗어날 때 사용됩니다.
        beta가 2일 경우 이는 시스템의 초기 분포가 정확히 가우시안(정규 분포)임을 의미합니다. 다른 값은 비가우시안 분포를 가정할 때 사용됩니다.
        beta를 조정함으로써 필터의 성능을 비선형성이 강한 시스템에 맞출 수 있습니다. 일반적으로, beta는 0 또는 2 근처의 값을 사용하지만, 시스템의 특성에 따라 최적의 값을 찾기 위한 실험이 필요할 수 있습니다.
        
        Kappa (κ):
        kappa는 보통 0 또는 3-n (여기서 n은 상태 변수의 차원)으로 설정됩니다. kappa는 시그마 포인트 생성시 중심 포인트의 가중치를 조정합니다.
        kappa를 조정함으로써 필터의 안정성과 정확성을 향상시킬 수 있습니다. 특히, kappa를 사용하여 비선형 시스템의 특성을 더 잘 반영할 수 있습니다.'''
        sigma_points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=12)
        self.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=4, dt=self.dt, fx=self.state_transition, hx=self.measurement_function, points=sigma_points)
        self.ukf.x = np.array([0., 0., 0., 0.])  # 초기 상태 추정치
        self.ukf.P = np.eye(4) * 10000.  # 초기 공분산 행렬
        # self.ukf.P *= 10  # 초기 공분산 행렬

        self.ukf.R = np.eye(4) * .1  # 측정 노이즈
        self.ukf.Q = np.eye(4) * .1  # 프로세스 노이즈

    def state_transition(self, x, dt):
        # 여기에 mmg가 들어가야함 movingship
        x_new = np.zeros_like(x)
        x_new[0] = x[0] + x[2] * np.cos(np.deg2rad(x[3])) * dt  # x 위치 업데이트
        x_new[1] = x[1] + x[2] * np.sin(np.deg2rad(x[3])) * dt  # y 위치 업데이트
        x_new[2] = x[2]  # 속도는 변하지 않는다고 가정
        x_new[3] = x[3]  # 방향은 변하지 않는다고 가정

        return x_new
    
    def measurement_function(self, x):
        return x

    def update_ukf(self, lat, long, speed, heading):

        measurement = [lat, long, speed, heading]

        if not self.ukf_initialized:
            self.ukf.x = measurement  # 초기 상태 추정치 설정
            self.ukf_initialized = True  # UKF가 초기화되었음을 표시
            # rospy.loginfo("UKF initialized with first measurement: %s", self.ukf.x)
            
        # 측정값이 변경되었는지 확인
        if self.last_measurement is not None and np.array_equal(measurement, self.last_measurement):

            self.last_heading = self.ukf.x[3]

            # 측정값이 변경되지 않았다면 predict만 수행
            self.ukf.predict()

            self.predicted_values.append(self.ukf.x)

        else:
            # 측정값이 변경되었다면 예측 및 업데이트 수행
            if self.last_measurement is not None:
                # 각도 변화량 계산 및 업데이트 로직 적용
                self.last_heading = self.last_measurement[3]  # 이전 헤딩 값을 업데이트
            self.last_measurement = measurement

            self.ukf.predict()
            self.ukf.update(measurement)

            self.predicted_values = []

        # for i in range(4):
        #     # for j in 4:
        #     print(self.ukf.P[i][i])    
        # print(self.ukf.P)
        return self.ukf.x #, self.ukf.P
    
def main():
    rospy.init_node('ship_predict', anonymous=False)
    update_rate = rospy.get_param("update_rate")
    rate = rospy.Rate(update_rate) # 10 Hz renew
    
    ukf = UKF()
    ukf_instances = {}
    
    OS_ID = rospy.get_param("shipInfo_all/ship1_info/ship_ID")
    first_loop = True  # 첫 번째 루프 실행 여부를 추적하는 변수
    
    waypointIndex = 0

    while not rospy.is_shutdown():
        inha = Inha_dataProcess(
            ukf.ship_ID,
            ukf.Pos_X, 
            ukf.Pos_Y, 
            ukf.Vel_U, 
            ukf.Heading,
            ukf.waypoint_dict,
            Pre_X,
            Pre_Y
            )
           
        wpts_x_os = list(ukf.waypoint_dict['{}'.format(OS_ID)].wpts_x)
        wpts_y_os = list(ukf.waypoint_dict['{}'.format(OS_ID)].wpts_y)
        Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]        
        ship_list, ship_ID = inha.ship_list_container(OS_ID)

        if first_loop:
            for ship_id in ukf.ship_ID:
                # 각 선박 ID에 대해 독립적인 UKF 인스턴스 생성 및 저장
                ukf_instances[ship_id] = UKF()

            first_loop = False  # 첫 번째 루프가 실행된 후에는 이 조건을 더 이상 만족시키지 않음

        for ship_id, ship_info in ship_list.items():

            # 해당 선박의 UKF 인스턴스를 사용하여 업데이트

            if ship_id == OS_ID:
                ship_list[OS_ID].update({
                    'Pos_X' : ship_info['Ori_X'],
                    'Pos_Y' : ship_info['Ori_Y'],
                    })
                
            else:
                # 발행 주기 사이에는 마지막으로 업데이트된 상태 정보를 유지하며 발행
                predicted_state = ukf_instances[ship_id].update_ukf(ship_list[ship_id]['Ori_X'], ship_list[ship_id]['Ori_Y'], ship_list[ship_id]['Vel_U'], ship_list[ship_id]['Heading'])
                Pre_X = predicted_state[0]
                Pre_Y = predicted_state[1]

                # 예측 진행
                ship_list[ship_id].update({
                    'Ship_ID' : ship_list[ship_id]['Ship_ID'],
                    'Ori_X' : ship_list[ship_id]['Ori_X'],
                    'Ori_Y' : ship_list[ship_id]['Ori_Y'],
                    'Vel_U' : ship_list[ship_id]['Vel_U'],
                    'Heading' : ship_list[ship_id]['Heading'],
                    'Pos_X' : Pre_X,
                    'Pos_Y' : Pre_Y,
                    })

        # print(ship_list)
        # print("예측됨 X: {}, Y: {}".format(ship_list[OS_ID]['next_X'], ship_list[OS_ID]['next_Y']))
        print("plz")
        rate.sleep()
        
    rospy.spin()

if __name__ == '__main__':
    main()
