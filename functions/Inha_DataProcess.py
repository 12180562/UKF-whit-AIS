import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.CRI import CRI
from numpy import deg2rad, rad2deg
from math import sin, cos, pi, sqrt, atan2

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
        self.dt = rospy.get_param('ukf_dt')  # 샘플링 시간

        self.last_measurement = None
        self.last_heading = None
        self.ukf_initialized = False

        self.initialize_ukf()

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
        sigma_points = MerweScaledSigmaPoints(n=4, alpha=.1, beta=2., kappa=.1)
        self.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=4, dt=self.dt, fx=self.state_transition, hx=self.measurement_function, points=sigma_points)
        self.ukf.x = np.array([0., 0., 0., 0.])  # 초기 상태 추정치
        self.ukf.P = np.eye(4) * 10000.  # 초기 공분산 행렬
        # self.ukf.P += np.eye(self.ukf.P.shape[0]) * 1e-6
        self.ukf.R = np.eye(4) * .5  # 측정 노이즈
        self.ukf.Q = np.eye(4) * .5  # 프로세스 노이즈

    def state_transition(self, x, dt):
        update_rate = rospy.get_param('update_rate') -1 
        angle_dt = rospy.get_param('ukf_angle_dt')  # 각속도 조정 파라미터
        new_x = x.copy()
        # print(new_x)
        # print('gg')
        if self.last_heading is not None:
            heading_change = x[3] - self.last_heading
            angular_velocity = heading_change / ((angle_dt + 0.000001) * update_rate)
            new_x[3] += angular_velocity
        else:
            new_x[3] = x[3]

        new_x[0] += dt * (x[2] * np.cos(np.deg2rad(new_x[3])))/update_rate
        new_x[1] += dt * (x[2] * np.sin(np.deg2rad(new_x[3])))/update_rate

        # print(new_x)
        return new_x
    
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
            # predict=self.ukf.predict()
            # self.ukf.update(predict)
            # print('predict')
            # rospy.loginfo("Measurement unchanged, prediction only: %s", self.ukf.x)
            self.predicted_values.append(self.ukf.x)

        else:
            self.initialize_ukf()

            # 측정값이 변경되었다면 예측 및 업데이트 수행
            if self.last_measurement is not None:
                # 각도 변화량 계산 및 업데이트 로직 적용
                self.last_heading = self.last_measurement[3]  # 이전 헤딩 값을 업데이트
            self.last_measurement = measurement

            self.ukf.predict()
            self.ukf.update(measurement)
            # print('update')
            # rospy.loginfo("Measurement updated, state updated: %s", self.ukf.x)
            self.predicted_values = []

        return self.ukf.x
 
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
        Pre_X,
        Pre_Y
        ):

        self.ship_ID = ship_ID
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Vel_U = Vel_U
        self.Heading = Heading
        self.waypoint_dict = waypoint_dict
        self.Pre_X = Pre_X
        self.Pre_Y = Pre_Y

        self.ship_dic= {}
        self.SD_param = rospy.get_param('SD_param')

    def ship_list_container(self, OS_ID):
        ''' 
            Subscribe한 선박의 운항정보를 dictionary로 저장 
        
            Return : 
                ship_list_dic
                ship_ID
        '''

        for i in range(len(self.ship_ID)):
            index_ship = self.ship_ID[i]
            if index_ship == OS_ID:
                self.ship_dic[OS_ID]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Ori_X' : self.Pos_X[i],
                    'Ori_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    'Pos_X' : self.Pre_X,
                    'Pos_Y' : self.Pre_Y,
                    }

            else:
                self.ship_dic[index_ship]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Ori_X' : self.Pos_X[i],
                    'Ori_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    'Pos_X' : self.Pre_X,
                    'Pos_Y' : self.Pre_Y,
                    }

        # print(self.ship_dic)
        # print(self.ship_ID)
        # print("원래 X: {}, Y: {}".format(self.ship_dic[OS_ID]['Pos_X'], self.ship_dic[OS_ID]['Pos_Y']))

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
        # print(TS_list)
        return OS_list, TS_list

    def CRI_cal(self, OS, TS):
        cri = CRI(
            rospy.get_param("shipInfo_all/ship1_info/ship_L"),
            rospy.get_param("shipInfo_all/ship1_info/ship_B"),
            OS['Pos_X'],
            OS['Pos_Y'],
            TS['Pos_X'],
            TS['Pos_Y'],
            deg2rad(OS['Heading']),
            deg2rad(TS['Heading']),
            OS['Vel_U'],
            TS['Vel_U'],
            rospy.get_param("shipInfo_all/ship1_info/ship_scale"),
        )

        RD = cri.RD()
        TB = rad2deg(cri.TB())
        RB = rad2deg(cri.RB())

        Vox = cri.Vox()
        Voy = cri.Voy()
        Vtx = cri.Vtx()
        Vty = cri.Vty()

        DCPA = cri.dcpa()
        TCPA = cri.tcpa()

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
        rb, lb = cri.SD_dist_new()

        cri_value = cri.CRI()

        return RD, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value, rb,lb

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
        ''' `V_des` 방향 벡터를 기준으로 1초뒤 point를 waypoint 생성
        
        Return :
            wp_x, wp_y [m]
        '''

        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])

        wp = OS_X + V_selected * dt

        wp_x = wp[0]
        wp_y = wp[1]

        return wp_x, wp_y

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


    def TS_info_supplement(self, OS_list, TS_list):   
        """ TS에 대한 추가적인 information 생성 

        Returns:
            TS_info : pandas dataframe

        Note : 
            TS list =  {'Ship_ID': [], 'Pos_X' : [],  'Pos_Y' : [],   'Vel_U' : [],   'Heading' : [], 'V_x' : [], 'V_y' : [], 
                        'radius' : [], 'RD' : [], 'RB' : [],'RC' : [], 'local_rc' : [], 'status' : []}
        """

        if TS_list == None:
            TS_list = None
        else:
            TS_ID = TS_list.keys()
            for ts_ID in TS_ID:
                RD, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value,rb,lb = self.CRI_cal(OS_list, TS_list[ts_ID])

                TS_list[ts_ID]['RD'] = RD 
                TS_list[ts_ID]['TB'] = TB  
                TS_list[ts_ID]['RB'] = RB

                TS_list[ts_ID]['V_x'] = Vtx
                TS_list[ts_ID]['V_y'] = Vty

                TS_list[ts_ID]['DCPA'] = DCPA
                TS_list[ts_ID]['TCPA'] = TCPA

                TS_list[ts_ID]['UDCPA'] = UDCPA
                TS_list[ts_ID]['UTCPA'] = UTCPA
                TS_list[ts_ID]['UD'] = UD
                TS_list[ts_ID]['UB'] = UB
                TS_list[ts_ID]['UK'] = UK

                TS_list[ts_ID]['status'] = enc

                TS_list[ts_ID]['Rf'] = Rf
                TS_list[ts_ID]['Ra'] = Ra
                TS_list[ts_ID]['Rs'] = Rs
                TS_list[ts_ID]['Rp'] = Rp
                TS_list[ts_ID]['mapped_radius'] = SD_dist * self.SD_param
                TS_list[ts_ID]["right_boundary"] = rb
                TS_list[ts_ID]["left_boundary"] = lb

                TS_list[ts_ID]['CRI'] = cri_value

                # print(enc)

        return TS_list