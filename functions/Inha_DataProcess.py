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
        sigma_points = MerweScaledSigmaPoints(n=4, alpha=.1, beta=2., kappa=0.1)
        self.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=4, dt=self.dt, fx=self.state_transition, hx=self.measurement_function, points=sigma_points)
        self.ukf.x = np.array([0., 0., 0., 0.])  # 초기 상태 추정치
        self.ukf.P *= 1.  # 초기 공분산 행렬
        self.ukf.R = np.eye(4) * 0.5  # 측정 노이즈
        self.ukf.Q = np.eye(4) * 0.5  # 프로세스 노이즈

        self.last_measurement = None
        self.last_heading = None
        self.ukf_initialized = False

    def state_transition(self, x, dt):
        angle_dt = rospy.get_param('ukf_angle_dt')  # 각속도 조정 파라미터
        new_x = x.copy()
        if self.last_heading is not None:
            heading_change = x[3] - self.last_heading
            angular_velocity = heading_change / angle_dt
            new_x[3] += angular_velocity
        else:
            new_x[3] = x[3]

        new_x[0] += dt * (((x[2]*1852) * np.cos(np.deg2rad(new_x[3])))/(60*60))
        new_x[1] += dt * (((x[2]*1852) * np.sin(np.deg2rad(new_x[3])))/(60*60))
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
            predict=self.ukf.predict()
            self.ukf.update(predict)
            # rospy.loginfo("Measurement unchanged, prediction only: %s", self.ukf.x)
            self.predicted_values.append(self.ukf.x)
        else:
            # 측정값이 변경되었다면 예측 및 업데이트 수행
            if self.last_measurement is not None:
                # 각도 변화량 계산 및 업데이트 로직 적용
                self.last_heading = self.last_measurement[3]  # 이전 헤딩 값을 업데이트
            self.last_measurement = measurement
            self.ukf.predict()
            self.ukf.update(measurement)
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
        ):

        self.ship_ID = ship_ID
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Vel_U = Vel_U
        self.Heading = Heading
        self.waypoint_dict = waypoint_dict

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
            self.ship_dic[index_ship] = UKF()
            predicted_state = self.ship_dic[index_ship].update_ukf(self.Pos_X[i], self.Pos_Y[i], self.Vel_U[i], self.Heading[i])
            if index_ship == OS_ID:
                self.ship_dic[OS_ID]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Ori_X' : self.Pos_X[i],
                    'Ori_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    'Pos_X' : predicted_state[0],
                    'Pos_Y' : predicted_state[1],
                    }

            else:
                self.ship_dic[index_ship]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Ori_X' : self.Pos_X[i],
                    'Ori_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    'Pos_X' : predicted_state[0],
                    'Pos_Y' : predicted_state[1],
                    }

        # print(self.ship_dic)
        # print(self.ship_ID)
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