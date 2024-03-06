#!/usr/bin/env python

import rospy
import csv
import datetime
import numpy as np
from std_msgs.msg import String
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

class ShipUKF:
    def __init__(self):
        self.dt = 20  # 샘플링 시간
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
        angle_dt = 1  # 각속도 조정 파라미터
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

    def update_ukf(self, measurement):
        if not self.ukf_initialized:
            self.ukf.x = measurement  # 초기 상태 추정치 설정
            self.ukf_initialized = True  # UKF가 초기화되었음을 표시
            rospy.loginfo("UKF initialized with first measurement: %s", self.ukf.x)
        # 측정값이 변경되었는지 확인
        if self.last_measurement is not None and np.array_equal(measurement, self.last_measurement):
            self.last_heading = self.ukf.x[3]
            # 측정값이 변경되지 않았다면 predict만 수행
            predict=self.ukf.predict()
            self.ukf.update(predict)
            rospy.loginfo("Measurement unchanged, prediction only: %s", self.ukf.x)
            self.predicted_values.append(self.ukf.x)
        else:
            # 측정값이 변경되었다면 예측 및 업데이트 수행
            if self.last_measurement is not None:
                # 각도 변화량 계산 및 업데이트 로직 적용
                self.last_heading = self.last_measurement[3]  # 이전 헤딩 값을 업데이트
            self.last_measurement = measurement
            self.ukf.predict()
            self.ukf.update(measurement)
            rospy.loginfo("Measurement updated, state updated: %s", self.ukf.x)
            self.predicted_values = []
        return self.ukf.x

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
            self.ships[ship_id] = ShipUKF()

        predicted_state = self.ships[ship_id].update_ukf(current_measurement)

        # 현재 시간, 선박 ID, 위도, 경도 추출 및 CSV 파일에 기록
        time = rospy.get_time()
        self.csv_writer.writerow([time, ship_id, predicted_state[0], predicted_state[1]])
        self.csv_file.flush()

    def on_shutdown(self):
        self.csv_file.close()

if __name__ == '__main__':
    try:
        ukf_node = UKFNode()
        rospy.on_shutdown(ukf_node.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
