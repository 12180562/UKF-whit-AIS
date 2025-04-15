import numpy as np

class UKF:
    def __init__(self):
        """
        상태: [x, y, speed, heading_deg]
        """
        # 초기 상태 추정치
        self.x = np.array([0.0, 0.0, 0.0, 0.0])  
        # 초기 공분산
        self.P = 100.0 * np.eye(4)  
        
        # 상태 차원
        self.n = 4  
        
        # 프로세스 노이즈 공분산 (4x4)
        self.Q = np.diag([0.01, 0.01, 0.01, 0.01])
        
        # AIS 측정 노이즈 공분산 (4x4): [x, y, speed, heading]
        self.R_AIS = np.diag([0.1, 0.1, 0.1, 0.1])
        
        # Radar 측정 노이즈 공분산 (2x2): [range, bearing_deg]
        self.R_Radar = np.diag([1.0, 1.0])
        
        # 시그마 포인트 파라미터 (alpha, beta 등 생략, kappa만 사용)
        self.kappa = 1.0

    # ---------------------------------------------------------
    # 1. 시그마 포인트 생성
    # ---------------------------------------------------------
    def sigma_points(self, x, P):
        """
        x: 상태 벡터 (n,)
        P: 공분산 (n,n)
        """
        n = self.n
        Xi = np.zeros((n, 2*n + 1))
        W = np.zeros(2*n + 1)
        
        # (n + kappa)*P에 대한 Cholesky 분해
        U = np.linalg.cholesky((n + self.kappa) * P)
        
        # 시그마 포인트
        Xi[:, 0] = x
        W[0] = self.kappa / (n + self.kappa)
        for i in range(n):
            Xi[:, i+1]       = x + U[:, i]
            Xi[:, n + i + 1] = x - U[:, i]
            W[i+1]           = 1.0 / (2.0*(n + self.kappa))
            W[n + i + 1]     = 1.0 / (2.0*(n + self.kappa))
        
        return Xi, W
    
    # ---------------------------------------------------------
    # 2. Unscented Transform
    # ---------------------------------------------------------
    def unscented_transform(self, Xi, W, noise_cov):
        """
        Xi: 시그마 포인트 (n, 2n+1) 또는 (m, 2n+1)
        W:  가중치 (2n+1,)
        noise_cov: 공분산에 더할 노이즈(Q 또는 R)
        
        return: (x_mean, x_cov)
        """
        n, kmax = Xi.shape
        
        # (1) 평균
        x_mean = np.zeros(n)
        for k in range(kmax):
            x_mean += W[k] * Xi[:, k]
        
        # (2) 공분산
        x_cov = np.zeros((n, n))
        for k in range(kmax):
            diff = (Xi[:, k] - x_mean).reshape(-1, 1)
            x_cov += W[k] * (diff @ diff.T)
        
        # (3) 노이즈 추가
        x_cov += noise_cov
        
        return x_mean, x_cov
    
    # ---------------------------------------------------------
    # 3. 프로세스 모델 f(x)
    #    상태: [ x, y, speed, heading_deg ]
    # ---------------------------------------------------------
    def fx(self, x, dt):
        # x[0]: pos_x
        # x[1]: pos_y
        # x[2]: speed
        # x[3]: heading (degrees)

        speed = x[2]
        heading = x[3]
        
        x_new = x[0] + speed * dt * np.cos(np.deg2rad(heading))
        y_new = x[1] + speed * dt * np.sin(np.deg2rad(heading))

        # 헤딩 값을 0~360도로 정상화
        # heading_new = (heading + 360) % 360

        return np.array([x_new, y_new, speed, heading])
    
    # ---------------------------------------------------------
    # 4. 관측 모델 (A) AIS -> 4차원
    # ---------------------------------------------------------
    def hx_AIS(self, x):
        """
        z = [ x, y, speed, heading ]
        상태를 그대로 반환
        """
        return x  # shape: (4,)

    # ---------------------------------------------------------
    # 4. 관측 모델 (B) Radar -> 2차원
    #    z = [ range, bearing_deg ]
    # ---------------------------------------------------------
    def hx_Radar(self, x, sensor_pos=np.array([0.0, 0.0])):
        """
        sensor_pos: 레이더(자선) 위치
        x = [x, y, speed, heading_deg]
        
        range = sqrt((x - sx)^2 + (y - sy)^2)
        bearing = atan2(y - sy, x - sx) (deg)
        """
        sx, sy = sensor_pos
        dx = x[0] - sx
        dy = x[1] - sy
        
        r = np.sqrt(dx*dx + dy*dy)
        b_rad = np.arctan2(dy, dx)  # -pi ~ pi
        b_deg = np.rad2deg(b_rad)   # -180~180
        
        return np.array([r, b_deg])

    # ---------------------------------------------------------
    # 5. 예측 단계
    # ---------------------------------------------------------
    def predict(self, dt):
        """
        시그마 포인트 -> 프로세스모델 -> Unscented Transform
        """
        # 시그마 포인트 생성
        Xi, W = self.sigma_points(self.x, self.P)
        
        # 각 시그마 포인트에 f(x, dt) 적용
        fXi = np.zeros_like(Xi)
        for i in range(Xi.shape[1]):
            fXi[:, i] = self.fx(Xi[:, i], dt)
        
        # 예측 상태, 예측 공분산
        xp, Pp = self.unscented_transform(fXi, W, self.Q)
        
        # 필터 내부 상태 갱신
        self.x = xp
        self.P = Pp
        
        return self.x, self.P

    # ---------------------------------------------------------
    # 6. Update (A) AIS -> 4차원
    # ---------------------------------------------------------
    def update_AIS(self, z):
        """
        z: [x_meas, y_meas, speed_meas, heading_meas]
        """
        # 시그마 포인트
        Xi, W = self.sigma_points(self.x, self.P)
        
        # 프로세스 모델(여기서는 dt=0 가정 or 이미 predict 후이므로)
        fXi = np.zeros_like(Xi)
        for i in range(Xi.shape[1]):
            # 이미 예측단계 후라고 생각하면 dt=0
            fXi[:, i] = self.fx(Xi[:, i], dt=0.0)
        
        # 예측 상태 xp, Pp
        xp, Pp = self.unscented_transform(fXi, W, self.Q)
        
        # hXi: 각 시그마 포인트를 AIS 관측 모델로
        hXi = np.zeros((4, Xi.shape[1]))
        for i in range(Xi.shape[1]):
            hXi[:, i] = self.hx_AIS(fXi[:, i])  # (4,)
        
        z_pred, Pz = self.unscented_transform(hXi, W, self.R_AIS)  # (4,) , (4x4)
        
        # 상태-관측 간 공분산
        Pxz = np.zeros((self.n, 4))
        for i in range(Xi.shape[1]):
            Pxz += W[i] * np.outer(fXi[:, i] - xp, hXi[:, i] - z_pred)
        
        # 칼만 이득
        K = Pxz @ np.linalg.inv(Pz)
        
        # 잔차
        z_diff = z - z_pred
        
        # 상태, 공분산 갱신
        self.x = xp + K @ z_diff
        self.P = Pp - K @ Pz @ K.T
        
        return self.x, self.P
    
    # ---------------------------------------------------------
    # 6. Update (B) Radar -> 2차원
    # ---------------------------------------------------------
    def update_Radar(self, z, sensor_pos=np.array([0.0, 0.0])):
        """
        z: [range_meas, bearing_meas(deg)]
        """
        # 시그마 포인트
        Xi, W = self.sigma_points(self.x, self.P)
        
        # 프로세스 모델 (dt=0 가정)
        fXi = np.zeros_like(Xi)
        for i in range(Xi.shape[1]):
            fXi[:, i] = self.fx(Xi[:, i], dt=0.0)
        
        xp, Pp = self.unscented_transform(fXi, W, self.Q)
        
        # Radar 관측 모델
        hXi = np.zeros((2, Xi.shape[1]))
        for i in range(Xi.shape[1]):
            hXi[:, i] = self.hx_Radar(fXi[:, i], sensor_pos)
        
        z_pred, Pz = self.unscented_transform(hXi, W, self.R_Radar)  # (2,), (2x2)
        
        # 상태-관측 간 공분산
        Pxz = np.zeros((self.n, 2))
        for i in range(Xi.shape[1]):
            Pxz += W[i] * np.outer(fXi[:, i] - xp, hXi[:, i] - z_pred)
        
        # 칼만 이득
        K = Pxz @ np.linalg.inv(Pz)
        
        # 잔차
        z_diff = z - z_pred
        
        # 갱신
        self.x = xp + K @ z_diff
        self.P = Pp - K @ Pz @ K.T
        
        return self.x, self.P