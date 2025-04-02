import sys, os
from math import *
import numpy as np
from numpy import deg2rad,rad2deg
# import rospy

class CRI:
    def __init__(self, L, B, Xo, Yo, Xt, Yt, Co, Ct, Vo, Vt, ship_scale, x_var = 0, y_var = 0):
        self.ship_scale = ship_scale
        self.L = L / self.ship_scale    #타선의 길이 [m] from pram
        self.B = B / self.ship_scale     #타선의 폭 [m]
        self.Xo = Xo    #자선 x좌표  [m]
        self.Yo = Yo    #자선 y좌표  [m] 
        self.Xt = Xt    #타선 x좌표  [m]
        self.Yt = Yt    #타선 y좌표  [m]
        self.Co = Co    #자선 Heading angle [rad]
        self.Ct = Ct    #타선 Heading angle [rad]
        self.Vo = Vo    #자선 속도   [knots]
        self.Vt = Vt    #타선 속도   [knots]
        self.ratio = 1852 / self.ship_scale #1852/110  #1 해리는 1852m
        # self.ratio = (12*self.L) / self.ship_scale #1852/110  #1 해리는 1852m
        self.x_sigma = 2*sqrt(x_var)
        self.y_sigma = 2*sqrt(y_var)

        # self.x_var = 0
        # self.y_var = 0
        # 평균에 대해 쁠마 1 표준편차 (시그마)는 68% 데이터 포함
        # 쁠마 2 시그마는 95%
        # 쁠마 3 시그마는 99.7% 포함

        self.mapped_radius = 0

    def RD(self):
        '''Relative Distance, 자선과 타선 사이의 상대 거리'''
        result = sqrt(((self.Xt - self.Xo) ** 2) + ((self.Yt - self.Yo) ** 2)) + 0.0001
        # print(round(result,2))
        return result

    def TB(self):
        '''True Bearing, 자선의 위치 기준 타선의 절대 방위, rad'''
        Xot = self.Xt - self.Xo
        Yot = self.Yt - self.Yo
        result = atan2(Yot, Xot) % (2*pi)
        return result

    def RB(self):
        '''Relative Bearing, 자선의 Heading angle에 대한 타선의 방위, rad'''
        if self.TB() - self.Co >= 0:
            result = self.TB() - self.Co
        else:
            result = self.TB() - self.Co + (2 * pi)
        return result
    
    def HAD(self):
        '''Heading angle difference, rad'''
        result = self.Ct - self.Co
        if result < 0 :
            result += 2*pi
        return result

    def Vox(self):
        '''자선 x방향 속도'''
        result = self.Vo * cos(self.Co)
        return result

    def Voy(self):
        '''자선 y방향 속도'''
        result = self.Vo * sin(self.Co)
        return result

    def Vtx(self):
        '''타선 x방향 속도'''
        result = self.Vt * cos(self.Ct)
        return result

    def Vty(self):
        '''타선 y방향 속도'''
        result = self.Vt * sin(self.Ct)
        return result

    def Vrx(self):
        result = self.Vtx() - self.Vox()
        return result

    def Vry(self):
        result = self.Vty() - self.Voy()
        return result

    def RV(self):
        '''Relative Velocity, 자선에 대한 타선의 상대속도'''
        result = sqrt(pow(self.Vrx(), 2) + pow(self.Vry(), 2)) + 0.001
        return result

    def RC(self):
        '''Relative speed heading direction, 상대속도(RV)의 방향'''
        result = atan2(self.Vry(), self.Vrx()) % (2*pi)
        return result

    def tcpa(self):
        # dx = self.Xt - self.Xo
        # dy = self.Yt - self.Yo
        # vrel_dot = self.Vrx() ** 2 + self.Vry() ** 2
        # if vrel_dot == 0:
        #     return None
        # result = -(dx * self.Vrx() + dy * self.Vry()) / vrel_dot
        v_r = self.RV()
        if v_r == 0:
            result = 0
        
        numerator = abs((self.Xo - self.Xt) * self.Vrx() + (self.Yo - self.Yt) * self.Vry())      
        result = numerator / (v_r ** 2)  

        return result

    def dcpa(self):
        # dx = self.Xt - self.Xo
        # dy = self.Yt - self.Yo
        # x_rel = dx + self.Vrx() * self.tcpa()
        # y_rel = dy + self.Vry() * self.tcpa()
        # result = sqrt(x_rel**2 + y_rel**2)
        v_r = self.RV()
        if v_r == 0:
            result = self.RD()
        numerator = abs((self.Xo - self.Xt) * self.Vrx() - (self.Yo - self.Yt) * self.Vry())
        result = numerator / v_r
        
        return result

    def d1(self):
        '''Safe approaching distance'''
        # RB = np.rad2deg(self.RB())
        # if 0 <= RB < 112.5:
        #     result = self.ratio * (1.1 - 0.2 * (self.RB()/pi))
        # elif 112.5 <= RB < 180:
        #     result = self.ratio * (1.0 - 0.4 * (self.RB()/pi))
        # elif 180 <= RB < 247.5:
        #     result = self.ratio * (1.0 - 0.4 * ((2 * pi - self.RB())/pi))
        # else:
        #     result = self.ratio * (1.1 - 0.2 * ((2 * pi - self.RB())/pi))
        result =  15 * self.L
        # print("d1 : ", result)
        return result

    def d2(self):
        '''Safe passing distance'''
        result = 2 * self.d1()
        return result

    def UDCPA(self):
        '''#d1, d2의 범위에 따른 DCPA의 계수'''
        if abs(self.dcpa()) <= self.d1():
            result = 1
        elif self.d2() < abs(self.dcpa()):
            result = 0
        else:
            result = 0.5 - 0.5 * sin((pi/(self.d2() - self.d1())) * (abs(self.dcpa()) - (self.d1() + self.d2())/2))
        return result

    def D1(self):
        '''Distance of action'''
        result = 12 * self.L
        return result

    def D2(self):
        '''Distance of last action'''
        # result = self.ratio * (1.7 * cos(self.RB() - np.deg2rad(19))) + sqrt(4.4 + 2.89 * pow(cos(self.RB() - np.deg2rad(19)), 2))
        result = 2 * self.D1()
        return result

    def UD(self):
        '''D1, D2의 범위에 따른 Relative distance의 계수'''
        if self.RD() <= self.D1():
            result = 1
        elif self.D2() < self.RD():
            result = 0
        else:
            result = pow((self.D2() - self.RD())/(self.D2() - self.D1()), 2)
        return result

    def t1(self):
        '''Collision time'''
        D1 = self.D1()
        if abs(self.dcpa()) <= D1:
            result = sqrt(pow(D1, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D1 - abs(self.dcpa())) / self.RV()
        return result

    def t2(self):
        '''Avoidance time'''
        # D2 = 12 * self.ratio  # 원래 이렇게 되어 있었음
        # D2 = 12 * 12  # 동훈 논문
        D2 = self.D2()  # 다른 논문
        if abs(self.dcpa()) <= D2:
            result = sqrt(pow(D2, 2) - pow(self.dcpa(), 2)) / self.RV()
        else:
            result = (D2 - abs(self.dcpa())) / self.RV()
        return result

    def UTCPA(self):
        '''t1, t2의 범위에 따른 TCPA의 계수'''
        if self.tcpa() < 0:
            result = 0
        else:
            if self.tcpa() <= self.t1():
                result = 1
            elif self.t2() < self.tcpa():
                result = 0
            else:
                result = pow(((self.t2() - abs(self.tcpa()))/(self.t2() - self.t1())), 2)
        return result

    def UB(self):
        '''Relative bearing에 대한 계수 UB'''
        result = 0.5 * (cos(self.RB() - np.deg2rad(19)) + sqrt((440/289) + pow(cos(self.RB() - np.deg2rad(19)), 2))) - (5/17)
        return result

    def K(self):
        '''Speed factor'''
        if self.Vt == 0 or self.Vo == 0:
            result = 0.001
        else:
            result = self.Vt / self.Vo
        return result

    def sinC(self):
        '''Collision angle, UK의 계산에 사용'''
        result = abs(sin(abs(self.Ct - self.Co)))
        return result

    def UK(self):
        '''Speed factor에 대한 계수 UK'''
        result = 1 / (1 + (2 / (self.K() * sqrt(pow(self.K(), 2) + 1 + (2 * self.K() * self.sinC())))))
        return result

    def CRI(self):
        '''충돌위험도지수, UDCPA, UTCPA, UD, UB, UK 5개의 파라미터에 가중치를 곱하여 계산'''
        result = 0.4 * self.UDCPA() + 0.367 * self.UTCPA() + 0.133 * self.UD() + 0.067 * self.UB() + 0.033 * self.UK() #원래 값
        # result = 0.4457 * self.UDCPA() + 0.2258 * self.UTCPA() + 0.1408 * self.UD() + 0.1321 * self.UB() + 0.0556 * self.UK() #원준 수정 값
        return round(result, 3)

    def encounter_classification(self):
        HAD = np.rad2deg(self.HAD())
        RB = np.rad2deg(self.RB())

        if 0 <= RB <= 22.5 or 337.5 <= RB <= 360:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"

        elif 22.5 < RB <= 90:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Safe"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"

        elif 90 < RB <= 112.5:
            if 67.5 <= HAD < 202.5:
                return "Safe"
            elif 202.5 < HAD <= 292.5:
                return "Starboard crossing"
            else:
                return "Overtaking"

        elif 247.5 <= RB < 270:
            if 157.5 <= HAD <= 292.5:
                return "Safe"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            else:
                return "Overtaking"

        elif 270 <= RB < 337.5:
            if 157.5 <= HAD <= 202.5:
                return "Head-on"
            elif 67.5 <= HAD < 157.5:
                return "Port crossing"
            elif 202.5 < HAD <= 292.5:
                return "Safe"
            else:
                return "Overtaking"

        else:
            if 0 <= HAD <= 67.5 or 292.5 <= HAD <= 360:
                if self.Vt > self.Vo:
                    return "Overtaking"
                else:
                    return "Safe"
            else:
                return "Safe"

    def CoE(self):
        '''Coefficients of encounter situations'''
        # a = self.encounter_classification()
        # print(a)
        if self.encounter_classification() == "Head-on":
            s = abs(2 - (self.Vo - self.Vt)/self.Vo)
            t = 0.2
        elif self.encounter_classification() == "Starboard crossing" or self.encounter_classification() == "Port crossing":
            s = 2 - self.HAD()/pi
            t = self.HAD()/pi
        elif self.encounter_classification() == "Overtaking":
            s = 1
            t = 0.2
        else:
            s = abs(1 + (self.Vo - self.Vt)/self.Vo)
            t = abs(0.5 + (self.Vo - self.Vt)/self.Vo)
        return s, t

    def ship_domain(self):
        if self.Vt <= 0.0: 
            self.Vt = 0.1

        KAD = pow(10, (0.3591 * log10(self.Vt) + 0.0952))  ## 논문에서 보면 지수함수를 사용
        KDT = pow(10, (0.5411 * log10(self.Vt) - 0.0795))  ## 논문에서 보면 지수함수를 사용
        AD = self.L * KAD
        DT = self.L * KDT

        s, t = self.CoE()

        # R_fore = self.L + (0.67 * (1 + s) * sqrt(pow(AD,2) + pow(DT/2,2)))
        # R_aft = self.L + (0.67 * sqrt(pow(AD,2) + pow(DT/2,2)))
        # R_stbd = self.B + DT * (1 + t)
        # R_port = self.B + (0.75 * DT * (1 + t))

        R_fore = (1 + 1.34 * sqrt(pow(KAD, 2) + pow(KDT / 2, 2))) * self.L
        R_aft = (1 + 0.67 * sqrt(pow(KAD, 2) + pow(KDT / 2, 2))) * self.L
        R_stbd = (0.2 + KDT) * self.L
        R_port = (0.2 + 0.75*KDT) * self.L

        return R_fore, R_aft, R_stbd, R_port

    def Rf(self):
        SD = self.ship_domain()
        result = SD[0]
        return result

    def Ra(self):
        SD = self.ship_domain()
        result = SD[1]
        return result

    def Rs(self):
        SD = self.ship_domain()
        result = SD[2]
        return result

    def Rp(self):
        SD = self.ship_domain()
        result = SD[3]
        return result
    
    def SD_dist_yoo(self):
        self.var_scale = 1  # 10 넣으면 우측으로 갈곳 없음
        Rf, Ra, Rs, Rp = self.Rf(), self.Ra(), self.Rs(), self.Rp()
        # print(Rf, Ra, Rs, Rp)
        tb = self.TB()

        # 도메인 스케일
        Rf_scaled = self.var_scale * Rf
        Ra_scaled = self.var_scale * Ra
        Rs_scaled = self.var_scale * Rs
        Rp_scaled = self.var_scale * Rp

        def ellipse_radius(a, b, theta):
            """
            중심이 (0,0)에 있고,
            x축 방향 반경이 a, y축 방향 반경이 b인 타원의
            극좌표식 r(θ)를 반환.
            (단, 타원의 주축이 x축과 y축에 평행하다고 가정)
            """
            a_scaled = a + self.y_sigma
            b_scaled = b + self.x_sigma
            return (a_scaled * b_scaled) / np.sqrt((b_scaled * np.cos(theta))**2 + (a_scaled * np.sin(theta))**2)
        # a는 장반경으로 x축 세로방향, b는 단반경을 y축 가로방향을 의미함
        # self.x_sigma는 시뮬레이션 기준 x축 세로방향
        # self.y_sigma는 시뮬레이션 기준 y축 가로방향
        # 점들을 저장할 리스트
        xy_points = []

        # 각 사분면별로 90개의 점을 만든다고 가정 (총 360점)
        num_points_per_quadrant = 90

        # 1사분면(0 ~ 90도) : (Rf, 0) -> (0, Rs) 여기서 양의 각도는 반시계 방향
        #   a=Rf, b=Rs, theta 범위 = [0, pi/2]
        thetas_1 = np.linspace(0, (np.pi/2), num_points_per_quadrant, endpoint=False)
        for t in thetas_1:
            r = ellipse_radius(Rf_scaled, Rp_scaled, t)
            x = r * np.cos(t)
            y = r * np.sin(t)
            xy_points.append([x, y])

        # 2사분면(90 ~ 180도) : (0, Rs) -> (-Ra, 0)
        #   a=Ra, b=Rs, theta 범위 = [pi/2, pi]
        thetas_2 = np.linspace(np.pi/2, np.pi, num_points_per_quadrant, endpoint=False)
        for t in thetas_2:
            r = ellipse_radius(Ra_scaled, Rp_scaled, t)
            x = r * np.cos(t)
            y = r * np.sin(t)
            xy_points.append([x, y])

        # 3사분면(180 ~ 270도) : (-Ra, 0) -> (0, -Rp)
        #   a=Ra, b=Rp, theta 범위 = [pi, 3pi/2]
        thetas_3 = np.linspace(np.pi, 3*np.pi/2, num_points_per_quadrant, endpoint=False)
        for t in thetas_3:
            r = ellipse_radius(Ra_scaled, Rs_scaled, t)
            x = r * np.cos(t)
            y = r * np.sin(t)
            xy_points.append([x, y])

        # 4사분면(270 ~ 360도) : (0, -Rp) -> (Rf, 0)
        #   a=Rf, b=Rp, theta 범위 = [3pi/2, 2pi]
        thetas_4 = np.linspace(3*np.pi/2, 2*np.pi, num_points_per_quadrant, endpoint=True)
        for t in thetas_4:
            r = ellipse_radius(Rf_scaled, Rs_scaled, t)
            x = r * np.cos(t)
            y = r * np.sin(t)
            xy_points.append([x, y])

        Ct = self.Ct
        angle = -Ct + np.pi/2
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle],
                                    [sin_angle,  cos_angle]])
        translation = np.array([self.Yt, self.Xt])

        rotated_points = []
        for (x, y) in xy_points:
            rotated = rotation_matrix.dot(np.array([x, y]))
            # 회전된 점에 평행이동을 적용합니다.
            translated = rotated + translation
            # print(translated)
            rotated_points.append(translated.tolist())

        angle_0 = tb % (2*np.pi)
        # 4) angle_0가 속하는 구간 확인
        if 0+self.Ct <= angle_0 < (np.pi/2)+self.Ct:
            # 1사분면 파라미터
            a, b = (Rf, Rs)
        elif (np.pi/2)+self.Ct <= angle_0 < np.pi+self.Ct:
            # 2사분면 파라미터
            a, b = (Ra, Rs)
        elif np.pi+self.Ct <= angle_0 < (3*np.pi/2)+self.Ct:
            # 3사분면 파라미터
            a, b = (Ra, Rp)
        else:
            # 4사분면 파라미터
            a, b = (Rf, Rp)

        # 5) 해당 구간의 (a, b)를 써서 r( tb_rad ) 계산
        #    (주의: 구간 판별은 angle_0 기준이지만,
        #           실제 ellipse_radius()는 '실제 θ'인 tb_rad 를 넣어주면 됨)
        self.mapped_radius = ellipse_radius(a, b, tb)
        # print("self.x_sigma : ",self.x_sigma)
        # print("self.y_sigma : ",self.y_sigma)
        # print("\n")
        # print("rotated_points : ",rotated_points)
        # print("\n")
        # print("mapped_radius : ",self.mapped_radius)
        return rotated_points, self.mapped_radius    
    
    def lb_rb(self):
        boundary_pts, mapped_radius = self.SD_dist_yoo()

        rel_bearings = []
        for (by, bx) in boundary_pts:
            dx = bx - self.Xo
            dy = by - self.Yo
            angle_rad = (atan2(dy, dx) + 2*pi) % (2*pi)
            rel_bearings.append(angle_rad)
        rel_bearings.sort()
        # rel_bearings.append(rel_bearings[0] + 2*pi)

        max_gap = 0
        pair_index = (0, 0)
        N = len(rel_bearings)

        for i in range(N):
            for j in range(i+1, N):
                diff = abs(rel_bearings[j] - rel_bearings[i])
                if diff > pi:
                    diff = 2*pi - diff
                if diff > max_gap:
                    max_gap = diff
                    pair_index = (i, j)

        angle_a = rel_bearings[pair_index[1]]
        angle_b = rel_bearings[pair_index[0]]

        d = (angle_a - angle_b + 2*pi) % (2*pi)

        bisector = (angle_b + d/2) % (2*pi)

        def relative_angle(angle, reference):
            diff = (angle - reference + 2*pi) % (2*pi)
            
            return diff

        rel1 = relative_angle(angle_a, bisector)
        rel2 = relative_angle(angle_b, bisector)
        
        if rel1 > 0 and rel2 < 0:
            left_bound_rad = angle_b
            right_bound_rad = angle_a

        elif rel1 < 0 and rel2 > 0:
            left_bound_rad = angle_a
            right_bound_rad = angle_b

        else:
            if abs(rel1) > abs(rel2): # starboard 에선 이건 > 가 맞다
                left_bound_rad = angle_b #if rel1 > 0 else angle_b
                right_bound_rad = angle_a #if rel1 > 0 else angle_a

            else:
                left_bound_rad = angle_a#-deg2rad(10) #if rel2 > 0 else angle_a
                right_bound_rad = angle_b#+ deg2rad(10)#if rel2 > 0 else angle_b

        # print("max_gap : ",max_gap)
        # print("mapped_radius : ",2*atan2(mapped_radius,rd))
        # print("left_bound_rad : ",rad2deg(left_bound_rad))
        # print("right_bound_rad : ",rad2deg(right_bound_rad))
        return left_bound_rad, right_bound_rad

    def SD_dist_lee(self):
        RB = np.rad2deg(self.RB())
        Rf, Ra, Rs, Rp = self.Rf(), self.Ra(), self.Rs(), self.Rp()
        if 0 <= RB < 90:
            result = sqrt(pow(Rf,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Rf,2)/pow(Rs,2))))
        elif 90 <= RB < 180:
            result = sqrt(pow(Ra,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Ra,2)/pow(Rs,2))))
        elif 180 <= RB < 270:
            result = sqrt(pow(Ra,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Ra,2)/pow(Rp,2))))
        else:
            result = sqrt(pow(Rf,2)/(pow(sin(RB),2) + pow(cos(RB),2) * (pow(Rf,2)/pow(Rp,2))))

        return result

    def SD_dist_hyo(self):
        Rf, Ra, Rs, Rp = self.Rf(), self.Ra(), self.Rs(), self.Rp()
        
        param = 4
        Xot = self.Xt-self.Xo
        Yot = self.Yt-self.Yo
        Rf_position = np.array([param*Rf*cos(deg2rad(self.Ct)),param*Rf*sin(deg2rad(self.Ct))]) + np.array([Xot,Yot])
        Ra_position = np.array([param*Ra*cos(deg2rad(self.Ct)+pi),param*Ra*sin(deg2rad(self.Ct)+pi)]) + np.array([Xot,Yot])
        Rs_position = np.array([param*Rs*cos(deg2rad(self.Ct)-pi/2),param*Rs*sin(deg2rad(self.Ct)-pi/2)]) + np.array([Xot,Yot])
        Rp_position = np.array([param*Rp*cos(deg2rad(self.Ct)+pi/2),param*Rp*sin(deg2rad(self.Ct)+pi/2)]) + np.array([Xot,Yot])

        Rf_rad = atan2(Rf_position[1],Rf_position[0])
        Ra_rad = atan2(Ra_position[1],Ra_position[0])
        Rs_rad = atan2(Rs_position[1],Rs_position[0])
        Rp_rad = atan2(Rp_position[1],Rp_position[0])

        R_rad_list = [Rf_rad,Ra_rad,Rs_rad,Rp_rad]
        new_rad_list = []

        max_angle = 0

        for j in R_rad_list:
            for k in R_rad_list:
                if abs(j - k) > pi:
                    angle = abs(abs(j-k) - 2*pi)
                else:
                    angle = abs(j-k)

                if angle > max_angle:
                    max_angle = angle
                    new_rad_list = [j,k]

        if abs(new_rad_list[0]-new_rad_list[1]) > pi:
            right_bound = max(new_rad_list)
            left_bound = min(new_rad_list)

        else:
            right_bound = min(new_rad_list)
            left_bound = max(new_rad_list)

        return right_bound, left_bound