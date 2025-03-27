import math
import matplotlib.pyplot as plt
import numpy as np

def demo_smaller_arc_midpoint(left_deg, right_deg):
    """
    left_deg, right_deg: 각도를 '도(deg)'로 전달해 주세요.
    이 함수는 두 각도 사이의 작은 호를 찾아 그 중앙(LOS)을 계산한 뒤,
    원 위에 시각적으로 표시해 줍니다.
    """
    # (1) 각도를 라디안으로 변환 후 0 ~ 2π 범위로 보정
    left_rad = math.radians(left_deg)  % (2*math.pi)
    right_rad = math.radians(right_deg) % (2*math.pi)

    # (2) 두 각 사이의 간격(diff)을 구함
    diff = (right_rad - left_rad) % (2*math.pi)

    # (3) 작은 호의 중간각(LOS) 구하기
    if diff <= math.pi:
        # left->right 자체가 작은 호
        half_diff = diff / 2
        LOS_rad = (left_rad + half_diff) % (2*math.pi)
    else:
        # (right-left)가 큰 호이므로, 실제 작은 호는 (2π - diff)
        half_diff = (2*math.pi - diff) / 2
        LOS_rad = (left_rad - half_diff) % (2*math.pi)

    # (4) 원을 그릴 데이터
    t = np.linspace(0, 2*math.pi, 300)
    x_circle = np.cos(t)
    y_circle = np.sin(t)

    # (5) 각 선분(원점→각도)에 해당하는 x, y 좌표
    x_left = [0, math.cos(left_rad)]
    y_left = [0, math.sin(left_rad)]
    x_right = [0, math.cos(right_rad)]
    y_right = [0, math.sin(right_rad)]
    x_los = [0, math.cos(LOS_rad)]
    y_los = [0, math.sin(LOS_rad)]

    # (6) 그래프 그리기
    plt.figure()  # 하나의 독립된 그림
    plt.plot(x_circle, y_circle)  # 단위 원
    plt.plot(x_left,  y_left)     # Left 각도
    plt.plot(x_right, y_right)    # Right 각도
    plt.plot(x_los,   y_los, linestyle='--')  # LOS(점선)

    # (7) 텍스트로 표시
    #     각 선분 끝 지점 근처에 angle 표기
    plt.text(math.cos(left_rad)*1.05,
             math.sin(left_rad)*1.05,
             f"Left: {left_deg}°")
    plt.text(math.cos(right_rad)*1.05,
             math.sin(right_rad)*1.05,
             f"Right: {right_deg}°")
    # LOS는 선분 중간쯤에 라벨
    plt.text(math.cos(LOS_rad)*0.5,
             math.sin(LOS_rad)*0.5,
             "LOS (mid)")

    plt.gca().set_aspect('equal', adjustable='box')  # 원이 찌그러지지 않도록
    plt.title("Smaller Arc and Its Midpoint (LOS)")
    plt.show()

# 예시 실행: left=350°, right=10°
demo_smaller_arc_midpoint(350, 10)
