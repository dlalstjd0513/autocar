#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
from pyproj import Proj, transform
from geometry_msgs.msg import Twist
from pynput import keyboard
from tf.transformations import euler_from_quaternion

# 경로 CSV 파일 경로 (GPS 및 IMU 데이터 포함)
gps_csv_file = "/home/woo/l2y_ws/src/sensor_fusion/config/fix_front.csv"  # GPS 데이터 파일 경로
imu_csv_file = "/home/woo/l2y_ws/src/sensor_fusion/config/imu_data.csv"   # IMU 데이터 파일 경로

# UTM 변환을 위한 설정 (지역에 맞게 UTM Zone을 설정하세요)
proj_wgs84 = Proj(init='epsg:4326')  # WGS84 좌표계
proj_utm = Proj(init='epsg:32652')   # UTM Zone 52 좌표계 (지역에 맞게 변경 필요)

# 제어를 위한 PID 제어기 상수
k_v = 0.3  # 속도 제어 상수
k_w = 1.0  # 회전 제어 상수

# ROS 퍼블리셔 설정
pub = None
stop_program = False  # 프로그램 중지 플래그

def load_gps_imu_data():
    """
    GPS 및 IMU 데이터 파일을 읽고 UTM 좌표로 변환하여 병합합니다.
    """
    # GPS 데이터 읽기 및 UTM 변환
    gps_data = pd.read_csv(gps_csv_file)
    gps_data['utm_x'], gps_data['utm_y'] = transform(proj_wgs84, proj_utm, gps_data['field.longitude'].values, gps_data['field.latitude'].values)

    # IMU 데이터 읽기 및 yaw 계산
    imu_data = pd.read_csv(imu_csv_file)
    imu_data['yaw'] = imu_data.apply(lambda row: euler_from_quaternion(
        [row['field.orientation.x'], row['field.orientation.y'], row['field.orientation.z'], row['field.orientation.w']])[2], axis=1)

    # GPS와 IMU 데이터를 시간 기준으로 동기화하여 병합
    merged_data = pd.merge_asof(gps_data.sort_values('%time'), imu_data.sort_values('%time'), on='%time', direction='nearest')

    return merged_data

def control_vehicle(current_x, current_y, goal_x, goal_y, current_yaw):
    """
    현재 위치와 목표 위치를 바탕으로 모터 제어 명령을 생성하여 퍼블리시합니다.
    """
    # 목표 지점까지의 거리와 각도 계산
    distance = np.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
    angle_to_goal = np.arctan2(goal_y - current_y, goal_x - current_x)

    # 속도와 회전 속도 계산
    linear_velocity = k_v * distance  # 목표 지점까지의 거리 비례 속도
    angular_velocity = k_w * (angle_to_goal - current_yaw)  # 목표 각도와 현재 각도의 차이 비례 회전 속도

    # ROS Twist 메시지를 사용하여 모터 제어 명령 퍼블리시
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity
    pub.publish(twist)

def on_press(key):
    global stop_program
    if key == keyboard.Key.esc:
        rospy.loginfo("ESC 키가 눌렸습니다. 프로그램을 종료합니다.")
        stop_program = True
        rospy.signal_shutdown("ESC 키를 눌러 프로그램 종료")  # ROS 노드 종료 신호

def main():
    global pub, stop_program

    rospy.init_node('autonomous_driving_controller', anonymous=True)
    
    # ROS 퍼블리셔 초기화
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # GPS 및 IMU 데이터 로드 및 처리
    data = load_gps_imu_data()

    rospy.loginfo("자율주행 경로 시작.")

    # 키보드 리스너 설정
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # 경로의 각 지점에서 모터 명령 생성 및 퍼블리시
    for index, row in data.iterrows():
        if stop_program or rospy.is_shutdown():  # 종료 신호를 받으면 중단
            rospy.loginfo("종료 신호를 받아 노드를 중지합니다.")
            break

        current_x = row['utm_x']
        current_y = row['utm_y']
        current_yaw = row['yaw']

        # 다음 위치를 목표 지점으로 설정
        if index < len(data) - 1:
            goal_x = data['utm_x'].iloc[index + 1]
            goal_y = data['utm_y'].iloc[index + 1]
        else:
            # 마지막 점에 도달하면 종료
            goal_x = data['utm_x'].iloc[0]
            goal_y = data['utm_y'].iloc[0]

        rospy.loginfo(f"현재 위치: UTM X: {current_x}, UTM Y: {current_y}, Yaw: {current_yaw} / 목표 지점: UTM X: {goal_x}, UTM Y: {goal_y}")

        # 제어 명령 생성 및 퍼블리시
        control_vehicle(current_x, current_y, goal_x, goal_y, current_yaw)

        # 각 위치에서 일정 시간 대기 (2초)
        rospy.sleep(2.0)

        # 목표 지점에 도달하면 로그 출력
        if np.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2) < 0.5:  # 목표 지점 반경 0.5m 이내
            rospy.loginfo(f"목표 지점 {index + 1}/{len(data)} 도달!")

    listener.stop()  # 키보드 리스너 종료
    rospy.loginfo("경로 재생 완료.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

