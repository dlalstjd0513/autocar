#!/usr/bin/env python3

import rospy
import csv
import os
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion
import utm
import math

# CSV 파일 경로 설정
WAYPOINT_PATH = "/home/woo/l2y_ws/src/sensor_fusion/config/waypoints.csv"

# 제어를 위한 고정 속도 상수
fixed_velocity = 0.1  # 고정된 속도
k_w = 0.01  # 회전 제어 상수
goal_reach_threshold = 10  # 목표 지점 도달 임계값
L = 0.5  # 차량 휠베이스

# ROS 퍼블리셔 및 서브스크라이버 설정
pub_cmd = None  # 차량 제어 명령어 퍼블리셔
current_position = None
current_orientation = None
goal_position = None  # 목표 위치 변수

# 웨이포인트 리스트
waypoints = []

# PurePursuit 클래스 정의
class PurePursuit:
    def __init__(self):
        self.last_waypoint_index = 0  # 가장 최근에 통과한 waypoint의 인덱스

    # 두 점 사이의 거리 계산
    def get_dist(self, point1, point2):
        return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    # Lookahead Point를 경로 상에서 찾음
    def get_lookahead_point(self, curr_pos, lookahead_distance, waypoints):
        min_dist = float('inf')
        close_index = self.last_waypoint_index  # 가장 최근에 통과한 waypoint 이후부터 탐색

        # 현재 위치와 가장 가까운 경로 상의 다음 점을 찾음
        for i in range(self.last_waypoint_index, len(waypoints)):
            dist = self.get_dist(waypoints[i][:2], curr_pos)
            if dist < min_dist:
                min_dist = dist
                close_index = i

        # 경로의 범위를 초과하지 않도록 처리
        lookahead_index = min(close_index + lookahead_distance, len(waypoints) - 1)
        lookahead_point = waypoints[lookahead_index]
        return lookahead_point

    # 조향각 계산 (웨이포인트의 예상 방향과 실시간 IMU 데이터 비교)
    def get_steering_angle(self, curr_pos, lookahead_point, current_heading, expected_heading):
        Ld = self.get_dist(lookahead_point[:2], curr_pos)  # Lookahead 거리
        alpha = np.arctan2((lookahead_point[1] - curr_pos[1]), (lookahead_point[0] - curr_pos[0])) - current_heading  # 목표점까지의 각도 차이

        # 예상된 헤딩과 현재 헤딩의 차이를 보정하여 조향각 계산
        heading_diff = expected_heading - current_heading
        corrected_alpha = alpha + k_w * heading_diff
        steering_angle = np.arctan2(2 * L * np.sin(corrected_alpha), Ld)  # 조향각 계산
        return steering_angle

    # 경로 이탈 여부 확인
    def is_off_path(self, curr_pos, max_deviation, waypoints):
        min_dist = float('inf')

        # 현재 위치와 가장 가까운 경로 상의 점을 찾음
        for i in range(self.last_waypoint_index, len(waypoints)):
            dist = self.get_dist(waypoints[i][:2], curr_pos)
            if dist < min_dist:
                min_dist = dist

        # 경로에서 너무 멀어졌다면 경로 이탈로 간주
        return min_dist > max_deviation

    # 지나간 waypoint를 제외하고, 다음 waypoint 중 가장 가까운 지점을 찾음
    def find_next_closest_point(self, curr_pos, waypoints):
        min_dist = float('inf')
        next_index = self.last_waypoint_index + 1  # 지나간 waypoint를 제외하고 검색 시작

        # 현재 위치와 가장 가까운 다음 경로 상의 점을 찾음
        for i in range(next_index, len(waypoints)):
            dist = self.get_dist(waypoints[i][:2], curr_pos)
            if dist < min_dist:
                min_dist = dist
                next_index = i

        # 인덱스가 waypoints의 길이를 초과하지 않도록 처리
        if next_index >= len(waypoints):
            next_index = len(waypoints) - 1  # 마지막 웨이포인트로 설정

        return waypoints[next_index], next_index

def load_waypoints(file_path):
    """ 웨이포인트 데이터를 CSV 파일에서 로드하는 함수 """
    waypoints = []
    try:
        with open(file_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile, delimiter=',')
            for row in reader:
                latitude = float(row['latitude'])
                longitude = float(row['longitude'])
                altitude = float(row['altitude'])
                orientation_x = float(row['orientation_x'])
                orientation_y = float(row['orientation_y'])
                orientation_z = float(row['orientation_z'])
                orientation_w = float(row['orientation_w'])
                
                # GPS 좌표를 UTM 좌표로 변환
                utm_coords = utm.from_latlon(latitude, longitude)
                
                # 쿼터니언을 유러내이션으로 변환
                orientation_quat = [orientation_x, orientation_y, orientation_z, orientation_w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_quat)
                
                waypoints.append((utm_coords[0], utm_coords[1], altitude, yaw))
                
        rospy.loginfo(f"{len(waypoints)}개의 웨이포인트 로드 완료")
    except Exception as e:
        rospy.logerr(f"웨이포인트를 로드할 수 없습니다: {e}")
    return waypoints

def gps_callback(msg):
    """ GPS 데이터를 수신하고 현재 위치를 업데이트하는 콜백 함수 """
    global current_position
    # GPS 좌표를 UTM 좌표로 변환하여 저장
    utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
    current_position = (utm_coords[0], utm_coords[1], msg.altitude)
    rospy.loginfo(f"GPS 수신: {current_position}")

def imu_callback(msg):
    """ IMU 데이터를 수신하고 현재 방향을 업데이트하는 콜백 함수 """
    global current_orientation
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_orientation = yaw
    rospy.loginfo(f"IMU 수신: Yaw = {current_orientation}")

def stop_vehicle():
    """ 프로그램 종료 시 차량을 멈추는 함수 """
    global pub_cmd
    rospy.loginfo("프로그램 종료, 차량을 멈춥니다.")
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub_cmd.publish(twist)

def control_vehicle(goal_position, current_orientation, waypoints, pp):
    """ 목표 지점과 방향으로 차량을 이동시키기 위한 제어 함수 """
    global pub_cmd, current_position

    # 차량이 주행할 준비가 되었는지 확인
    if current_position is None or current_orientation is None or goal_position is None:
        rospy.logwarn("현재 위치 또는 목표 데이터가 없습니다.")
        return
    
    # 위치와 목표 지점 확인
    rospy.loginfo(f"Current Position: {current_position}")
    rospy.loginfo(f"Goal Position: {goal_position}")
    rospy.loginfo(f"Current Orientation: {current_orientation}")

    # Pure Pursuit 알고리즘으로 차량 제어
    if pp.is_off_path(current_position, 20.0, waypoints):  # 경로 이탈 범위
        rospy.logwarn("경로 이탈, 복구 중...")
        closest_point, closest_index = pp.find_next_closest_point(current_position, waypoints)
        pp.last_waypoint_index = closest_index  # 복귀한 웨이포인트를 시작점으로 설정
        goal_position = closest_point
        rospy.loginfo(f"복구 중... 목표 위치: {goal_position}")

    # 마지막 웨이포인트에 도달했는지 확인 후 다시 순환
    if pp.last_waypoint_index >= len(waypoints):
        rospy.loginfo("마지막 웨이포인트에 도달했습니다. 첫 번째 웨이포인트로 돌아갑니다.")
        pp.last_waypoint_index = 0  # 다시 첫 번째 웨이포인트로 설정
        goal_position = waypoints[pp.last_waypoint_index]

    # 목표점 순차적으로 업데이트
    pp.last_waypoint_index += 1
    if pp.last_waypoint_index < len(waypoints):
        goal_position = waypoints[pp.last_waypoint_index]
    else:
        # 웨이포인트의 범위를 초과하지 않도록 설정
        pp.last_waypoint_index = 0
        goal_position = waypoints[pp.last_waypoint_index]

    # Lookahead Point 계산 및 조향각 계산
    lookahead_point = pp.get_lookahead_point(current_position, 3, waypoints)
    expected_heading = lookahead_point[3]  # 목표 웨이포인트의 예상 헤딩 (yaw)
    steering_angle = pp.get_steering_angle(current_position, lookahead_point, current_orientation, expected_heading)
    
    # 조향각 제한
    max_steering_angle = 0.5  # 최대 조향각
    min_steering_angle = -0.5  # 최소 조향각
    steering_angle = max(min(steering_angle, max_steering_angle), min_steering_angle)
    
    # 속도는 고정된 값으로 설정
    linear_velocity = fixed_velocity
    angular_velocity = steering_angle

    # Twist 메시지 생성 및 퍼블리시 확인
    rospy.loginfo(f"Twist 생성: linear.x = {linear_velocity}, angular.z = {angular_velocity}")
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity
    pub_cmd.publish(twist)
    rospy.loginfo("Twist 메시지 퍼블리시됨")

def main():
    global pub_cmd, waypoints, goal_position

    rospy.init_node('waypoint_driver', anonymous=True)

    # 노드가 종료될 때 차량을 멈추는 함수를 등록
    rospy.on_shutdown(stop_vehicle)

    # 웨이포인트 데이터 로드 (초기 로드)
    waypoints = load_waypoints(WAYPOINT_PATH)

    # 웨이포인트가 비어있는지 확인
    if len(waypoints) == 0:
        rospy.logerr("웨이포인트가 없습니다. 종료합니다.")
        return

    # goal_position을 첫 번째 웨이포인트로 초기화
    goal_position = waypoints[0]

    # 차량 제어 퍼블리셔
    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # GPS, IMU 및 Waypoint 데이터 구독
    rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    rate = rospy.Rate(30)  # 30 Hz
    pp = PurePursuit()

    # GPS와 IMU 데이터가 수신될 때까지 대기
    rospy.loginfo("GPS 및 IMU 데이터 수신 대기 중...")
    while not rospy.is_shutdown() and (current_position is None or current_orientation is None):
        rospy.sleep(0.1)  # 0.1초 대기
    rospy.loginfo("GPS 및 IMU 데이터 수신 완료. 주행 시작.")

    # 주행 시작
    while not rospy.is_shutdown():
        # 수신된 웨이포인트로 차량을 제어
        if goal_position:
            control_vehicle(goal_position, current_orientation, waypoints, pp)
        else:
            rospy.logwarn("goal_position 값이 None 입니다. 제어 함수 호출되지 않음.")
        
        rate.sleep()

    rospy.loginfo("노드 종료")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


