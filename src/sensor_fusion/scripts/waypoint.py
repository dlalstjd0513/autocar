#!/usr/bin/env python3

import rospy
import csv
import os
import subprocess
from sensor_msgs.msg import NavSatFix, Imu
from pynput import keyboard

# 웨이포인트를 저장할 파일 경로
waypoint_file = "/home/woo/l2y_ws/src/sensor_fusion/config/waypoints.csv"
bag_file_dir = "/home/woo/l2y_ws/src/sensor_fusion/bagfiles"  # bag 파일 저장 경로

# 웨이포인트 리스트
waypoints = []
bag_process = None  # rosbag 프로세스를 저장할 변수
waypoint_index = 0  # 웨이포인트 인덱스

# 현재 GPS와 IMU 데이터를 저장할 변수
current_gps = None
current_imu = None

def save_waypoints():
    """
    웨이포인트를 CSV 파일로 저장하는 함수
    """
    global waypoints, waypoint_file
    # 웨이포인트가 비어 있지 않으면 저장
    if waypoints:
        with open(waypoint_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['latitude', 'longitude', 'altitude', 
                             'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])  # 헤더 추가
            writer.writerows(waypoints)
        rospy.loginfo(f"웨이포인트가 저장되었습니다: {waypoint_file}")
    else:
        rospy.loginfo("저장할 웨이포인트가 없습니다.")

def gps_callback(data):
    """
    GPS 데이터를 수신할 때마다 호출되는 콜백 함수
    """
    global current_gps
    # 현재 GPS 데이터를 업데이트
    current_gps = (data.latitude, data.longitude, data.altitude)

def imu_callback(data):
    """
    IMU 데이터를 수신할 때마다 호출되는 콜백 함수
    """
    global current_imu
    # 현재 IMU 데이터를 업데이트
    current_imu = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

def start_rosbag_record():
    """
    rosbag 기록을 시작하는 함수
    """
    global bag_process, waypoint_index
    # bag 파일 이름 설정
    bag_file_name = os.path.join(bag_file_dir, f"waypoint_{waypoint_index}.bag")
    rospy.loginfo(f"rosbag 기록을 시작합니다: {bag_file_name}")

    # rosbag record 명령 실행
    bag_process = subprocess.Popen(["rosbag", "record", "-O", bag_file_name, "/ublox/fix", "/imu/data"],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)

def stop_rosbag_record():
    """
    rosbag 기록을 중지하는 함수
    """
    global bag_process
    if bag_process is not None:
        bag_process.terminate()  # 프로세스 종료
        bag_process.wait()  # 종료 대기
        rospy.loginfo("rosbag 기록이 중지되었습니다.")
        bag_process = None

def on_press(key):
    global current_gps, current_imu, waypoints, waypoint_index
    try:
        if key == keyboard.Key.enter and current_gps and current_imu:
            # 현재 GPS 및 IMU 데이터를 웨이포인트로 추가
            waypoints.append(current_gps + current_imu)
            rospy.loginfo(f"웨이포인트 추가: {current_gps + current_imu}")

            # 이전 rosbag 기록을 중지
            stop_rosbag_record()

            # 새로운 rosbag 기록 시작
            waypoint_index += 1
            start_rosbag_record()
    except AttributeError:
        pass

def main():
    global current_gps, current_imu, waypoint_index
    current_gps = None
    current_imu = None

    # 웨이포인트 인덱스 초기화
    waypoint_index = 0

    # bag 파일 저장 디렉토리 생성
    if not os.path.exists(bag_file_dir):
        os.makedirs(bag_file_dir)

    # ROS 노드 초기화
    rospy.init_node('waypoint_recorder', anonymous=True)

    # GPS 데이터 수신
    rospy.Subscriber("/ublox/fix", NavSatFix, gps_callback)
    
    # IMU 데이터 수신
    rospy.Subscriber("/imu/data", Imu, imu_callback)

    # 키보드 리스너 설정
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    rospy.loginfo("웨이포인트 기록을 시작합니다. Enter 키를 눌러 웨이포인트를 추가하세요.")

    # 노드 종료 시 웨이포인트 및 rosbag 기록 중지
    rospy.on_shutdown(save_waypoints)
    rospy.on_shutdown(stop_rosbag_record)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

