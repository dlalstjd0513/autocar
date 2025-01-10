#!/usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import utm

# CSV 파일 경로
FIX_FRONT_PATH = "/home/woo/l2y_ws/src/sensor_fusion/config/gps_data.csv"
IMU_DATA_PATH = "/home/woo/l2y_ws/src/sensor_fusion/config/imu_data.csv"
WAYPOINT_PATH = "/home/woo/l2y_ws/src/sensor_fusion/config/waypoints.csv"  # waypoint.csv 경로

# ROS 노드 초기화s
rospy.init_node('navsatfix_publisher')

# NavSatFix 메시지 퍼블리셔
navsatfix_pub = rospy.Publisher('/path/fix', NavSatFix, queue_size=10)
waypoint_pub = rospy.Publisher('/waypoint', NavSatFix, queue_size=10)  # waypoint 퍼블리셔 추가
pose_pub = rospy.Publisher('/waypoint_pose', PoseStamped, queue_size=10)  # Pose 메시지 퍼블리셔 추가

def load_gps_data(file_path):
    """ GPS 데이터를 CSV 파일에서 로드하는 함수 """
    gps_data = []
    try:
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # 헤더 스킵
            for row in reader:
                latitude = float(row[6])  # 첫 번째 열: latitude
                longitude = float(row[7])  # 두 번째 열: longitude
                altitude = float(row[8])  # 세 번째 열: altitude
                gps_data.append((latitude, longitude, altitude))
                
    except Exception as e:
        rospy.logerr(f"GPS 데이터를 로드할 수 없습니다: {e}")
    return gps_data

def load_waypoint_data(file_path):
    """ Waypoint 데이터를 CSV 파일에서 로드하는 함수 """
    waypoint_data = []
    try:
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # 헤더 스킵
            for row in reader:
                # 각 행의 데이터를 Waypoint 형식으로 로드 (예: latitude, longitude, altitude, orientation_x, orientation_y, orientation_z, orientation_w)
                latitude = float(row[0])  # 첫 번째 열: latitude
                longitude = float(row[1])  # 두 번째 열: longitude
                altitude = float(row[2])  # 세 번째 열: altitude
                orientation_x = float(row[3])  # 네 번째 열: orientation_x
                orientation_y = float(row[4])  # 다섯 번째 열: orientation_y
                orientation_z = float(row[5])  # 여섯 번째 열: orientation_z
                orientation_w = float(row[6])  # 일곱 번째 열: orientation_w
                waypoint_data.append((latitude, longitude, altitude, orientation_x, orientation_y, orientation_z, orientation_w))
              
    except Exception as e:
        rospy.logerr(f"웨이포인트 데이터를 로드할 수 없습니다: {e}")
    return waypoint_data

def generate_navsatfix(gps_data, pub, is_waypoint=False):
    """ NavSatFix 메시지를 생성하고 퍼블리싱하는 함수 """
    for i in range(len(gps_data)):
        navsatfix = NavSatFix()
        navsatfix.header.stamp = rospy.Time.now()
        navsatfix.header.frame_id = "map"

        # GPS 데이터를 NavSatFix 메시지로 변환
        if is_waypoint:
            navsatfix.latitude = gps_data[i][0]  # waypoint 데이터의 latitude
            navsatfix.longitude = gps_data[i][1]  # waypoint 데이터의 longitude
            navsatfix.altitude = gps_data[i][2]  # waypoint 데이터의 altitude
        else:
            navsatfix.latitude = gps_data[i][0]  # 일반 GPS 데이터의 latitude
            navsatfix.longitude = gps_data[i][1]  # 일반 GPS 데이터의 longitude
            navsatfix.altitude = gps_data[i][2]  # 일반 GPS 데이터의 altitude
        
        # NavSatFix 메시지에 임의의 공분산 행렬 설정
        navsatfix.position_covariance = [0.1] * 9
        navsatfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        # NavSatFix 메시지 게시
        pub.publish(navsatfix)

def generate_pose(waypoint_data, pub):
    """ Pose 메시지를 생성하고 퍼블리싱하는 함수 """
    for i in range(len(waypoint_data)):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        # Waypoint 데이터를 Pose 메시지로 변환
        pose.pose.position.x = waypoint_data[i][0]
        pose.pose.position.y = waypoint_data[i][1]
        pose.pose.position.z = waypoint_data[i][2]
        
        # Orientation 데이터 할당
        pose.pose.orientation.x = waypoint_data[i][3]
        pose.pose.orientation.y = waypoint_data[i][4]
        pose.pose.orientation.z = waypoint_data[i][5]
        pose.pose.orientation.w = waypoint_data[i][6]

        # Pose 메시지 게시
        pub.publish(pose)


def publish_navsatfix():
    """ 주기적으로 NavSatFix 및 Pose 메시지를 퍼블리싱하는 함수 """
    # GPS 데이터 로드
    gps_data = load_gps_data(FIX_FRONT_PATH)
    waypoint_data = load_waypoint_data(WAYPOINT_PATH)  # waypoint.csv 데이터 로드
    
    if not gps_data:
        rospy.logerr("GPS 데이터가 비어 있습니다. CSV 파일을 확인하세요.")
        return
    
    if not waypoint_data:
        rospy.logerr("웨이포인트 데이터가 비어 있습니다. CSV 파일을 확인하세요.")
        return

    # NavSatFix 및 Pose 메시지 생성 및 게시
    rate = rospy.Rate(50)  # 1 Hz
    while not rospy.is_shutdown():
        # 기존 GPS 데이터 게시
        generate_navsatfix(gps_data, navsatfix_pub)  
        
        # 웨이포인트 데이터 게시 (NavSatFix와 Pose 모두 게시)
        generate_navsatfix(waypoint_data, waypoint_pub, is_waypoint=True)  # 웨이포인트 데이터 게시
        generate_pose(waypoint_data, pose_pub)  # Pose 메시지 게시
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_navsatfix()
    except rospy.ROSInterruptException:
        rospy.loginfo("NavSatFix 퍼블리싱 노드 종료")

