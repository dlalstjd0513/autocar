#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist  # 로봇 제어 메시지
import math

# 객체가 감지되는 최소 거리 임계값 (예: 1m)
DETECTION_THRESHOLD = 1.0  # 1미터 이내로 물체가 있을 경우 감지

# 로봇 제어를 위한 퍼블리셔
cmd_pub = None

def lidar_callback(data):
    global cmd_pub

    # LiDAR에서 수신한 거리 데이터
    ranges = data.ranges

    # 유효한 거리 데이터만 추출 (range_min과 range_max 사이의 값, 0 또는 NaN 필터링)
    valid_ranges = [r for r in ranges if data.range_min < r < data.range_max and r != 0 and not math.isnan(r)]

    if len(valid_ranges) > 0:
        # 유효 범위 내에서 가장 가까운 거리 계산
        min_distance = min(valid_ranges)

        # Twist 메시지 생성
        twist = Twist()

        # 최소 거리가 임계값 이하일 때 객체 감지 -> 정지
        if min_distance < DETECTION_THRESHOLD:
            rospy.loginfo(f"객체 감지! 거리: {min_distance:.2f}m. 정지.")
            twist.linear.x = 0.0  # 정지
            twist.angular.z = 0.0  # 회전 없음
        else:
            rospy.loginfo(f"최소 거리: {min_distance:.2f}m. 직진.")
            twist.linear.x = 0.1  # 직진 속도 (양수면 전진)
            twist.angular.z = 0.0  # 회전 없음
        
        # 명령 퍼블리시
        cmd_pub.publish(twist)
    else:
        rospy.loginfo("유효한 거리 데이터가 없습니다.")

        # 유효한 데이터가 없을 때도 정지 (안전)
        twist = Twist()
        twist.linear.x = 0.1  # 정지
        twist.angular.z = 0.0  # 회전 없음
        cmd_pub.publish(twist)

def lidar_detection_node():
    global cmd_pub

    # ROS 노드 초기화
    rospy.init_node('lidar_detection_node', anonymous=True)

    # LiDAR 데이터 구독 (토픽: /scan)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    # cmd_vel 퍼블리셔 생성 (로봇 속도 명령을 퍼블리시할 토픽)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 노드가 종료되지 않도록 유지
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_detection_node()
    except rospy.ROSInterruptException:
        pass

