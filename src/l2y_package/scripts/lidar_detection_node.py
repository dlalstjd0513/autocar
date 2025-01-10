#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math  # math 모듈을 사용하여 isnan() 함수 호출

# 객체가 감지되는 최소 거리 임계값 (예: 1m)
DETECTION_THRESHOLD = 1.0  # 1미터 이내로 물체가 있을 경우 감지

def lidar_callback(data):
    ranges = data.ranges
    

    # 유효한 거리 데이터만 추출 (range_min과 range_max 사이의 값, 0 또는 NaN 필터링)
    valid_ranges = [r for r in ranges if data.range_min < r < data.range_max and r != 0 and not math.isnan(r)]

    if len(valid_ranges) > 0:
        # 유효 범위 내에서 가장 가까운 거리 계산
        min_distance = min(valid_ranges)

        # 최소 거리가 임계값 이하일 때 객체 감지
        if min_distance < DETECTION_THRESHOLD:
            rospy.loginfo(f"객체 감지! 거리: {min_distance:.2f}m")
        else:
            rospy.loginfo(f"최소 거리: {min_distance:.2f}m")
    else:
        rospy.loginfo("유효한 거리 데이터가 없습니다.")

def lidar_detection_node():
    # ROS 노드 초기화
    rospy.init_node('lidar_detection_node', anonymous=True)

    # LiDAR 데이터 구독 (토픽: /scan)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    # 노드가 종료되지 않도록 유지
    rospy.spin()

if __name__ == '__main__':
    try:
        lidar_detection_node()
    except rospy.ROSInterruptException:
        pass

