#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, NavSatFix

class SensorFusion:
    def __init__(self):
        # IMU와 GPS 데이터를 수신하기 위한 서브스크라이버
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)

        # 융합된 데이터를 퍼블리시할 퍼블리셔
        self.fusion_pub = rospy.Publisher('/sensor_fusion/fix', NavSatFix, queue_size=10)

        # 초기 데이터
        self.latest_imu = None
        self.latest_gps = None

    def imu_callback(self, data):
        self.latest_imu = data
        self.publish_fusion()

    def gps_callback(self, data):
        self.latest_gps = data
        self.publish_fusion()

    def publish_fusion(self):
        if self.latest_imu and self.latest_gps:
            fused_fix = NavSatFix()
            fused_fix.header.stamp = rospy.Time.now()
            fused_fix.header.frame_id = 'map'

            # GPS 위치 정보 복사 (예시)
            fused_fix.latitude = self.latest_gps.latitude
            fused_fix.longitude = self.latest_gps.longitude
            fused_fix.altitude = self.latest_gps.altitude
            fused_fix.status = self.latest_gps.status
            fused_fix.position_covariance = [0] * 9  # 공분산 행렬 설정
            fused_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.fusion_pub.publish(fused_fix)

if __name__ == '__main__':
    rospy.init_node('sensor_fusion_node')
    sensor_fusion = SensorFusion()
    rospy.spin()

