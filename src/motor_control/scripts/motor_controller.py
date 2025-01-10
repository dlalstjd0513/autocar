#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        self.is_on_slope = False
        self.pitch_deg = 0.0  # 피치 각도 저장 변수
        self.pitch_threshold = 20.0  # 경사로 판단 임계값 (도)
        
        self.rate = rospy.Rate(10)  # 10Hz
        self.run()
        
    def imu_callback(self, msg):
        # 쿼터니언 데이터를 오일러 각도로 변환하여 피치 각도 계산
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        self.pitch_deg = math.degrees(pitch)  # 피치 각도를 클래스 변수로 저장
        
        # 경사로 여부 판단
        if abs(self.pitch_deg) > self.pitch_threshold:
            self.is_on_slope = True
        else:
            self.is_on_slope = False
        
        # 디버깅용 출력
        rospy.loginfo("Pitch: {:.2f}, On Slope: {}".format(self.pitch_deg, self.is_on_slope))
        
    def run(self):
        while not rospy.is_shutdown():
            cmd_msg = Twist()
            desired_speed = 0.5  # 초기 선속도 설정
            desired_steering = 0.0  # 조향 각도 설정
            
            # 피치 각도의 부호에 따라 선속도 설정
            if self.is_on_slope:
                if self.pitch_deg > self.pitch_threshold:
                    # 오르막길: 전진
                    desired_speed = 0.5  # 양수 값
                elif self.pitch_deg < -self.pitch_threshold:
                    # 내리막길: 후진
                    desired_speed = -0.5  # 음수 값
                else:
                    # 평지: 정지
                    desired_speed = 0.0
            else:
                # 경사로가 아닌 경우 원하는 속도로 주행
                desired_speed = 0.5  # 정지 또는 원하는 속도로 설정

            cmd_msg.linear.x = desired_speed
            cmd_msg.angular.z = desired_steering

            # 브레이크 모드 설정
            if self.is_on_slope and desired_speed == 0.5:
                cmd_msg.linear.z = 1.0  # 브레이크 모드 활성화
            else:
                cmd_msg.linear.z = 0.0  # 브레이크 모드 비활성화

            # 디버깅용 출력
            rospy.loginfo("Desired Speed: {:.2f}, Brake: {}".format(desired_speed, cmd_msg.linear.z))

            self.cmd_pub.publish(cmd_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MotorController()
    except rospy.ROSInterruptException:
        pass

