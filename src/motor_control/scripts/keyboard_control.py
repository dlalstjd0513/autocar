#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

# ROS 노드 초기화
rospy.init_node('keyboard_control')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Twist 메시지 생성
twist = Twist()

# 속도 조절 변수 (속도를 줄이기 위해 값을 작게 설정)
linear_speed = 0.3 # 전진/후진 속도
angular_speed = 0.5  # 회전 속도

def on_press(key):
    try:
        if key.char == 'w':  # Forward (전진)
            twist.linear.x = linear_speed  # 전진 속도 설정
            twist.angular.z = 0.0
            print("Forward")

        elif key.char == 's':  # Backward (후진)
            twist.linear.x = -linear_speed  # 후진 속도 설정
            twist.angular.z = 0.0
            print("Backward")

        elif key.char == 'a':  # Steer Left (좌회전)
            twist.angular.z = angular_speed  # 좌회전 설정
            twist.linear.x = 0.0
            print("Steer Left")

        elif key.char == 'd':  # Steer Right (우회전)
            twist.angular.z = -angular_speed  # 우회전 설정
            twist.linear.x = 0.0
            print("Steer Right")

        pub.publish(twist)

    except AttributeError:
        pass

def on_release(key):
    try:
        # 전진 또는 후진 정지
        if key.char in ['w', 's']:
            twist.linear.x = 0.0
            print("Stop")

        # 좌회전 또는 우회전 정지
        elif key.char in ['a', 'd']:
            twist.angular.z = 0.0
            print("Center")

        pub.publish(twist)

    except AttributeError:
        pass

    # ESC 키를 누르면 프로그램 종료
    if key == keyboard.Key.esc:
        return False

# 키보드 리스너 설정
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

