#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

# ROS 노드 초기화
rospy.init_node('keyboard_control')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Twist 메시지 생성
twist = Twist()

# 속도 조절 변수
linear_speed = 0.1  # 전진/후진 속도
angular_speed = 0.2  # 회전 속도
speed_increment = 0.1  # 속도 증가/감소 값

# 키 상태 변수
key_state = {
    'w': False,
    's': False,
    'a': False,
    'd': False
}

# 키 입력 상태에 따른 Twist 메시지 갱신
def update_twist():
    # 전진 및 후진 상태
    if key_state['w']:
        twist.linear.x = linear_speed
    elif key_state['s']:
        twist.linear.x = -linear_speed
    else:
        twist.linear.x = 0.0

    # 좌회전 및 우회전 상태
    if key_state['a']:
        twist.angular.z = angular_speed
    elif key_state['d']:
        twist.angular.z = -angular_speed
    else:
        twist.angular.z = 0.0

    # 메시지 발행
    pub.publish(twist)
    print(f"Updated Twist: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")

def on_press(key):
    global linear_speed, angular_speed
    try:
        # 방향 키 입력 상태 업데이트
        if key.char in key_state:
            key_state[key.char] = True
            update_twist()

        # 속도 증가
        elif key.char == '+':
            linear_speed += speed_increment
            angular_speed += speed_increment / 2
            print(f"Speed Increased: Linear = {linear_speed}, Angular = {angular_speed}")

        # 속도 감소
        elif key.char == '-':
            linear_speed = max(0.1, linear_speed - speed_increment)
            angular_speed = max(0.1, angular_speed - speed_increment / 2)
            print(f"Speed Decreased: Linear = {linear_speed}, Angular = {angular_speed}")

        # 브레이크 모드 (정지)
        elif key.char == 'b':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
            print("Brake: Full Stop")

    except AttributeError:
        pass

def on_release(key):
    try:
        # 방향 키 해제 상태 업데이트
        if key.char in key_state:
            key_state[key.char] = False
            update_twist()

    except AttributeError:
        pass

    # ESC 키를 누르면 프로그램 종료
    if key == keyboard.Key.esc:
        return False

def main():
    # 키보드 리스너 설정
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        print("Keyboard control started. Use 'w', 's', 'a', 'd' to move, '+' or '-' to change speed, 'b' for brake. Press ESC to exit.")
        listener.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 프로그램 종료 시 로봇 정지
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        print("Node shutdown, robot stopped.")

