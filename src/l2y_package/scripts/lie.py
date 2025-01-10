#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

# 모터 핀 정의
frontDirPin = 6       # 앞바퀴 방향 제어 핀
frontPwmPin = 9       # 앞바퀴 속도 제어 핀 (PWM 핀)
backDirPin = 7        # 뒷바퀴 방향 제어 핀
backPwmPin = 10       # 뒷바퀴 속도 제어 핀 (PWM 핀)
steerDirPin = 2       # 조향 모터 방향 제어 핀
steerPwmPin = 4       # 조향 모터 속도 제어 핀 (PWM 핀)

# 모터 속도 변수
maxSpeed = 255        # 최대 속도 (PWM 범위: 0-255)
steerAdjustment = 100 # 조향 조정 값

def setup():
    # 모터 핀 설정
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(frontDirPin, GPIO.OUT)
    GPIO.setup(frontPwmPin, GPIO.OUT)
    GPIO.setup(backDirPin, GPIO.OUT)
    GPIO.setup(backPwmPin, GPIO.OUT)
    GPIO.setup(steerDirPin, GPIO.OUT)
    GPIO.setup(steerPwmPin, GPIO.OUT)

    # PWM 설정
    global frontPwm, backPwm, steerPwm
    frontPwm = GPIO.PWM(frontPwmPin, 100)
    backPwm = GPIO.PWM(backPwmPin, 100)
    steerPwm = GPIO.PWM(steerPwmPin, 100)
    frontPwm.start(0)
    backPwm.start(0)
    steerPwm.start(0)

def setMotorSpeed(frontSpeed, backSpeed):
    # 앞바퀴 제어
    if frontSpeed >= 0:
        GPIO.output(frontDirPin, GPIO.LOW)
        frontPwm.ChangeDutyCycle(frontSpeed / maxSpeed * 100)
    else:
        GPIO.output(frontDirPin, GPIO.HIGH)
        frontPwm.ChangeDutyCycle(-frontSpeed / maxSpeed * 100)

    # 뒷바퀴 제어
    if backSpeed >= 0:
        GPIO.output(backDirPin, GPIO.LOW)
        backPwm.ChangeDutyCycle(backSpeed / maxSpeed * 100)
    else:
        GPIO.output(backDirPin, GPIO.HIGH)
        backPwm.ChangeDutyCycle(-backSpeed / maxSpeed * 100)

def setSteerMotor(direction, speed):
    # 조향 모터 제어
    GPIO.output(steerDirPin, direction)
    steerPwm.ChangeDutyCycle(speed / maxSpeed * 100)

def applyBrake():
    # 앞바퀴 브레이크 모드
    GPIO.output(frontDirPin, GPIO.LOW)
    frontPwm.ChangeDutyCycle(0)

    # 뒷바퀴 브레이크 모드
    GPIO.output(backDirPin, GPIO.LOW)
    backPwm.ChangeDutyCycle(0)

    # 조향 모터 브레이크 모드 (정지)
    GPIO.output(steerDirPin, GPIO.LOW)
    steerPwm.ChangeDutyCycle(0)

def driveSequence():
    # 10초 동안 직진
    setMotorSpeed(0.3 * maxSpeed, 0.3 * maxSpeed)
    setSteerMotor(GPIO.LOW, 0)
    time.sleep(18)

    # 왼쪽 조향각 30도로 설정하고 4초 동안 주행
    setMotorSpeed(0.3 * maxSpeed, 0.3 * maxSpeed)
    setSteerMotor(GPIO.HIGH, 30)
    time.sleep(1.5)

    # 10초 동안 직진
    setMotorSpeed(0.3 * maxSpeed, 0.3 * maxSpeed)
    setSteerMotor(GPIO.LOW, 0)
    time.sleep(10)

    # 오른쪽 조향각 30도로 설정하고 5초 동안 주행
    setMotorSpeed(0.3 * maxSpeed, 0.3 * maxSpeed)
    setSteerMotor(GPIO.LOW, 30)
    time.sleep(1.5)

    # 조향각을 원위치하고 5초 동안 직진
    setMotorSpeed(0.3 * maxSpeed, 0.3 * maxSpeed)
    setSteerMotor(GPIO.LOW, 0)
    time.sleep(15)

    # 브레이크 활성화
    applyBrake()

def main():
    rospy.init_node('motor_drive_sequence')
    setup()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        driveSequence()
        rate.sleep()
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
