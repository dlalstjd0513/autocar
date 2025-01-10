#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

def process_image(frame):
    """
    차선 탐지 알고리즘을 적용하여 양쪽 차선에 점을 찍는 함수.
    왼쪽 차선은 하늘색 점, 오른쪽 차선은 빨간색 점을 표시.
    :param frame: USB 웹캠으로부터 입력된 이미지 (프레임)
    :return: 각 차선의 중심과 이미지의 중심 간의 오차 (CTE)
    """

    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 가우시안 블러 적용 (노이즈 제거)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Canny 엣지 검출 적용
    edges = cv2.Canny(blur, 50, 150)

    # 관심 영역(ROI) 설정 (하단 부분만 차선 탐지)
    height, width = frame.shape[:2]
    roi_top = int(height * 2 / 5)  # ROI의 상단 위치 (기존보다 위로 올림)
    roi_bottom = int(height * 4 / 5)  # ROI의 하단 위치
    roi = edges[roi_top:roi_bottom, :]

    # ROI 범위 시각적으로 표시
    cv2.rectangle(frame, (0, roi_top), (width, roi_bottom), (0, 255, 255), 2)  # 노란색 사각형으로 ROI 표시

    # 허프 변환을 사용하여 직선 검출
    lines = cv2.HoughLinesP(roi, 1, np.pi/180, 100, minLineLength=50, maxLineGap=150)

    left_lines = []  # 왼쪽 차선을 저장할 리스트
    right_lines = []  # 오른쪽 차선을 저장할 리스트

    if lines is not None:
        # 검출된 직선들을 분류 (왼쪽과 오른쪽 차선으로 구분)
        for line in lines:
            x1, y1, x2, y2 = line[0]  # [0]을 추가하여 라인을 가져옵니다

            # 기울기를 통해 왼쪽과 오른쪽 차선을 구분
            slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else np.inf

            if slope < 0:  # 왼쪽 차선
                left_lines.append([x1, y1, x2, y2])
            elif slope > 0:  # 오른쪽 차선
                right_lines.append([x1, y1, x2, y2])

        # 왼쪽 차선의 중앙 좌표 계산
        if len(left_lines) > 0:
            left_lane_center = np.mean([line[0] + (line[2] - line[0]) / 2 for line in left_lines])
            # 왼쪽 차선의 중앙에 하늘색 점 표시
            cv2.circle(frame, (int(left_lane_center), (roi_top + roi_bottom) // 2), 5, (255, 255, 0), -1)

        # 오른쪽 차선의 중앙 좌표 계산
        if len(right_lines) > 0:
            right_lane_center = np.mean([line[0] + (line[2] - line[0]) / 2 for line in right_lines])
            # 오른쪽 차선의 중앙에 빨간색 점 표시
            cv2.circle(frame, (int(right_lane_center), (roi_top + roi_bottom) // 2), 5, (0, 0, 255), -1)

        # 검출된 직선들을 초록색 선으로 그리기
        for line in lines:
            x1, y1, x2, y2 = line[0]  # [0]으로 접근합니다
            cv2.line(frame, (x1, y1 + roi_top), (x2, y2 + roi_top), (0, 255, 0), 2)  # 초록색 선으로 차선 그리기

    # 이미지의 중심 표시 (파란색 점)
    image_center = width / 2
    roi_mid = (roi_top + roi_bottom) // 2  # ROI의 세로 중간 부분
    cv2.circle(frame, (int(image_center), roi_mid), 5, (255, 0, 0), -1)

    return frame

def main():
    # 웹캠 초기화
    cap = cv2.VideoCapture(2)  # 0번 카메라(기본 USB 웹캠)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # 차선 인식 및 오차 계산
        processed_frame = process_image(frame)

        # 결과 출력
        cv2.imshow('Lane Detection', processed_frame)

        # ESC 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

