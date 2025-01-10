#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import warnings

# 차선 인식에 사용할 상수 설정
num_windows = 10
window_margin = 100
min_num_pixel = 30  # 최소 픽셀 수를 줄여 차선 검출 확률 증가
ROI_START_ROW = 400  # ROI 시작 행 조정 (이미지의 하단)
ROI_END_ROW = 700    # ROI 끝 행 조정 (이미지 하단)
WIDTH = 640

class LaneDetector:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('lane_drive_node', anonymous=True)

        # 카메라 이미지 구독 (usb_cam 노드가 퍼블리시하는 토픽 사용)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # cv_bridge 초기화
        self.bridge = CvBridge()

        # 이전 차선 좌표 저장
        self.prev_x_left = 0
        self.prev_x_right = WIDTH

        # 메인 루프 시작
        rospy.spin()

    def image_callback(self, data):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # 차선 인식 수행
        self.process_image(cv_image)

    def process_image(self, img):
        # Step 1: Edge, Sobel Filter, Gradient 계산
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]
        
        # Sobel 필터를 사용한 x, y 방향의 편미분
        sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(l_channel, cv2.CV_64F, 0, 1, ksize=3)

        # Sobel 기반 Edge 추출 (Canny 사용, 임계값 조정)
        edge_img = cv2.Canny(np.uint8(l_channel), 30, 100)

        # Step 2: Perspective Transform 적용
        warp_img, matrix = self.perspective_transform(edge_img)

        # Step 3: 히스토그램을 그려 차선의 시작점 탐색 (분석 범위 조정)
        histogram = np.sum(warp_img[warp_img.shape[0]//3:, :], axis=0)
        midpoint = int(histogram.shape[0] // 2)
        start_leftX = np.argmax(histogram[:midpoint])
        start_rightX = np.argmax(histogram[midpoint:]) + midpoint

        # Step 4: 슬라이딩 윈도우 탐색
        win_left_lane, win_right_lane, nonzerox, nonzeroy = self.sliding_window_search(warp_img, start_leftX, start_rightX)

        # Step 5: 2차 다항식 피팅
        leftx, lefty = nonzerox[win_left_lane], nonzeroy[win_left_lane]
        rightx, righty = nonzerox[win_right_lane], nonzeroy[win_right_lane]
        left_fit, right_fit, left_plotx, right_plotx, ploty = self.fit_polynomial(leftx, lefty, rightx, righty, warp_img.shape)

        if left_fit is None or right_fit is None:
            return  # 차선 검출 실패 시 아무것도 하지 않음

        # Step 6: 차선 그리기
        result = self.draw_lane(img, warp_img, left_plotx, right_plotx, ploty, np.linalg.inv(matrix))

        # 결과 이미지 출력
        cv2.imshow("Lane Detection", result)
        cv2.waitKey(1)

    def perspective_transform(self, img):
        height, width = img.shape[:2]
        src = np.float32([[width * 0.45, height * 0.63], 
                          [width * 0.55, height * 0.63],
                          [width * 0.1, height], 
                          [width * 0.9, height]])
        dst = np.float32([[width * 0.2, 0],
                          [width * 0.8, 0],
                          [width * 0.2, height],
                          [width * 0.8, height]])

        matrix = cv2.getPerspectiveTransform(src, dst)
        warp_img = cv2.warpPerspective(img, matrix, (width, height))

        return warp_img, matrix

    def sliding_window_search(self, warp_img, start_leftX, start_rightX):
        window_height = int(warp_img.shape[0] // num_windows)
        nonzero = warp_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        current_leftX = start_leftX
        current_rightX = start_rightX

        win_left_lane = []
        win_right_lane = []

        for window in range(num_windows):
            win_y_low = warp_img.shape[0] - (window + 1) * window_height
            win_y_high = warp_img.shape[0] - window * window_height
            win_leftx_min = current_leftX - window_margin
            win_leftx_max = current_leftX + window_margin
            win_rightx_min = current_rightX - window_margin
            win_rightx_max = current_rightX + window_margin

            left_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high) &
                                (nonzerox >= win_leftx_min) & (nonzerox <= win_leftx_max)).nonzero()[0]
            right_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high) &
                                 (nonzerox >= win_rightx_min) & (nonzerox <= win_rightx_max)).nonzero()[0]

            win_left_lane.append(left_window_inds)
            win_right_lane.append(right_window_inds)

            if len(left_window_inds) > min_num_pixel:
                current_leftX = int(np.mean(nonzerox[left_window_inds]))
            if len(right_window_inds) > min_num_pixel:
                current_rightX = int(np.mean(nonzerox[right_window_inds]))

        win_left_lane = np.concatenate(win_left_lane)
        win_right_lane = np.concatenate(win_right_lane)

        return win_left_lane, win_right_lane, nonzerox, nonzeroy

    def fit_polynomial(self, leftx, lefty, rightx, righty, img_shape):
        # 최소 픽셀 개수 설정
        min_pixels = 30  # 최소 30개의 픽셀이 있어야 함

        # 차선 픽셀이 검출되었는지 확인 (픽셀이 너무 적으면 건너뜀)
        if len(leftx) < min_pixels or len(lefty) < min_pixels or len(rightx) < min_pixels or len(righty) < min_pixels:
            rospy.logwarn("Insufficient lane pixels found! Using previous fit or skipping frame.")
            return None, None, None, None, None  # 차선을 찾지 못한 경우 None 반환

        # RankWarning 무시
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', np.RankWarning)
            
            # 2차 다항식 피팅
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, img_shape[0] - 1, img_shape[0])
        left_plotx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_plotx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        return left_fit, right_fit, left_plotx, right_plotx, ploty

    def draw_lane(self, img, warp_img, left_plotx, right_plotx, ploty, Minv):
        warp_zero = np.zeros_like(warp_img).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_plotx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_plotx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))  # np.int_ 사용
        newwarp = cv2.warpPerspective(color_warp, Minv, (img.shape[1], img.shape[0]))
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)

        return result

if __name__ == '__main__':
    try:
        LaneDetector()
    except rospy.ROSInterruptException:
        pass

