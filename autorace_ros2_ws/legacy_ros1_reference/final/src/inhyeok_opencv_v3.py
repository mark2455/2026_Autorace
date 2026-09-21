#! /usr/bin/env python

import cv2
import numpy as np

# ... 이전에 정의한 함수들 ...

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


def reg_of_interest(image) :
    image_height = image.shape[0]
    polygons = np.array( [[ (200, image_height) , (1100, image_height), (550, 250) ]] )
    image_mask = np.zeros_like(image)
    cv2.fillPoly(image_mask, polygons, 255)
    masking_image = cv2.bitwise_and(image, image_mask)
    return masking_image

def canny_edge(image) :
    gray_conversion = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_conversion = cv2.GaussianBlur(gray_conversion, (5,5), 0)
    canny_conversion = cv2.Canny(blur_conversion, 50, 150)
    return canny_conversion


def show_lines(image, lines) : 
    lines_image = np.zeros_like(image)
    if lines is not None :
        for i in range(len(lines)):
            for x1,y1,x2,y2 in lines[i]:
                cv2.line(lines_image,(x1,y1),(x2,y2),(255,0,0), 10 )
    return lines_image

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1*(3/5))
    x1 = int((y1- intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameter = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameter[0]
        intercept = parameter[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    left_fit_average =np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis =0)
    left_line =make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)

    return np.array([[left_line, right_line]])


while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break


    canny_conversion = canny_edge(img)
    roi_conversion = reg_of_interest(canny_conversion)

    lines = cv2.HoughLinesP(roi_conversion, 1, np.pi / 180, 100, minLineLength=40, maxLineGap=5)

    if lines is not None:
        averaged_lines = average_slope_intercept(img, lines)
        lines_image = show_lines(img, averaged_lines)
        combine_image = cv2.addWeighted(img, 0.8, lines_image, 1, 1)
        cv2.imshow('Lane Detection', combine_image)
    else:
        cv2.imshow('Lane Detection', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()



#흰 검으로 변환해서 라인 검출함.
canny_conversion = canny_edge(lanelines_image)
roi_conversion = reg_of_interest(canny_conversion)

# cv2.imshow('ori', canny_conversion)
# cv2.imshow('ori', roi_conversion)

#라인 이어주기
lines = cv2.HoughLinesP(roi_conversion, 1, np.pi/180, 100, minLineLength = 40, maxLineGap = 5)

averaged_lines = average_slope_intercept(lanelines_image, lines)

#선을 기울기 평균값으로 적용
lines_image = show_lines(lanelines_image, averaged_lines)

#원본 이미지에 라인 그리기
combine_image = cv2.addWeighted(lanelines_image, 0.8, lines_image, 1, 1)

cv2.imshow('ori', lanelines_image)
cv2.imshow("roi", lines_image)
cv2.imshow("combined", combine_image)