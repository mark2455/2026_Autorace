#! /usr/bin/env python

import cv2
import numpy as np

def reg_of_interest(image):
    # ... 이전 코드 ...

def canny_edge(image):
    # ... 이전 코드 ...

def show_lines(image, lines):
    # ... 이전 코드 ...

def make_coordinates(image, line_parameters):
    # ... 이전 코드 ...

def average_slope_intercept(image, lines):
    # ... 이전 코드 ...

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

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