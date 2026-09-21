#!/usr/bin/env python3

import rospy
import cv2, time, base64

framesCaptured = 0
video=cv2.VideoCapture(0) 

# Save camera frames as movie
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('recording.avi', fourcc, 20.0, (640, 480))

while True:
    framesCaptured += 1

    check, frame = video.read() 
    # Write video frames
    out.write(frame)

    # Print out statements
    #print(check)
    print(frame)

    gray = cv2.VideoCapture(0) 

    cv2.imshow("frame", gray)   
    print(type(frame))

    key=cv2.waitKey(1)

    if key == ord('q'):
        break




#print('Frames captured: ' + str(framesCaptured))
video.release()
out.release()
cv2.destroyAllWindows