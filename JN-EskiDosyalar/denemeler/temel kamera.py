import cv2
import numpy as np

#Read Cam
cap = cv2.VideoCapture(0)
cap.set(3,1000)
cap.set(4,1000)
cap.set(10,1000)

while True:
    success, img = cap.read()
    cv2.imshow("video",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
break

