import cv2
from cv2 import inRange 
import numpy as np
import imutils

cap=cv2.VideoCapture(0)
cap.set(3, 960)
cap.set(4, 544)


while True: 


    _,frame=cap.read()

   
    cv2.imshow("FPV CAM", frame)
    

    if cv2.waitKey(1) == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
