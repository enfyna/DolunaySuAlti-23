from turtle import delay
import plane_lib, time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from math import sin, cos, sqrt, atan2, radians
import cv2 
import numpy as np
import imutils

cruise_altitude = 30

plane = plane_lib.Plane("/dev/serial/by-id/usb-ArduPilot_Pixhawk1_19003D000551353532333634-if00")

while True:
    print(plane.get_ap_mode())

    if plane.get_ap_mode() == "ACRO":
        plane.set_ap_mode("AUTO")
        break

def engelden_kacma():

    plane.set_ap_mode("GUIDED")
    print("guided")

    plane.turn_heading("l", -25, cruise_altitude)
    time.sleep(0.001)
    #plane.turn_heading("l",60 , cruise_altitude)
    #time.sleep(0.001)

    plane.set_ap_mode("AUTO")
        
cap = cv2.VideoCapture(0)
cap.set(3, 960)
cap.set(4, 544)

while True:

    _,frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red_1 = np.array([0,180,80])
    upper_red_1 = np.array([10,255,255])

    lower_red_2 = np.array([170,100,70])
    upper_red_2 = np.array([180,255,255])

    lower_yellow = np.array([15,80,70])
    upper_yellow = np.array([35,255,255])

    lower_green = np.array([40,70,80])
    upper_green = np.array([70,255,255])

    lower_white = np.array([5, 5, 0])
    upper_white = np.array([35, 35, 255])
   

    mask_red1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_red2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    
    mask1 = mask_red1 + mask_red2
    mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask3 = cv2.inRange(hsv, lower_green, upper_green)
    mask4 = cv2.inRange(hsv, lower_white, upper_white)

    cnts1 = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts1 = imutils.grab_contours(cnts1)

    cnts2 = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts2 = imutils.grab_contours(cnts2)

    cnts3 = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts3 = imutils.grab_contours(cnts3)

    cnts4 = cv2.findContours(mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts4 = imutils.grab_contours(cnts4)

    for c in cnts1:
        alan1 = cv2.contourArea(c)
        if alan1 > 1000:
            
            # engelden kaçma
            if alan1 > 15000:
                engelden_kacma()

            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Kirmizi", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

    for c in cnts2:
        alan2 = cv2.contourArea(c)
        if alan2 > 1000:

            # engelden kaçma
            if alan2 > 15000:
                engelden_kacma()

            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Sari", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

    for c in cnts3:
        alan3 = cv2.contourArea(c)
        if alan3 > 1000:
            
            if alan3 > 15000:
            # engelden kaçma
                engelden_kacma()

            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Yesil", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

    for c in cnts4:
        alan4 = cv2.contourArea(c)
        if alan4 > 1000:

            # engelden kaçma
            if alan4 > 15000:
                engelden_kacma()

            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Beyaz", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

    cv2.imshow("Renk Tespit", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
