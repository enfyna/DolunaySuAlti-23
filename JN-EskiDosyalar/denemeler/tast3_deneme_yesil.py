from pickle import TRUE
from turtle import delay
import plane_lib, time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from math import sin, cos, sqrt, atan2, radians
import cv2 
import numpy as np
import imutils

cruise_altitude = 30

plane = plane_lib.Plane("/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00")


while True:
    print(plane.get_ap_mode())

    if plane.get_ap_mode() == "ACRO":
        plane.set_ap_mode("AUTO")
        break


def get_distance_metres(aLocation1, aLocation2):
    R = 6378137.0
    lat1 = radians(aLocation1.lat)
    lon1 = radians(aLocation1.lon)
    lat2 = radians(aLocation2.lat)
    lon2 = radians(aLocation2.lon)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    # meter = distance * 1000.0
    print(distance)
    return distance

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = plane.vehicle.commands.next

   
        # ben 1000 yaptım YEC

    missionitem=plane.vehicle.commands[0] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(plane.vehicle.location.global_frame, targetWaypointLocation)
    

    return distancetopoint

def sagdan():

    plane.set_ap_mode("GUIDED")
    print("guided")
    while(1):
        plane.turn_heading("l", -25, cruise_altitude)
        time.sleep(0.001)
        break
    while(1):
        plane.turn_heading("l",30 , cruise_altitude)
        time.sleep(3)
        break
    plane.set_ap_mode("AUTO")


def soldan():

    plane.set_ap_mode("GUIDED")
    print("guided")
    while(1):
        plane.turn_heading("l", 25, cruise_altitude)
        time.sleep(0.001)
        break
    while(1):
        plane.turn_heading("l",-30 , cruise_altitude)
        time.sleep(3)
        break
    plane.set_ap_mode("AUTO")




def donus():

    plane.set_ap_mode("GUIDED")
    print("guided")

    while(1):

        plane.turn_heading("l",-60 , cruise_altitude)
        time.sleep(3)
        break
    while(1):
        plane.turn_heading("l", 60 , cruise_altitude)
        time.sleep(3)
        break
    while(1):
        plane.turn_heading("l", 60 , cruise_altitude)
        time.sleep(3)
        break
    while(1):
        plane.turn_heading("l", 60 , cruise_altitude)
        time.sleep(3)
        break

    plane.set_ap_mode("RTL")
    #plane.set_ap_wp = 2








cap = cv2.VideoCapture(0)
cap.set(3, 960)
cap.set(4, 544)

while True:

    limit = 15000
    _,frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red_1 = np.array([0,180,80])
    upper_red_1 = np.array([10,255,255])

    lower_red_2 = np.array([170,100,70])
    upper_red_2 = np.array([180,255,255])

    lower_yellow = np.array([20,100,100])
    upper_yellow = np.array([30,255,255])
    
    lower_green = np.array([40,70,80])
    upper_green = np.array([70,255,255])

    lower_blue = np.array([90,60,0])
    upper_blue = np.array([121,255,255])

    mask_red_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask1 = mask_red_1 + mask_red_2
    mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask3 = cv2.inRange(hsv, lower_green, upper_green)
    mask4 = cv2.inRange(hsv, lower_blue, upper_blue)

    cnts1 = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts1 = imutils.grab_contours(cnts1)

    cnts2 = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts2 = imutils.grab_contours(cnts2)

    cnts3 = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts3 = imutils.grab_contours(cnts3)

    cnts4 = cv2.findContours(mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts4 = imutils.grab_contours(cnts4)

    # kırmızı
    for c in cnts1:
        alan1 = cv2.contourArea(c)
        if alan1 > 5000:
            if (alan1 > limit) :
                sagdan()

            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Kirmizi", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

    # sarı
    for c in cnts2:
        alan2 = cv2.contourArea(c)
        if alan2 > 5000:
            """
            if alan2 > limit: 
                soldan()
            """

            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Sari", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

    # yeşil 
    for c in cnts3:
        alan3 = cv2.contourArea(c)
        if alan3 > 5000:
            if alan3 > limit:
                soldan()
            


            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Yesil", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)


    # mavi
    for c in cnts4:
        alan4 = cv2.contourArea(c)
        if alan4 > 5000:
            
            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])

            cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
            cv2.putText(frame, "Mavi", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

    cv2.imshow("Renk Tespit", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()