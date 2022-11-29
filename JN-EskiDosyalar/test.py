import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil
import math
import time
import numpy as np


görev_flag = True


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00')
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    # newlat  = latitude  + (dy / r_earth) * (180 / pi);
    # newlon = longitude + (dx / r_earth) * (180 / pi) / cos(dLat * pi/180);
    a = LocationGlobal(newlat, newlon, original_location.alt)
    # print("location",a)
    return a



görev_flag = True

def görev():
    # pointler 
    point1 = LocationGlobalRelative(40.1959956, 29.0430792, 100)
    point2 = LocationGlobalRelative(40.1958829, 29.0431201, 100)
    point3 = LocationGlobalRelative(40.1959014, 29.0433307, 100)
    #point4 = LocationGlobalRelative(-35.3628224,149.1650891, 100)
    #point5 = LocationGlobalRelative(-35.3627005, 149.1652420, 100)

    #noktalar = [point1,point2,point3]
    time.sleep(5)
    vehicle.simple_goto(point1)
    time.sleep(20)
    vehicle.simple_goto(point2)
    time.sleep(20)
    vehicle.simple_goto(point3)
    #time.sleep(20)
    #vehicle.simple_goto(point4)
    #time.sleep(20)
    #vehicle.simple_goto(point5)



def show_camera():
    window_title = "CSI Camera"
    global görev_flag
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()

                #     BİZİ ALGILAMAYAN  
                #    red_lower = np.array([160,150,50], np.uint8)
                #    red_upper = np.array([180,255,150], np.uint8)

                red_lower = np.array([160,150,150], np.uint8)
                red_upper = np.array([180,255,250], np.uint8)

                # hsv dönütürmeq
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # kırmızı için maskeleme
                mask2 = cv2.inRange(hsv, red_lower, red_upper)
                
                # kırmızı için temizleme
                mask2 = cv2.erode(mask2, None, iterations= 2)
                mask2 = cv2.dilate(mask2, None, iterations= 2)

                # kırmızı için kontur bulma 
                (counters_red, _) = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE )

                center_red = None

                if len(counters_red) > 0:

                    # görev başlatma komutu
                    
                    if (görev_flag):
                        görev()
                        görev_flag = False


                    c = max(counters_red, key=cv2.contourArea)

                    rect_red = cv2.minAreaRect(c)

                    ((x_red, y_red), (width_red, Height_red), rotation_red) = rect_red
                
                    box_red = cv2.boxPoints(rect_red)
                    box_red = np.int64(box_red)

                    # red moment bulma 

                    M = cv2.moments(c)
                    center_red = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                    # red konturları çizdirme 
                    cv2.drawContours(frame, [box_red], 0, (0,255,255), 2)
                    cv2.circle(frame, center_red, 5, (255,0,255), -1)
                    cv2.putText(frame, "red", (50,75), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,0), 2)
                cv2.imshow("window_title", frame)


                if cv2.waitKey(1) & 0xFF == ord("q"): break

                
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break 

                if cv2.waitKey(1) & 0XFF == ord("q"): break
                """
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
                """
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()




