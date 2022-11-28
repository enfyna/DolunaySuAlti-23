import cv2
import numpy as np
import imutils 

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


def show_camera():
    window_title = "CSI Camera - Uzaklik"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)


            while True: 
                    ret_val, frame = video_capture.read()
                    
                    lower_blue = np.array([90,60,0],np.uint64) ## mavi renk algilama
                    upper_blue = np.array([121,255,255],np.uint64)
                    
                    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    
                    mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
                    
                    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    contours = imutils.grab_contours(contours)

                    for i in contours:
                        area = cv2.contourArea(i)
                        distance = 2*(10**(-7))* (area**2) - (0.0067 * area) + 83.487   ## uzaklik hesaplayan formul
                        
                        if area > 5000:
                            cv2.drawContours(frame, [i], -1, (0,255,0), 3)

                            M = cv2.moments(i)
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            S = 'Cismin Uzakligi = ' + str(distance) 
                        
                            cv2.circle(frame, (cx, cy), 7, (255,255,255), -1)
                            cv2.putText(frame, S , (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 3)
                            

                    cv2.imshow("Uzaklik  Tespiti", frame)

                
                    if cv2.waitKey(1) & 0xFF == ord('q'): break
                    
        finally:
            video_capture.release()
            cv2.destrolAllWindows()
    else:
            print("Hata: Kamera acilamadi")

if __name__ == "__main__":
    show_camera()


            

































