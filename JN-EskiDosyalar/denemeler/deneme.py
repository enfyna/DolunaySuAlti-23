
from turtle import width
import cv2
import numpy as np


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
    window_title = "CSI Camera"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                
                blue_lower = np.array([90, 60, 0],np.uint8)
                blue_upper = np.array([121,255,255], np.uint8)

                blured = cv2.GaussianBlur(frame, (11,11), 0)
                hsv = cv2.cvtColor(blured, cv2.COLOR_BGR2HSV)

                # mavi için maskeleme
                mask1 = cv2.inRange(hsv, blue_lower, blue_upper)

                # görüntü temizleme (erozyon + genişleme)
                mask1 = cv2.erode(mask1, None, iterations= 2)
                mask1 = cv2.dilate(mask1, None, iterations= 2)

                # kontur bulma 
                (counters, _) = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

                center = None

                if len(counters) > 0:
                    # en büyük kontur alma
                    c = max(counters, key = cv2.contourArea)

                    #  bolding box
                    rect = cv2.minAreaRect(c)

                    ((X,y), (Widht, Height), rotation) = rect

                    box = cv2.boxPoints(rect)
                    box = np.int64(box)

                    # momentleri bulma
                    M = cv2.moments(c)
                    center = (int(M["m10"]/M["m00"]), int(M["m01"] / M["m00"]))

                    cv2.drawContours(frame, [box], 0, (0,255,255), 2)
                    cv2.circle(frame, center, 5, (255,0,255), -1)
                    cv2.putText(frame, "mavi", (50,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,0), 2)
                
                #cv2.imshow("mavi", frame)

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
