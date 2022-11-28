
from turtle import width
import cv2
from cv2 import RETR_EXTERNAL
from cv2 import erode
import numpy as np


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=-1,
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
                
                # renk aralıkları 
                blue_lower = np.array([90, 60, 0],np.uint8)
                blue_upper = np.array([121,255,255], np.uint8)

                red_lower = np.array([0,85,0], np.uint8)
                red_upper = np.array([15,155,255], np.uint8)

                yellow_lower = np.array([20,100,100], np.uint8)
                yellow_upper = np.array([30,255,255], np.uint8)

                green_lower = np.array([36,25,25], np.uint8)
                green_upper = np.array([70,255,255],np.uint8)




                #blured = cv2.GaussianBlur(frame, (11,11), 0)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # mavi için maskeleme
                mask1 = cv2.inRange(hsv, blue_lower, blue_upper)

                
                # görüntü temizleme (erozyon + genişleme)
                mask1 = cv2.erode(mask1, None, iterations= 1)
                mask1 = cv2.dilate(mask1, None, iterations= 1)

                # mavi için kontur bulma 
                (counters_blue, _) = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

                center_blue = None

                if len(counters_blue) > 0:
                    # en büyük kontur alma
                    c = max(counters_blue , key = cv2.contourArea)

                    #  bolding box
                    rect = cv2.minAreaRect(c)

                    ((X,y), (Widht, Height), rotation) = rect

                    box = cv2.boxPoints(rect)
                    box = np.int64(box)

                    # momentleri bulma
                    M = cv2.moments(c)
                    center_blue = (int(M["m10"]/(M["m00"]+1)), int(M["m01"] / (M["m00"]+1)))

                    cv2.drawContours(frame, [box], 0, (0,255,255), 2)
                    cv2.circle(frame, center_blue, 5, (255,0,255), -1)
                    cv2.putText(frame, "mavi", (50,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,0), 2)
                
                # kırmızı için maskeleme
                mask2 = cv2.inRange(hsv, red_lower, red_upper)
                
                # kırmızı için temizleme
                #mask2 = cv2.erode(mask2, None, iterations= 1)
                #mask2 = cv2.dilate(mask2, None, iterations= 1)

                # kırmızı için kontur bulma 
                (counters_red, _) = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE )

                center_red = None

                if len(counters_red) > 0:
                    c = max(counters_red, key=cv2.contourArea)

                    rect_red = cv2.minAreaRect(c)

                    ((x_red, y_red), (width_red, Height_red), rotation_red) = rect_red
                
                    box_red = cv2.boxPoints(rect_red)
                    box_red = np.int64(box_red)

                    # red moment bulma 
    
                    M = cv2.moments(c)
                    center_red = (int(M["m10"] / (M["m00"]+1)), int(M["m01"] / (M["m00"]+1)))

                    # red konturları çizdirme 
                    cv2.drawContours(frame, [box_red], 0, (0,255,255), 2)
                    cv2.circle(frame, center_red, 5, (255,0,255), -1)
                    cv2.putText(frame, "red", (50,200), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,0), 2)

                
                # sarı için maskeleme
                mask3 = cv2.inRange(hsv, yellow_lower, yellow_upper)

                # sarı için temizleme
                #mask3 = cv2.erode(mask3, None, iterations= 1)
                #mask3 = cv2.dilate(mask3, None, iterations= 1)

                # sarı için kontur bulma
                (contour_yellow, _) = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

                center_yellow = None

                if len(contour_yellow) > 0 :
                    c = max(contour_yellow, key= cv2.contourArea)

                    rect_yellow = cv2.minAreaRect(c)

                    ((x_yellow, y_yellow), (weight_yellow, height_yellow), rotation_yellow) =  rect_yellow

                    box_yellow = cv2.boxPoints(rect_yellow)
                    box_yellow = np.int64(box_yellow)

                    # yellow moment bulma 
                    M = cv2.moments(c)
                    center_yellow = (int(M["m10"] / (M["m00"]+1)) , int(M["m01"] / (M["m00"]+1)))

                    # yellow kontur çizdirme

                    cv2.drawContours(frame, [box_yellow], 0, (0,255,255), 2)
                    cv2.circle(frame, center_yellow, 5, (255,0,255), -1)
                    cv2.putText(frame, "yellow", (50,100), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,0), 2)

                # yeşil içil için maskeleme
                mask4 = cv2.inRange(hsv, green_lower, green_upper)

                # yeşil için temizleme
                #mask4 = cv2.erode(mask4, None, iterations= 1)
                #mask4 = cv2.dilate(mask4, None, iterations= 1)

                # yeşil için kontur bulma
                (conteur_green, _) = cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

                center_green = None

                if len(conteur_green) > 0:
                    c = max(conteur_green, key = cv2.contourArea)

                    rect_green = cv2.minAreaRect(c)

                    box_green = cv2.boxPoints(rect_green)
                    box_green = np.int64(box_green)

                    # green için momentlerl bulma 
                    M = cv2.moments(c)
                    center_green = (int(M["m10"] / (M["m00"]+1)) , int(M["m01"] /( M["m00"]+1)))

                    # yellow kontur çizdirme

                    cv2.drawContours(frame, [box_green], 0, (0,255,255), 2)
                    cv2.circle(frame, center_green, 5, (255,0,255), -1)
                    cv2.putText(frame, "yesil", (50,75), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,0), 2)



                

                
                
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
