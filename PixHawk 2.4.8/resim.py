import cv2
import datetime

def kaydet(frame):
    cv2.imwrite(str(datetime.datetime.now().strftime("%X")).replace(":","")
                + ".png",frame)  