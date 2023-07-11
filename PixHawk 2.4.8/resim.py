import cv2
import datetime

def kaydet(frame):
    cv2.imwrite(
        datetime.datetime.now().strftime("%X").replace(":","") + ".png",
        frame
    )