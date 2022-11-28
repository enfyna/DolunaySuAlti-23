import cv2
import numpy as np
from time import time
import socket
#from goprocam import GoProCamera
#from goprocam import constants
#gpCam = GoProCamera.GoPro()
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# t=time()
# gpCam.livestream("start")
cap = cv2.VideoCapture(0)

while True:
    nmat, frame = cap.read()
    cv2.imshow("GoPro OpenCV", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # if time() - t >= 2.5:
    #     sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("0.0.0.0", 8554))
    #     t=time()
# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
