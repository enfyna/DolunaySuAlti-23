# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 13:29:12 2023

@author: Yunus
"""

import paramiko
import cv2 
import time

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

client.connect('raspberrypi.local',username='dolunay',password="123456")

sftp = client.open_sftp()

while True:
    time.sleep(0.01)
    try:
        sftp.get('kamera.png', 'photo.png')
        img = cv2.imread('photo.png')
        cv2.imshow("Image",img)
    except:
        pass
    cv2.waitKey(1)
    if ord('q') == cv2.waitKey(33):
        break
client.close()
cv2.destroyAllWindows()