import paramiko
import cv2 
import time

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

client.connect('dolunay-desktop',username='dolunay',password="12345")

sftp = client.open_sftp()
while True:
    time.sleep(0.03)
    try:
        sftp.get('kamera'+'.png', 'photo.png')
    except:
        pass
    cv2.waitKey(1)

client.close()
