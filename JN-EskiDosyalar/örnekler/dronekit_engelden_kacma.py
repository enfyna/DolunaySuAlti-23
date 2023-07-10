from turtle import delay
import plane_lib, time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from math import sin, cos, sqrt, atan2, radians
import cv2
import numpy as np
import imutils

cruise_altitude = 30

plane = plane_lib.Plane('/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00')

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

def engelden_kacma():
	plane.set_ap_mode("GUIDED")
	print("guided")
	while(1):
		plane.turn_heading("l", 15, cruise_altitude)
		time.sleep(1.5)
		break
	while(1):
		plane.turn_heading("l", -75, cruise_altitude)
		time.sleep(7)
		break
	while(1):
		plane.turn_heading("l", 75, cruise_altitude)
		time.sleep(7)
		break
	while(1):
		plane.turn_heading("l", -75, cruise_altitude)
		time.sleep(7)
		break
	while(1):
		plane.turn_heading("l", 75, cruise_altitude)
		time.sleep(7)
		break
	while(1):
		plane.turn_heading("l", -15, cruise_altitude)
		time.sleep(1.5)
		break

	plane.set_ap_mode("AUTO")

def görev_sonu():
	while True:
		print("İskeleye varıldı")