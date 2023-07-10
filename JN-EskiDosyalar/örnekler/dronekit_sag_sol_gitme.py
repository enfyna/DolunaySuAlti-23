from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from math import sin, cos, sqrt, atan2, radians
from turtle import delay
from pickle import TRUE
import plane_lib, time
import numpy as np
import imutils
import cv2

cruise_altitude = 30

plane = plane_lib.Plane("/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00")

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

def get_distance_metres(aLocation1, aLocation2):
	R = 6378137.0
	lat1 = radians(aLocation1.lat)
	lon1 = radians(aLocation1.lon)
	lat2 = radians(aLocation2.lat)
	lon2 = radians(aLocation2.lon)
	dlon = lon2 - lon1
	dlat = lat2 - lat1
	a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
	c = 2 * atan2(sqrt(a), sqrt(1 - a))
	distance = R * c
	# meter = distance * 1000.0
	print(distance)
	return distance

def distance_to_current_waypoint():
	"""
	Gets distance in metres to the current waypoint.
	It returns None for the first waypoint (Home location).
	"""
	nextwaypoint = plane.vehicle.commands.next

		# ben 1000 yaptım YEC

	missionitem=plane.vehicle.commands[0] #commands are zero indexed
	lat = missionitem.x
	lon = missionitem.y
	alt = missionitem.z
	targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
	distancetopoint = get_distance_metres(plane.vehicle.location.global_frame, targetWaypointLocation)

	return distancetopoint

def sagdan():

	plane.set_ap_mode("GUIDED")
	print("guided")

	plane.turn_heading("l", -15, cruise_altitude)
	time.sleep(0.0001)
	plane.turn_heading("l",20 , cruise_altitude)
	time.sleep(1)

	plane.set_ap_mode("AUTO")

def soldan():

	plane.set_ap_mode("GUIDED")
	print("guided")

	plane.turn_heading("l", 15, cruise_altitude)
	time.sleep(0.0001)
	plane.turn_heading("l",-20 , cruise_altitude)
	time.sleep(1)

	plane.set_ap_mode("AUTO")

def görev_sonu():
	while True:
		plane.set_ap_mode("HOLD")