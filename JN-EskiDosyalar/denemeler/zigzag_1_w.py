from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil
import math
import time
import numpy as np
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

### araç ile bağlantı
import argparse
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
					default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2B004D000B51373232393438-if00')
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
"""
if not connection_string:
	import dronekit_sitl
	sitl = dronekit_sitl.start_default()
	connection_string = sitl.connection_string()

"""
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

###
def nokta_ekleme(hedef):
	a_file = open("nokta_sekiz.txt", "r")
	list_of_lines = a_file.readlines()
	list_of_lines[9] = "8    0    3    16    0.00000000    0.00000000    0.00000000    0.00000000    " + str(
		hedef.lat) + "\t" + str(hedef.lon) + "\t" + "20.000000    1 \n"

	list_of_lines[21] = "20    0    3    16    0.00000000    0.00000000    0.00000000    0.00000000    " + str(hedef.lat) + "\t" + str(hedef.lon) + "\t" + "20.000000    1 \n"

	a_file = open("yeni_1.txt", "w")
	a_file.writelines(list_of_lines)
	a_file.close()

def readmission(aFileName):
	"""
	Load a mission from a file into a list. The mission definition is in the Waypoint file
	format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
	This function is used by upload_mission().
	"""
	print("\nReading mission from file: %s" % aFileName)
	cmds = vehicle.commands
	missionlist = []
	with open(aFileName) as f:
		for i, line in enumerate(f):
			if i == 0:
				if not line.startswith('QGC WPL 110'):
					raise Exception('File is not supported WP version')
			else:
				linearray = line.split('\t')
				ln_index = int(linearray[0])
				ln_currentwp = int(linearray[1])
				ln_frame = int(linearray[2])
				ln_command = int(linearray[3])
				ln_param1 = float(linearray[4])
				ln_param2 = float(linearray[5])
				ln_param3 = float(linearray[6])
				ln_param4 = float(linearray[7])
				ln_param5 = float(linearray[8])
				ln_param6 = float(linearray[9])
				ln_param7 = float(linearray[10])
				ln_autocontinue = int(linearray[11].strip())
				cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2,
							  ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
				missionlist.append(cmd)
	return missionlist


def upload_mission(aFileName):
	"""
	Upload a mission from a file.
	"""
	# Read mission from file
	missionlist = readmission(aFileName)

	print("\nUpload mission from a file: %s" % aFileName)
	# Clear existing mission from vehicle
	print(' Clear mission')
	cmds = vehicle.commands
	cmds.clear()
	# Add new mission to vehicle
	for command in missionlist:
		cmds.add(command)
	print(' Upload mission')
	vehicle.commands.upload()


### istenilen noktanın koordinatlarını alan fonksiyon
def get_location_metres(original_location, dNorth, dEast):

	earth_radius=6378137.0 #Radius of "spherical" earth
	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	return LocationGlobal(newlat, newlon,original_location.alt)

### iki noktanın arasındaki mesafeyi alan fonksiyon
def get_distance_metres(aLocation1, aLocation2):

	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

### şuanki hedef notaya uzaklığı alan fonksiyon
def distance_to_current_waypoint():

	nextwaypoint = vehicle.commands.next
	if nextwaypoint==0:
		return None
	missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
	lat = missionitem.x
	lon = missionitem.y
	alt = missionitem.z
	targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
	distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
	return distancetopoint

### arm etme fonksiyonu
def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""

	print("Basic pre-arm checks")
	# Don't let the user try to arm until autopilot is ready
	while not vehicle.is_armable:
		print(" Waiting for vehicle to initialise...")
		time.sleep(1)


	print("Arming motors")
	# Copter should arm in GUIDED mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
		print(" Waiting for arming...")
		time.sleep(1)

	print("Taking off!")
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
	#  after Vehicle.simple_takeoff will execute immediately).
	while True:
		print(" Altitude: ", vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
			print("Reached target altitude")
			break
		time.sleep(1)

### arm etme ve yükseklik verme
#arm_and_takeoff(10)

görev_flag = True

"""
while True:
	if (vehicle.commands.next == 3) and görev_flag:
		sağdan()
		görev_flag = False
		import_mission_filename = 'yeni_1.txt'
		upload_mission(import_mission_filename)
"""

# Flag tanımlamaları
red_flag = True
green_flag = True
yellow_flag = True

def show_camera():
	window_title = "CSI Camera"

	# To flip the image, modify the flip_method parameter (0 and 2 are the most common)
	print(gstreamer_pipeline(flip_method=0))
	video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
	if video_capture.isOpened():
		try:
			window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

			while True:

				_,frame = video_capture.read()

				hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


				lower_red = np.array([0,100,80])
				upper_red = np.array([10,255,255])

				lower_red_iki = np.array([170,100,70])
				upper_red_iki = np.array([180,255,255])

				lower_yellow = np.array([20,100,100])
				upper_yellow = np.array([30,255,255])

				lower_green = np.array([40,70,80])
				upper_green = np.array([70,255,255])

				lower_blue = np.array([90,60,0])
				upper_blue = np.array([121,255,255])

				mask_red =  cv2.inRange(hsv, lower_red, upper_red)
				mask_red_iki =  cv2.inRange(hsv, lower_red_iki, upper_red_iki)
				mask1 = mask_red + mask_red_iki

				mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
				mask3 = cv2.inRange(hsv, lower_green, upper_green)
				mask4 = cv2.inRange(hsv, lower_blue, upper_blue)

				cnts1 = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				cnts1 = imutils.grab_contours(cnts1)

				cnts2 = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				cnts2 = imutils.grab_contours(cnts2)

				cnts3 = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				cnts3 = imutils.grab_contours(cnts3)

				cnts4 = cv2.findContours(mask4, cv2.
				RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				cnts4 = imutils.grab_contours(cnts4)

				for c in cnts1:
					alan1 = cv2.contourArea(c)
					if alan1 > 5000:

						# engelden kaçma


						cv2.drawContours(frame, [c], -1, (0,255,0), 3)

						M = cv2.moments(c)

						cx = int(M["m10"]/M["m00"])
						cy = int(M["m01"]/M["m00"])

						cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
						cv2.putText(frame, "Kirmizi", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

				for c in cnts2:
					alan2 = cv2.contourArea(c)
					if alan2 > 5000:

						# engelden kaçma
						#engelden_kacma()

						cv2.drawContours(frame, [c], -1, (0,255,0), 3)

						M = cv2.moments(c)

						cx = int(M["m10"]/M["m00"])
						cy = int(M["m01"]/M["m00"])

						cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
						cv2.putText(frame, "Sari", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

				for c in cnts3:
					alan3 = cv2.contourArea(c)
					if alan3 > 5000:

						# engelden kaçma
						#engelden_kacma()

						cv2.drawContours(frame, [c], -1, (0,255,0), 3)

						M = cv2.moments(c)

						cx = int(M["m10"]/M["m00"])
						cy = int(M["m01"]/M["m00"])

						cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
						cv2.putText(frame, "Yesil", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

				for c in cnts4:
					alan4 = cv2.contourArea(c)
					if alan4 > 5000:


						# engelden kaçma
						#engelden_kacma()

						cv2.drawContours(frame, [c], -1, (0,255,0), 3)

						M = cv2.moments(c)

						cx = int(M["m10"]/M["m00"])
						cy = int(M["m01"]/M["m00"])

						cv2.circle(frame, (cx,cy), 7, (255,255,255), -1)
						cv2.putText(frame, "Mavi", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255, 255, 255), 3)

				cv2.imshow("Renk Tespit", frame)

				if cv2.waitKey(1) == ord('q'):
					break

		finally:
			video_capture.release()
			cv2.destroyAllWindows()
	else:
		print("Error: Unable to open camera")

def sağdan():

	global red_flag

	hedef1 = get_location_metres(vehicle.location.global_frame, -2, 2)
	hedef2 = get_location_metres(vehicle.location.global_frame, 0, 4)
	hedef3 = get_location_metres(vehicle.location.global_frame, 0, 6)

	a_file = open("deneme1.txt", "r")
	list_of_lines = a_file.readlines()
	numberOfLines = len(list_of_lines)
	list_of_lines[numberOfLines-1] = str(numberOfLines+2) + list_of_lines[numberOfLines-1][1:]

	hedef1_text =  str(numberOfLines-2) +      "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef1.lat) + "\t" + str(hedef1.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines-1, hedef1_text)

	hedef2_text =  str(numberOfLines-1) +      "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef2.lat) + "\t" + str(hedef2.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines, hedef2_text)

	hedef3_text =  str(numberOfLines) +    "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef3.lat) + "\t" + str(hedef3.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines+1, hedef3_text)

	a_file = open("yeni_1.txt", "w")
	a_file.writelines(list_of_lines)
	a_file.close()
	red_flag = True


def soldan():

	global green_flag
	hedef1 = get_location_metres(vehicle.location.global_frame, 2, 2)
	hedef2 = get_location_metres(vehicle.location.global_frame, 0, 4)
	hedef3 = get_location_metres(vehicle.location.global_frame, 0, 6)

	a_file = open("deneme1.txt", "r")
	list_of_lines = a_file.readlines()
	numberOfLines = len(list_of_lines)
	list_of_lines[numberOfLines-1] = str(numberOfLines+2) + list_of_lines[numberOfLines-1][1:]


	hedef1_text =  str(numberOfLines-2) +      "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef1.lat) + "\t" + str(hedef1.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines-1, hedef1_text)

	hedef2_text =  str(numberOfLines-1) +      "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef2.lat) + "\t" + str(hedef2.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines, hedef2_text)

	hedef3_text =  str(numberOfLines) +    "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef3.lat) + "\t" + str(hedef3.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines+1, hedef3_text)

	a_file = open("yeni_1.txt", "w")
	a_file.writelines(list_of_lines)
	a_file.close()

	green_flag = True

### soldan geçme
def sarı_dönüs():

	global yellow_flag
	hedef1 = get_location_metres(vehicle.location.global_frame, 2, 2)
	hedef2 = get_location_metres(vehicle.location.global_frame, 0, 4)
	hedef3 = get_location_metres(vehicle.location.global_frame, 0, 6)

	a_file = open("deneme1.txt", "r")
	list_of_lines = a_file.readlines()
	numberOfLines = len(list_of_lines)
	list_of_lines[numberOfLines-1] = str(numberOfLines+2) + list_of_lines[numberOfLines-1][1:]


	hedef1_text =  str(numberOfLines-2) +      "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef1.lat) + "\t" + str(hedef1.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines-1, hedef1_text)

	hedef2_text =  str(numberOfLines-1) +      "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef2.lat) + "\t" + str(hedef2.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines, hedef2_text)

	hedef3_text =  str(numberOfLines) +    "	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	" + str(hedef3.lat) + "\t" + str(hedef3.lon) + "\t" + "20.000000	1 \n"
	list_of_lines.insert(numberOfLines+1, hedef3_text)

	a_file = open("yeni_1.txt", "w")
	a_file.writelines(list_of_lines)
	a_file.close()

	yellow_flag = True

if __name__ == "__main__":
	show_camera()