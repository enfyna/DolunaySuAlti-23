from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from math import sin, cos, sqrt, atan2, radians
from pymavlink import mavutil
import numpy as np
import argparse
import socket
import time

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)
Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.
Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(
	description='Commands vehicle using vehicle.simple_goto.')

#parser.add_argument('--connect',
#  	 default='tcp:127.0.0.1:5762')

parser.add_argument('--connect',
	default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00')

args = parser.parse_args()
connection_string = args.connect

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""
	print("Basic pre-arm checks")
	# Don't try to arm until autopilot is ready
	while not vehicle.is_armable:
		print(" Waiting for vehicle to initialise...")
		time.sleep(1)

	print("Arming motors")
	# Copter should arm in GUIDED mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	# Confirm vehicle armed before attempting to take off
	while not vehicle.armed:
		print(" Waiting for arming...")
		time.sleep(1)

	print("Taking off!")
	#vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

	# Wait until the vehicle reaches a safe height before processing the goto
	#  (otherwise the command after Vehicle.simple_takeoff will execute
	#   immediately).
	while True:
		print(" Altitude: ", vehicle.location.global_relative_frame.alt)
		# Break and return from function just below target altitude.
		if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
			print("Reached target altitude")
			break
		time.sleep(1)

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

def get_location_metres(original_location, dNorth, dEast):
	earth_radius = 6378137.0  # Radius of "spherical" earth
	# Coordinate offsets in radians
	dLat = dNorth / earth_radius
	dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

	# New position in decimal degrees
	newlat = original_location.lat + (dLat * 180 / math.pi)
	newlon = original_location.lon + (dLon * 180 / math.pi)
	# newlat  = latitude  + (dy / r_earth) * (180 / pi);
	# newlon = longitude + (dx / r_earth) * (180 / pi) / cos(dLat * pi/180);
	a = LocationGlobal(newlat, newlon, original_location.alt)
	# print("location",a)
	return a

#arm_and_takeoff(10)

print("Set default/target airspeed to 3")
#vehicle.airspeed = 3
#vehicle.mode = VehicleMode("AUTO")
while(1):
	print("Going towards first point  ...")
	print("waypoint : ", vehicle.commands.next)
	point2 = LocationGlobalRelative(40.201282, 29.1105516, 20)
	vehicle.simple_goto(point2, groundspeed=10)
	#time.sleep(30)
	#point1 = LocationGlobalRelative(40.2310694, 29.0104079, 20)
	#vehicle.simple_goto(point1)

# sleep so we can see the change in map
#time.sleep(30)

while(1):
	print("Going towards second point for 30 seconds ...")
	print("waypoint : ", vehicle.commands.next)
	point1 = LocationGlobalRelative(40.2310694, 29.0104079, 20)
	vehicle.simple_goto(point1)
	#point1 = get_location_metres(vehicle.location.global_frame, -7 , -7)
	#print(point1)

# sleep so we can see the change in map
time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()