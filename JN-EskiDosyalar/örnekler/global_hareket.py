from pymavlink import mavutil

connection = mavutil.mavlink_connection("/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00")

connection.wait_heartbeat()

latitude = 10
longtitude = 10

connection.mav.send(
	mavutil.mavlink.MAVlink_set_position_target_global_int_message(
		10,
		connection.target_system,
		connection.target_component,
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
		int(0b110111111000),
		int(latitude * 10 ** 7),
		int(longtitude * 10 ** 7),
		10, 0, 0, 0, 0, 0, 0, 0, 0
		)
	)

# NOT: son iki parametre yaw ile alakalı