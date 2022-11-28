from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00")

the_connection.wait_heartbeat()
print("sistem: %u  companent: %u " % (the_connection.target_system, the_connection.target_component))


# ileri hareket için 
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b1101111111000),15,0,0,0,0,0,0,0,0,1.57,0))

 

# konumunu takip etme
while True:
    msg = the_connection.recv_match(type= 'LOCAL_POSITION_NED', blocking = True)
    print(msg)

