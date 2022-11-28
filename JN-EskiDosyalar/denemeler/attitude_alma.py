from pymavlink import mavutil 

# connection starting 
connection = mavutil.mavlink_connection("/dev/serial/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00") 

# sistem haetbeat alma
connection.wait_heartbeat()
print("sistem ilk iletişim (sistem %u ,  bileşen %u) " % (connection.target_system, connection.target_component))

while True:
    
    msg = connection.recv_match(type = "ATTITUDE", blocking = True)
    print(msg)



