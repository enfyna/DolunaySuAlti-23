from pymavlink import mavutil

# connection starting 
connection = mavutil.mavlink_connection("/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00") 

# sistem haetbeat alma
connection.wait_heartbeat()
print("sistem ilk iletişim (sistem %u ,  bileşen %u) " % (connection.target_system, connection.target_component))


# arming
# https://mavlink.io/en/messages/common.html#COMMAND_LONG    adresinden bak 
# 4 + 7 parametre alıyor 
# 7 paremetrenin ilki 0 - disarm, 1 - arm
connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)


# arming bilgisi alma 
# çıktıyı anlamak için incele:  https://mavlink.io/en/messages/common.html#COMMAND_ACK
# 1. parmaetre commend, 2. paremetre result (0 ise oynaylandı , 1 ise reddedildi)
msg = connection.recv_match(type = "COMMAND_ACK", blocking = True)
print(msg)






