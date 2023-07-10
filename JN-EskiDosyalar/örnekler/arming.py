from pymavlink import mavutil

connection = mavutil.mavlink_connection("/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00")

connection.wait_heartbeat()

# Arming
# https://mavlink.io/en/messages/common.html#COMMAND_LONG
# 4 + 7 parametre alıyor
# 7 parametrenin ilki 0 - disarm, 1 - arm
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,1,0,0,0,0,0,0
    )

# arming bilgisi alma
# https://mavlink.io/en/messages/common.html#COMMAND_ACK
# 1. parametre command, 2. parametre result (0 ise onaylandı, 1 ise reddedildi)
msg = connection.recv_match(type = "COMMAND_ACK", blocking = True)
print(msg)