from pymavlink import mavutil
import sys

# Pixhawk ile bağlantı kur
master = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00')

# Heartbeat mesajını bekle
master.wait_heartbeat()

# Pixhawkı geçirmek istediğin modu seç 
mode = 'ACRO'

# Pixhawkta bu mod var mı diye kontrol et
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Mod ID'sini al
mode_id = master.mode_mapping()[mode]

# Mod değiştirme için bir kaç tane fonksiyon var istediğini kullanabilirsin

# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0
# )

# master.set_mode(mode_id)

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
    )

while True:
    # ACK komutunu bekle
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Set mode fonksiyonunun ACK komutunu almış mıyız diye kontrol et
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # ACK mesajını yazdır
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break