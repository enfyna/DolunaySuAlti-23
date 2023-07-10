from pymavlink import mavutil

# Bağlantı kur
connection = mavutil.mavlink_connection("/dev/serial/usb-ArduPilot_Pixhawk1_3D004F000651353136343336-if00")

# Heartbeat mesajını al
connection.wait_heartbeat()

while True:

    # recv_match fonksiyonu ile pixhawktan mesaj alabiliriz.
    # type argümanı verilmezse gelen her mesajı yazdırır.

    msg = connection.recv_match(type = "ATTITUDE", blocking = True)
    print(msg)