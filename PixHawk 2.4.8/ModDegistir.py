from pymavlink import mavutil
import time

def mod(mode):
    print("degistir : ",mode)
    if mode not in master.mode_mapping():
        print("bilinmeyen mod : {}".format(mode))
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             mode_id)
    print(master.recv_match(type="COMMAND_ACK",blocking=True).to_dict())
    print(master.recv_match(type="HEARTBEAT",blocking = True).to_dict())
    return

def servo():
    #while True:
    for i in range(5):    
        master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     36,0,0,0,0,0,0)
        msg = master.recv_match(type="SERVO_OUTPUT_RAW",blocking=True)
        print(msg)
        time.sleep(0.1)
def armanddisarm(i):
    master.mav.command_long_send(master.target_system,master.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0,
                             i,0,0,0,0,0,0)
    if i == 0:
        master.motors_disarmed_wait()
        print("Disarm")
    else:
        master.motors_armed_wait()
        print("Arm")
        
master = mavutil.mavlink_connection("/dev/ttyACM0")
master.wait_heartbeat()
armanddisarm(1)


mod("STABILIZE")
time.sleep(0.1)
servo()
mod("MANUAL")
time.sleep(0.1)
servo()
mod("STABILIZE")
time.sleep(0.1)
servo()
mod("MANUAL")
time.sleep(0.1)
servo()

armanddisarm(0)
