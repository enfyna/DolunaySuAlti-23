from pymavlink import mavutil
import time
import cv2

def mod(mode):
    print("degistir : ",mode)
    if mode not in master.mode_mapping():
        print("bilinmeyen mod : {}".format(mode))
    print(master.mode_mapping())
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             mode_id)
    print(master.recv_match(type="COMMAND_ACK",blocking=True).to_dict())
    print(master.recv_match(type="HEARTBEAT",blocking = True).to_dict())
    return

def servo():
    #for i in range(5):
        master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     36,0,0,0,0,0,0)
        msg = master.recv_match(type="SERVO_OUTPUT_RAW",blocking=True)
        print(msg)
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
try:
    master = mavutil.mavlink_connection("/dev/ttyACM0")
    master.wait_heartbeat()
    armanddisarm(1)
    
    #mod("STABILIZE")
    #servo()
    #time.sleep(5)
    
    mod("STABILIZE")
    time.sleep(0.1)
    servo()
    while True:
        master.set_servo(16,1700)
        #master.mav.command_long_send(master.target_system,master.target_component,
                                     #mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                     #0,
                                     #16,1700,0,0,0,0,0)
        print(master.recv_match(type="COMMAND_ACK",blocking=True).to_dict())
        servo()
        time.sleep(0.1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            armanddisarm(0)
            break
except KeyboardInterrupt:
    armanddisarm(0)
except:
    armanddisarm(0)
armanddisarm(0)
