from pymavlink import mavutil
import time
import cv2
import logger as log
import resim
cap = cv2.VideoCapture(0)
def mod(mode):
    print("degistir : ",mode)
    log.yaz(mode + " Degistirldi.")
    if mode not in master.mode_mapping():
        print("bilinmeyen mod : {}".format(mode))
    print(master.mode_mapping())
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             mode_id)
    msg1 = master.recv_match(type="COMMAND_ACK",blocking=True).to_dict()
    log.yaz(msg1)
    print(master.recv_match(type="HEARTBEAT",blocking = True).to_dict())
    return
def kamera():
    try:   
        for i in range(0,5):
            _,frame = cap.read()
            time.sleep(1)
            resim.kaydet(frame)
            
    except Exception as e:
        log.yaz("HATA : "+str(type(e)) + " " + str(e.args))
def servo():
    #for i in range(5):
        master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     36,0,0,0,0,0,0)
        msg = master.recv_match(type="SERVO_OUTPUT_RAW",blocking=True)
        log.yaz(str(msg))

def armanddisarm(i):
    master.mav.command_long_send(master.target_system,master.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0,
                             i,0,0,0,0,0,0)
    if i == 0:
        master.motors_disarmed_wait()
        print("Disarm")
        log.yaz("Disarm edildi.")
    else:
        master.motors_armed_wait()
        print("Arm")
        log.yaz("Arm edildi.")
try:
    kamera()
    log.basla()
    try: 
        master = mavutil.mavlink_connection("/dev/ttyACM0")
    except:
        master = mavutil.mavlink_connection("/dev/ttyACM1")
    master.wait_heartbeat()
    armanddisarm(1)
    time.sleep(3)
    #mod("STABILIZE")
    #servo()
    #time.sleep(5)
    servo()
    mod("ACRO")
    try:
        master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,0,20,0,1,0,0,0)
        msg1 = master.recv_match(type="COMMAND_ACK",blocking=True).to_dict()
        log.yaz(str(msg1))
    except:
        log.yaz("HATA")
        
    servo()
    kamera()
    time.sleep(5)
    master.mav.command_long_send(master.target_system,master.target_component,
                                 mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,90,15,1,1,0,0,0)
    msg1 = master.recv_match(type="COMMAND_ACK",blocking=True).to_dict()
    log.yaz(str(msg1))
    servo()
    time.sleep(5)
    master.mav.command_long_send(master.target_system,master.target_component,
                                 mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,-90,15,-1,1,0,0,0)
    msg1 = master.recv_match(type="COMMAND_ACK",blocking=True).to_dict()
    log.yaz(str(msg1))
    servo()
    time.sleep(5)
    armanddisarm(0)
   # servo()
#     while True:
#         master.set_servo(16,1700)
        #master.mav.command_long_send(master.target_system,master.target_component,
                                     #mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                     #0,
                                     #16,1700,0,0,0,0,0)
#         print(master.recv_match(type="COMMAND_ACK",blocking=True).to_dict())
#         servo()
#         time.sleep(0.1)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             armanddisarm(0)
#             break
except KeyboardInterrupt:
    armanddisarm(0)
except Exception as i:
    log.yaz("HATA : " + str(type(i)))
    armanddisarm(0)
armanddisarm(0)

cap.release()