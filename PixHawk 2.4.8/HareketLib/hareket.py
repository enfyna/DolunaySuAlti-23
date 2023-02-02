from pymavlink import mavutil
import time
import textdata

def left(master,t): 
    buttons = 1 + 1 << 3 + 1 << 7
    print(buttons)
    for i in range(0,t):
        master.mav.manual_control_send(
            master.target_system,
            0,
            -800,
            500,
            0,
            buttons
            )  
        textdata.writedata(master)
        time.sleep(1)

def right(master,t): 
    buttons = 1 + 1 << 3 + 1 << 7
    print(buttons)
    for i in range(0,t):
        master.mav.manual_control_send(
            master.target_system,
            0,
            800,
            500,
            0,
            buttons
            )  
        textdata.writedata(master)
        time.sleep(1)
       

def down(master,t): 
    buttons = 1 + 1 << 3 + 1 << 7
    for i in range(0,t):
        master.mav.manual_control_send(
            master.target_system,
            0,
            0,
            -800,
            0,
            buttons
            )
        textdata.writedata(master)
        time.sleep(1)

def up(master,t): 
    buttons = 1 + 1 << 3 + 1 << 7
    for i in range(0,t):
        master.mav.manual_control_send(
            master.target_system,
            0,
            0,
            800,
            0,
            buttons
            )
        textdata.writedata(master)
        time.sleep(1)
def backward(master,t): 
    buttons = 1 + 1 << 3 + 1 << 7
    for i in range(0,t):
        master.mav.manual_control_send(
            master.target_system,
            -800,
            0,
            500,
            0,
            buttons
            )
        
        textdata.writedata(master)
        time.sleep(1)
def forward(master,t): 
    buttons = 1 + 1 << 3 + 1 << 7
    print(buttons)
    for i in range(0,t):
        master.mav.manual_control_send(
            master.target_system,
            800,
            0,
            500,
            0,
            buttons
            )  
        textdata.writedata(master)
        time.sleep(1)
        
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
    
    master = mavutil.mavlink_connection('COM6')
    master.wait_heartbeat()
    print("BAGLANTI KURULDU")
    time.sleep(0.2)
    armanddisarm(1)
    mod('MANUAL')
    for i in range(0,10):
        textdata.writedata(master)
        time.sleep(0.1)
    time.sleep(0.2)
   #forward(master, 10) 
   #up(master,5)
   #down(master, 10)
   #right(master, 10)
   #left(master, 10)
    for i in range(0,10):
        textdata.writedata(master)
        time.sleep(0.1)
    armanddisarm(0)
    mod('STABILIZE')
    armanddisarm(1)
    time.sleep(0.2)
    for i in range(0,10):
        textdata.writedata(master)
        time.sleep(0.2)
    armanddisarm(0)
    textdata.writedata(master)
    master.close()
except KeyboardInterrupt:
    print("BAGLANTI KAPANDI")
    master.close()
except:
    print("BAGLANTI KAPANDI")
    master.close()
print("BAGLANTI KAPANDI")
master.close()