from pymavlink import mavutil


def yunusbaligisag(master):
    sagadon(master, 20)
    #move ve görüntü al
    sagadon(master, 12)
    #move ve görüntü al
    sagadon(master, 10)
    #move ve görüntü al
    sagadon(master, 5)
    #move ve görüntü al


def sagadon(master,yawrate):
    """ 1. parametre masterkey 
        2. parametre kac derece donecegi 
    """
    
    command = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 0, yawrate, 0, 0, 0, 0, 0)
    
    master.mav.send(command)
    
def soladon(master,yawrate):
    """ 1. parametre masterkey 
        2. parametre kac derece donecegi 
    """
    
    command = master.mav.command_long_encode(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 0, -yawrate, 0, 0, 0, 0, 0)
    
    master.mav.send(command)
    
def ilerigit(master,hiz,saniye):
    command = master.mav.command_long_encode(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
    0, # confirmation
    0, # param 1 (0 = airspeed, 1 = ground speed)
    hiz, # param 2 target speed
    1, # param 3 (1 = forward, -1 = backward)
    0, # param 4
    0, # param 5
    0, # param 6
    0, # param 7
    )
    master.mav.send(command)

def yukseklik(master,metre):
    """ 1. parametre masterkey 
        2. parametre kac metreye inecegi ya da cikacagi 
    """
    command = master.mav.command_long_encode(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_Z_ALTITUDE,
    0, # confirmation
    0, # param 1
    metre, # param 2 target altitude in meters 
    0, # param 3
    0, # param 4
    0, # param 5
    0, # param 6
    0, # param 7
    )
    master.mav.send(command)
    
def sagayönel(master,yawrate):
    command = master.mav.command_long_encode(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0, # confirmation
    yawrate, # param 1 target yaw angle in degrees
    0, # param 2
    1, # param 3  1 = absolute angle, 0 = relative angle
    0, # param 4
    0, # param 5
    0, # param 6
    0, # param 7
    )
    master.mav.send(command)

def solayönel(master,yawrate):
    command = master.mav.command_long_encode(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0, # confirmation
    -yawrate, # param 1 target yaw angle in degrees
    0, # param 2
    1, # param 3  1 = absolute angle, 0 = relative angle
    0, # param 4
    0, # param 5
    0, # param 6
    0, # param 7
    )
    master.mav.send(command)
    
def arm(master,i):
    """ 1 arm etme 0 disarm"""
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