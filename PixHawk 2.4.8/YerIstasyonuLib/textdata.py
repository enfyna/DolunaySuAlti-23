from pymavlink import mavutil
import time

def writedata(master):
    lines = data(master)+basinc(master)+motor(master)+mod(master)
    with open('data.txt', 'w') as f:
        for line in lines:
            f.write(f"{line} ")
        f.write("\n")


def data(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     30,0,0,0,0,0,0)
    attitude = master.recv_match(type='ATTITUDE', blocking=True)
    yaw = attitude.yaw
    roll = attitude.roll
    pitch = attitude.pitch
    data = [yaw,roll,pitch]
    return data
    
def basinc(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     29,0,0,0,0,0,0)
    pressure = master.recv_match(type='SCALED_PRESSURE', blocking=True)
    pressure_abs = pressure.press_abs
    pressure_diff = pressure.press_diff
    temperature = pressure.temperature
    # Depth (m) = ((P0/ρ) - P) / g
    p0 = 1013.25 
    p = 1000.0
    P=pressure_abs/1000.0
    g = 9.83
    pressuremeter = ((p0/p) - P)/g
    data = [pressuremeter]
    return data
    
def motor(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     36,0,0,0,0,0,0)
    engine = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
    # The SERVO_OUTPUT_RAW message contains data for 8 servo/engine outputs
    # You can access the data for each output using the following indices:
    # engine.servo1_raw, engine.servo2_raw, engine.servo3_raw, engine.servo4_raw
    # engine.servo5_raw, engine.servo6_raw, engine.servo7_raw, engine.servo8_raw
    data =[engine.servo1_raw,engine.servo2_raw,engine.servo3_raw,engine.servo4_raw,engine.servo5_raw,
           engine.servo6_raw,engine.servo7_raw,engine.servo8_raw]
    return data
def decode_mode(base_mode, custom_mode):
    """Decode flight mode from base_mode and custom_mode fields of HEARTBEAT message"""
    mode = 'UNKNOWN'
    if base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        if custom_mode == 0:
            mode = 'STABILIZE'
        elif custom_mode == 1:
            mode = 'ACRO'
        elif custom_mode == 2:
            mode = 'ALT_HOLD'
        elif custom_mode == 3:
            mode = 'AUTO'
        elif custom_mode == 4:
            mode = 'GUIDED'
        elif custom_mode == 5:
            mode = 'LOITER'
        elif custom_mode == 6:
            mode = 'RTL'
        elif custom_mode == 7:
            mode = 'CIRCLE'
        elif custom_mode == 8:
            mode = 'POSITION'
        elif custom_mode == 9:
            mode = 'LAND'
        elif custom_mode == 10:
            mode = 'OF_LOITER'
        elif custom_mode == 11:
            mode = 'DRIFT'
        elif custom_mode == 13:
            mode = 'SPORT'
        elif custom_mode == 14:
            mode = 'FLIP'
        elif custom_mode == 15:
            mode = 'AUTOTUNE'
        elif custom_mode == 16:
            mode = 'POSHOLD'
        elif custom_mode == 17:
            mode = 'BRAKE'
        elif custom_mode == 18:
            mode = 'THROW'
        elif custom_mode == 19:
            mode = 'MANUAL'
        elif custom_mode == 20:
            mode = 'GUIDED_NOGPS'
    return mode
def mod(master):
    master.mav.command_long_send(master.target_system,master.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                     0,
                                     0,0,0,0,0,0,0)
    mode = master.recv_match(type='HEARTBEAT', blocking=True)
    mode_name = decode_mode(mode.base_mode, mode.custom_mode)
    armed = mode.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    armedtext = ""
    if armed ==128:
        armedtext = "ARMED"
    else:
        armedtext = "DISARM"
    data=[mode_name,armedtext]
    return data
