from pymavlink import mavutil


def data(master):
    attitude = master.recv_match(type='ATTITUDE', blocking=True)
    yaw = attitude.yaw
    roll = attitude.roll
    pitch = attitude.pitch
    
    print("Yaw: ", yaw)
    print("Roll: ", roll)
    print("Pitch: ", pitch)
    
def basinc(master):
    pressure = master.recv_match(type='SCALED_PRESSURE', blocking=True)
    pressure_abs = pressure.press_abs
    pressure_diff = pressure.press_diff
    temperature = pressure.temperature
    
    print("Absolute Pressure: ", pressure_abs)
    print("Differential Pressure: ", pressure_diff)
    print("Temperature: ", temperature)
    
    
def motor(master):
    engine = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
    # The SERVO_OUTPUT_RAW message contains data for 8 servo/engine outputs
    # You can access the data for each output using the following indices:
    # engine.servo1_raw, engine.servo2_raw, engine.servo3_raw, engine.servo4_raw
    # engine.servo5_raw, engine.servo6_raw, engine.servo7_raw, engine.servo8_raw
    
    print("Engine 1 Output: ", engine.servo1_raw)
    print("Engine 2 Output: ", engine.servo2_raw)
    print("Engine 3 Output: ", engine.servo3_raw)
    print("Engine 4 Output: ", engine.servo4_raw)
    print("Engine 5 Output: ", engine.servo5_raw)
    print("Engine 6 Output: ", engine.servo6_raw)
    print("Engine 7 Output: ", engine.servo7_raw)
    print("Engine 8 Output: ", engine.servo8_raw)
    #and so on
    
def mod(master):
    mode = master.recv_match(type='HEARTBEAT', blocking=True)
    mode_name = master.mode_mapping()[mode.custom_mode]
    armed = mode.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    
    print("Current Mode: ", mode_name)
    print("Armed: ", armed)