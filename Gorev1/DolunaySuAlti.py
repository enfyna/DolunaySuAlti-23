from pymavlink import mavutil
import sys

class Dolunay():
    def __init__(self,baglanti_modu):
        self.master = self.baglan(baglanti_modu)
        self.buttons = 1 + 1 << 3 + 1 << 7
        pass
    
    def hareket_et(self,x,y,z,r):
        """
       
        x = 0 # 1 ileri      ,-1 geri
        y = 0 # 1 saga git   ,-1 sola git 
        z = 0 # 1 yukari     ,-1 asagi
        r = 0 # 1 saga cevir ,-1 sola cevir
        """
        x = sorted([-1000,x,1000])[1]
        y = sorted([-1000,y,1000])[1]
        z = sorted([0,z+500,1000])[1]
        r = sorted([-1000,r,1000])[1]
        
        self.master.mav.manual_control_send(
            self.master.target_system,
            x,
            y,
            z,
            r,
            self.buttons
            )  

    def get_yaw_roll_pitch(self):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
                                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                        0,
                                        30,0,0,0,0,0,0)
        attitude = master.recv_match(type='ATTITUDE', blocking=True)
        yaw = attitude.yaw
        roll = attitude.roll
        pitch = attitude.pitch
        return [yaw,roll,pitch]
        
    def get_pressure(self):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
                                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                        0,
                                        29,0,0,0,0,0,0)
        pressure = self.master.recv_match(type='SCALED_PRESSURE', blocking=True)
        pressure_abs = pressure.press_abs
        pressure_diff = pressure.press_diff
        temperature = pressure.temperature
        # Depth (m) = ((P0/ρ) - P) / g
        p0 = 1013.25 
        p = 1000.0
        P = pressure_abs/1000.0
        g = 9.83
        pressuremeter = ((p0/p) - P)/g
        return [pressuremeter]
        
    def get_servo(self):
        """
        The SERVO_OUTPUT_RAW message contains data for 8 servo/engine outputs
        You can access the data for each output using the following indices:
        engine.servo1_raw, engine.servo2_raw, engine.servo3_raw, engine.servo4_raw
        engine.servo5_raw, engine.servo6_raw, engine.servo7_raw, engine.servo8_raw
        """
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
                                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                        0,
                                        36,0,0,0,0,0,0)
        engine = self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        return [engine.servo1_raw,engine.servo2_raw,engine.servo3_raw,
                engine.servo4_raw,engine.servo5_raw,engine.servo6_raw,
                engine.servo7_raw,engine.servo8_raw]
    
    def baglan(self,baglanti_modu):
        platform = sys.platform.upper()
        print("İsletim Sistemi : " + str(platform))
        i = 0
        if baglanti_modu == "USB":  
            while True:
                if platform == "linux":
                    port = '/tty/ACM' + str(i)
                elif platform == "windows":
                    port = 'COM' + str(i)
                else:
                    raise Exception("Bilinmeyen isletim sistemi ?")
                try:
                    master = mavutil.mavlink_connection(port)
                    master.wait_heartbeat()
                    print("Arac ile baglanti kuruldu.")
                except:
                    i += 1
                    if i > 20:
                        raise Exception("Arac ile baglanti kurulamadi port bilgisinde hata var yada kabloda temassizlik olabilir.")
                    continue
        elif baglanti_modu == "SITL":
            port = 'udp:127.0.0.1:14550'
            try:
                master = mavutil.mavlink_connection(port)
                master.wait_heartbeat()
                print("SITL ile baglanti kuruldu.")
            except:
                raise Exception("SITL ile bağlanti kurulamadi port bilgisini kontrol edin.")
        return master
    
    def set_arm(self,i):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            i,0,0,0,0,0,0)
        if i == 0:
            self.master.motors_disarmed_wait()
            print("-> Disarm")
        else:
            self.master.motors_armed_wait()
            print("-> Arm")
        return

    def set_mod(self,mode):
        mode = mode.upper()
        if mode not in self.master.mode_mapping():
            print("'{}' modu bulunamadi.Yazim hatasina dikkat et.\nSTABILIZE moda gecilecek.".format(mode))
            mode = "STABILIZE"

        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(self.master.target_system,
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                mode_id)
        heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True)
        mode = list(self.master.mode_mapping().keys())[list(self.master.mode_mapping().values()).index(heartbeat.custom_mode)]
        print("Mod -> ",mode)
        return
    def kapat(self):
        self.master.close()
