from pymavlink import mavutil
import sys,time,math
# import textdata

class Dolunay():
	def __init__(self, baglanti_modu):
		def baglan():
			platform = sys.platform.lower()
			print("İsletim Sistemi : " + str(platform))
			i = 0
			if baglanti_modu == "USB":
				while True:
					if platform == "linux":
						port = 'dev/ttyACM' + str(i)
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
							raise Exception(
								"Arac ile baglanti kurulamadi.Port bilgisinde hata var yada kablo duzgun takilmamis olabilir.")
						continue
			elif baglanti_modu == "SITL":
				port = 'udp:127.0.0.1:14550'
				try:
					master = mavutil.mavlink_connection(port)
					master.wait_heartbeat()
					print("SITL ile baglanti kuruldu.")
				except:
					raise Exception(
						"SITL ile bağlanti kurulamadi port bilgisini kontrol edin.")
			else:
				raise Exception(
					"Araca USB ya da SITL ile baglanmadan hareket komutlarini calistiramazsiniz.")
			return master
		self.master = baglan()
		# self.buttons = 1 + 1 << 3 + 1 << 7
		self.dx = 1500
		self.dy = 1500
		self.dz = 1500
		self.dr = 1500
		self.do = 1500
		self.dp = 1500
		self.lpp = 0.3
		self.lpr = 0.3

		#PID katsayilari
		self.kp_roll = 0
		self.ki_roll = 0
		self.kd_roll = 0

		self.kp_pitch = 0
		self.ki_pitch = 0
		self.kd_pitch = 0
		#-----------------------------
		#PID hesaplamalında kullanılacak degiskenler
		#Degerler degistirilmemeli
		self.integral_roll = 0
		self.integral_pitch = 0
		self.last_error_roll = 0
		self.last_error_pitch = 0

		self.dt = 1
		self.time = 0
		self.last_boot_time = 0
		#-----------------------------

		pass

	def hareket_et(self, x, y, z, r):
		"""
		x = 0 # 1 ileri      ,-1 geri
		y = 0 # 1 saga git   ,-1 sola git
		z = 0 # 1 yukari     ,-1 asagi
		r = 0 # 1 saga cevir ,-1 sola cevir
		Eger hareket icin bu fonksiyon kullanilacaksa baslangic degerleri 0'a ayarlanmali.
		Ayni kodda hem hareket_et() ve hareket_et_v2() kullanilmamali.1 tanesi secilmeli.
		Ornek :
		init():
			self.dx = 0
		"""
		def lerp(durum, hedef, agirlik):
			"""
			Durum sayisini hedefe dogru agirlik orani ile yaklastir.
			-> Agirlik orani 1 den kucuk olmali.
			Ornek :
			lerp( 0,100,0.5) == 50
			lerp(50,100,0.5) == 75
			"""
			return round((1 - agirlik) * durum + agirlik * hedef)

		x = sorted([-10000, int(x), 10000])[1]
		y = sorted([-10000, int(y), 10000])[1]
		z = sorted([-10000, int(z+500), 10000])[1]
		r = sorted([-10000, int(r), 10000])[1]

		self.dx = lerp(self.dx, x, 0.1)
		self.dy = lerp(self.dy, y, 0.1)
		self.dz = lerp(self.dz, z, 0.1)
		self.dr = lerp(self.dr, r, 0.1)

		self.master.mav.manual_control_send(
			self.master.target_system,
			self.dx,
			self.dy,
			self.dz,
			self.dr,
			0  # Buton parametresi eger butona basılacaksa buton numarasina denk gelen bit degeri gonderilmeli
		)
		return

	def hareket_et_v2(self, set_forward, set_lateral, set_throttle, set_yaw, set_roll_deg, set_pitch_deg):
		"""
		@set_forward  : "+" deger ileri         , "-" deger geri
		@set_lateral  : "+" deger saga oteleme  , "-" deger sola oteleme
		@set_throttle : "+" deger yukari        , "-" deger asagi
		@set_yaw      : "+" deger saga cevirme  , "-" deger sola cevirme
		@set_roll     : İstenilen roll acisi derece cinsinden verilmeli
		@set_pitch    : İstenilen pitch acisi derece cinsinden verilmeli
		Aracin dengesini saglamak icin roll ve pitch 0 verilmeli.
		Ayni kodda hem hareket_et() ve hareket_et_v2() kullanilmamali.1 tanesi secilmeli.
		Eger hareket icin bu fonksiyon kullanilacaksa baslangic degerleri 1500'e ayarlanmali.
		Ornek :
		init():
			self.dx = 1500
		"""
		def lerp(durum, hedef, agirlik):
			"""
			Durum sayisini hedefe dogru agirlik orani ile yaklastir.
			-> Agirlik orani 1 den kucuk olmali.
			Ornek :
			lerp( 0,100,0.5) == 50
			lerp(50,100,0.5) == 75
			"""
			return round((1 - agirlik) * durum + agirlik * hedef)

		#Aracin dengesini korumaya calis
		attitude = self.get_yaw_roll_pitch_deg() # Attitude bilgisini al
		if attitude is not None: # None durumunu kontrol et
			roll, rollspeed, pitch, pitchspeed = attitude[2], attitude[3], attitude[4], attitude[5] # Roll,Pitch bilgilerini al

			print("Set_Roll : {},Roll : {},Dif: {}".format(set_roll_deg,roll,set_roll_deg - roll))
			print("Set_Pitch : {},Pitch : {},Dif : {}".format(set_pitch_deg,pitch,set_pitch_deg-pitch))

			set_roll = (set_roll_deg - roll) * 5
			set_pitch = (set_pitch_deg - pitch) * 5

			print("Roll Motor : {}".format(set_roll+1500))
			print("Pitch Motor : {}".format(set_pitch))

			set_roll     = sorted([1100, int(set_roll    + 1500), 1900])[1]
			set_pitch    = sorted([1100, int(set_pitch   + 1500), 1900])[1]

			self.do = lerp(self.do, set_roll,     self.lpr)
			self.dp = lerp(self.dp, set_pitch,    self.lpp)
		else:
			#Attitude bilgisi alinamazsa roll ve pitch motorlarini durdur
			self.do = self.dp = 1500

		set_forward  = sorted([1100, int(set_forward + 1500), 1900])[1]
		set_lateral  = sorted([1100, int(set_lateral + 1500), 1900])[1]
		set_throttle = sorted([1100, int(set_throttle+ 1500), 1900])[1]
		set_yaw      = sorted([1100, int(set_yaw     + 1500), 1900])[1]

		self.dx = lerp(self.dx, set_forward,  0.1)
		self.dy = lerp(self.dy, set_lateral,  0.1)
		self.dz = lerp(self.dz, set_throttle, 0.1)
		self.dr = lerp(self.dr, set_yaw,      0.1)

		'''
		Degerler 1100-1900 arasinda olmali.1500 notr deger.
		Eger kullanilmayacaksa 65535 olarak birakilmali.
		Kaynak:
			https://gist.github.com/ES-Alexander/ee1dd479dd728b7cef1bf936f9114e71
		'''
		self.master.mav.rc_channels_override_send(
			self.master.target_system, self.master.target_component,
			self.dp, # Pitch
			self.do, # Roll
			self.dz, # Throttle
			self.dr, # Yaw
			self.dx, # Forward
			self.dy, # Lateral
			65535,   # Camera_pan
			65535,   # Camera_tilt
			65535,   # Lights1
			65535,   # Lights2
			65535,   # Video_switch
			65535, 65535, 65535, 65535, 65535, 65535, 65535) # Undefined (QGC kullanılarak bir fonksiyon atanabilinir.)
		return

	def hareket_et_PID(self, set_forward, set_lateral, set_throttle, set_yaw, set_roll_deg, set_pitch_deg):
		"""
		@set_forward  : "+" deger ileri         , "-" deger geri
		@set_lateral  : "+" deger saga oteleme  , "-" deger sola oteleme
		@set_throttle : "+" deger yukari        , "-" deger asagi
		@set_yaw      : "+" deger saga cevirme  , "-" deger sola cevirme
		@set_roll     : İstenilen roll acisi derece cinsinden verilmeli
		@set_pitch    : İstenilen pitch acisi derece cinsinden verilmeli
		Aracin dengesini saglamak icin roll ve pitch 0 verilmeli.
		Ayni kodda hem hareket_et() ve hareket_et_v2() kullanilmamali.1 tanesi secilmeli.
		Eger hareket icin bu fonksiyon kullanilacaksa baslangic degerleri 1500'e ayarlanmali.
		Ornek :
		init():
			self.dx = 1500
		"""
		def lerp(durum, hedef, agirlik):
			"""
			Durum sayisini hedefe dogru agirlik orani ile yaklastir.
			-> Agirlik orani 1 den kucuk olmali.
			Ornek :
			lerp( 0,100,0.5) == 50
			lerp(50,100,0.5) == 75
			"""
			return round((1 - agirlik) * durum + agirlik * hedef)

		#Aracin dengesini korumaya calis
		attitude = self.get_yaw_roll_pitch_deg() # Attitude bilgisini al
		if attitude is not None: # None durumunu kontrol et
			_, _, roll, rollspeed, pitch, pitchspeed ,time_boot_ms = attitude # Roll,Pitch bilgilerini al

			#PID hesaplamalarını yapabilmek icin dt yi hesapla
			if time_boot_ms > self.last_boot_time:#Eger pixhawktan guncel zaman gelmediyse hesaplama
				self.last_boot_time = time_boot_ms
				self.dt = self.last_boot_time - self.time

			#Roll icin PID hesapla
			error_roll = (set_roll_deg - roll)                           #P
			self.integral_roll += error_roll * self.dt                   #I
			derivative =  (error_roll - self.last_error_roll) / self.dt  #D
			self.last_error_roll = error_roll
			self.do = round(self.kp_roll * error_roll + self.ki_roll * self.integral_roll + self.kd_roll * derivative)+1500


			#Pitch icin PID hesapla
			error_pitch = (set_pitch_deg - roll)                          #P
			self.integral_pitch += error_pitch * self.dt                  #I
			derivative =  (error_pitch - self.last_error_pitch) / self.dt #D
			self.last_error_pitch = error_pitch
			self.dp = round(self.kp_pitch * error_pitch + self.ki_pitch * self.integral_pitch + self.kd_pitch * derivative)+1500

			#Zamani kaydet
			self.time = self.last_boot_time
		else:
			#Attitude bilgisi alinamazsa roll,pitch motorlarini ayni seklilde cailstirmaya devam et
			#self.do degismeyecegi icin bir onceki hesaplamadan calismaya devam edecek
			pass

		set_forward  = sorted([1100, int(set_forward + 1500), 1900])[1]
		set_lateral  = sorted([1100, int(set_lateral + 1500), 1900])[1]
		set_throttle = sorted([1100, int(set_throttle+ 1500), 1900])[1]
		set_yaw      = sorted([1100, int(set_yaw     + 1500), 1900])[1]

		self.do    = sorted([1100, int(self.do    + 1500), 1900])[1]
		self.dp    = sorted([1100, int(self.dp   + 1500), 1900])[1]

		self.dx = lerp(self.dx, set_forward,  0.1)
		self.dy = lerp(self.dy, set_lateral,  0.1)
		self.dz = lerp(self.dz, set_throttle, 0.1)
		self.dr = lerp(self.dr, set_yaw,      0.1)

		'''
		Degerler 1100-1900 arasinda olmali.1500 notr deger.
		Eger kullanilmayacaksa 65535 olarak birakilmali.
		Kaynak:
			https://gist.github.com/ES-Alexander/ee1dd479dd728b7cef1bf936f9114e71
		'''
		self.master.mav.rc_channels_override_send(
			self.master.target_system, self.master.target_component,
			self.dp, # Pitch
			self.do, # Roll
			self.dz, # Throttle
			self.dr, # Yaw
			self.dx, # Forward
			self.dy, # Lateral
			65535,   # Camera_pan
			65535,   # Camera_tilt
			65535,   # Lights1
			65535,   # Lights2
			65535,   # Video_switch
			65535, 65535, 65535, 65535, 65535, 65535, 65535) # Undefined (QGC kullanılarak bir fonksiyon atanabilinir.)
		return

	def yunusbaligi(self):
		self.hareket_et(400, 0, 0, 50)
		time.sleep(3)
		self.hareket_et(400, 0, 0, 25)
		time.sleep(3)
		self.hareket_et(0, 0, 0, 0)
		time.sleep(10)

	def get_yaw_roll_pitch_rad(self):
		"""
		Sirasiyla yaw roll pitch degerlerini radyan cinsinden verir.
		"""
		self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
										  mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
										  0,
										  30, 0, 0, 0, 0, 0, 0)
		attitude = self.master.recv_match(type='ATTITUDE', blocking=True)
		return [attitude.yaw, attitude.roll, attitude.pitch]

	def get_yaw_roll_pitch_deg(self):
		"""
		Sirasiyla yaw, yaw hizi, roll, roll hizi, pitch, pitch hizi degerlerini derece ve derece/saniye cinsinden verir.
		"""
		self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
										  mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
										  0,
										  30, 0, 0, 0, 0, 0, 0)
		attitude = self.master.recv_match(type='ATTITUDE', blocking=True)
		return [math.degrees(attitude.yaw),math.degrees(attitude.yawspeed),
				math.degrees(attitude.roll),math.degrees(attitude.rollspeed),
				math.degrees(attitude.pitch),math.degrees(attitude.pitchspeed)]

	def get_pressure(self):
		self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
										  mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
										  0,
										  29, 0, 0, 0, 0, 0, 0)
		pressure = self.master.recv_match(
			type='SCALED_PRESSURE', blocking=True)
		pressure_abs = float(pressure.press_abs)
		pressure_diff = pressure.press_diff
		temperature = pressure.temperature
		# Depth (m) = ((P0/ρ) - P) / g
		p0 = 1013.25
		p = 1000.0
		P = pressure_abs/1000.0
		g = 9.83  # -> NOT : Arac yukari yada asagiya giderkenki ivmesiyle toplanmasi lazim galiba denemek lazim
		pressuremeter = ((p0/p) - P)/g
		pressuremeter2 = pressure_abs / (9.8 * 997.0)
		return pressure_abs, pressuremeter*100, pressuremeter2

	def get_servo(self):
		"""
		The SERVO_OUTPUT_RAW message contains data for 8 servo/engine outputs
		You can access the data for each output using the following indices:
		engine.servo1_raw, engine.servo2_raw, engine.servo3_raw, engine.servo4_raw
		engine.servo5_raw, engine.servo6_raw, engine.servo7_raw, engine.servo8_raw
		"""
		self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
										  mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
										  0,
										  36, 0, 0, 0, 0, 0, 0)
		engine = self.master.recv_match(
			type='SERVO_OUTPUT_RAW', blocking=True).to_dict()
		return [engine[f'servo{i+1}_raw'] for i in range(8)]

	def set_arm(self, i):
		self.master.mav.command_long_send(
			self.master.target_system,
			self.master.target_component,
			mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
			0,
			i, 0, 0, 0, 0, 0, 0)
		if i == 0:
			self.master.motors_disarmed_wait()
			print("-> Disarm")
		else:
			self.master.motors_armed_wait()
			print("-> Arm")
		return

	def set_mod(self, mode):
		mode = mode.upper()
		if mode not in self.master.mode_mapping():
			print(
				"'{}' modu bulunamadi.Yazim hatasina dikkat et.\nSTABILIZE moda gecilecek.".format(mode))
			mode = "STABILIZE"

		mode_id = self.master.mode_mapping()[mode]
		self.master.mav.set_mode_send(self.master.target_system,
									  mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
									  mode_id)
		heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True)
		mode = list(self.master.mode_mapping().keys())[list(self.master.mode_mapping().values()).index(heartbeat.custom_mode)]
		print("Mod -> ", mode)
		return

	def get_heartbeat(self):
		return self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

	def get_battery_percentage(self):
		return self.master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1).battery_remaining

	def get_attitude_fast(self):
		return self.master.recv_match(type='ATTITUDE', blocking=False, timeout=0.5)

	def get_message(self):
		return self.master.messages

	def kapat(self):
		self.master.close()
		print("-> Baglanti kapatildi.")
