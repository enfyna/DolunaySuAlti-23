from pymavlink import mavutil
import cProfile
import numpy as np
import time
import math
import cv2
import sys
import os.path
# import textdata

class Gorev1:
	def __init__(self,baglanti_modu):

		self.baglanti_modu = baglanti_modu
		self.gorev_asamasi = 0
		self.hedef_kilit = 0
		self.anlik_kare_idx = 1
		self.agirlik = 0
		self.TKM = True # Tek Kamera Modu

		# Gorevde kullanılacaklari ayarla
			# Kamerayi al ve cozunurlugu ayarla
		if baglanti_modu == "USB":
			self.cap =  cv2.VideoCapture(0)
			self.cap_x , self.cap_y = int(cap.get(3)) , int(cap.get(4))
			self.cap_hx , self.cap_hy = self.cap_x//2 , self.cap_y//2
		elif baglanti_modu == "SITL" or baglanti_modu == "BAGLI_DEGIL":
			frame = self.frame_al_SITL()
			frame = frame.shape
			self.cap_x , self.cap_y , self.cap_hx , self.cap_hy = frame[1] , frame[0] , frame[1]//2 , frame[0]//2
		# Renk aralıgı
		self.lower_cyan = np.array([40, 0, 0])
		self.upper_cyan = np.array([100, 255, 255])
		# Erode-Dilate kernel
		self.kernel = np.ones((5, 5), np.float32)
		pass

	def gorev_asamasi_degistir(self):
		# Araçta birden fazla kamera varsa
		# Kamera geçişlerini ayarlamak için kullan

		# Gorev asamasi 0 ise 1 , 1 ise 0 yap
		self.gorev_asamasi = (self.gorev_asamasi + 1) % 2
		# Hedef kilidini sıfırla
		self.hedef_kilit = 0
		# Araca kamera bagli ise kameralari ayarla
		if self.baglanti_modu == "USB":
			self.cap.release() # Onceki kamerayi kapat
			self.cap.VideoCapture(secilen_asama) # Diger kamerayi ac
			# Kamera boyutunu ayarla
			self.cap_x , self.cap_y = int(cap.get(3)) , int(cap.get(4))
			self.cap_hx , self.cap_hy = int(self.cap_x/2) , int(self.cap_y/2)
		pass

	def calistir(self):
		# Kamera bagli ise
		if str(self.baglanti_modu) == "USB":
			_, dispframe = cap.read()
		#SITL ya da Bagli degil ise
		else:
			dispframe = self.frame_al_SITL()

		# Bulanıklastır
		#frame = cv2.GaussianBlur(dispframe, (5, 5), cv2.BORDER_DEFAULT)
		# Renkleri tersine cevir
		frame = cv2.bitwise_not(dispframe)
		# HSV'ye donustur
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		# inRange ile camgobegi rengini bul
		frame = cv2.inRange(frame, self.lower_cyan, self.upper_cyan)
		cv2.imshow("inrange",frame)
		# Erode-dilate (Suanda kullanilmiyor)
		# Kullanilmama sebebi bu islem yapildiktan sonra
		# approxPolyDP fonksiyonu karenin koselerini yumusattigi icin yuvarlak olarak goruyor
		#frame = cv2.erode(frame, kernel, iterations=2)
		#frame = cv2.dilate(frame, kernel, iterations=1)

		# contour listesini al
		contours, _ = cv2.findContours(
			frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# Contour ozelliklerini kaydedecegimiz bilinmeyenler
		anlik_max_c_alani , secilen_c , cx , cy = 0.0 , None , None , None
		# Aracin hareket eksenleri
		# x -> ileri-geri
		# y -> sag-sol oteleme
		# z -> asagi yukari
		# r -> sag-sol donme
		dx , dy , dz , dr = None , None , None , None

		for c in contours:
			approx = cv2.approxPolyDP(
				c, 0.01 * cv2.arcLength(c, True), True)
			if len(approx) < 8:# 8 koseden az olan contourlari ele
				continue
			else:
				# Moment hesabından c'nin alanını bul.
				M = cv2.moments(c)
				# Anlik karedeki piksel alani en buyuk c'yi bul
				if M['m00'] > anlik_max_c_alani:
					# Bulunan en buyuk c'yi anlik olarak kaydet
					secilen_c , scm , anlik_max_c_alani = c , M , M['m00']
					# scm -> secilen c momenti

		# Eger uygun bir c bulunursa olunan gorev asamasina gore islem yap
		if secilen_c is not None:
			# Secilen c'nin momentinden merkezini hesapla
			cx , cy = scm['m10']//scm['m00'] , scm['m01']//scm['m00']
			# Ekrana cizdir
			dispframe = self.cizgiciz(dispframe, secilen_c, cx, cy)

			if TKM:
				dx , dr , dz = cx - self.cap_hx , cy - self.cap_hy , cy - self.cap_hy
			else:
				if self.gorev_asamasi == 0:
					# Bu kisim 1.gorev asamasini ayarliyor
					# Araci en hizli bir sekilde buldugumuz c ye goturmemiz lazim

					# Merkez noktasinin kameranin orta noktasina gore nerede kaldigini hesapla
					dx , dr = cx - self.cap_hx , cy - self.cap_hy
					# dx degeri ileri gitme , dr degeri donme hizini ayarliyor

					# Eger buldugumuz c'ye yaklastiysak 2.asamaye gec
					if dr > int(self.cap_hy - self.cap_hy/5) and anlik_max_c_alani > 1000:
						# Yanlislikla 2.asamaya gecmemek icin c nin alaninin 1000 den buyuk
						# ve 10 defa ust uste gorursek gec
						self.agirlik +=1
						if self.agirlik == 5 :
							self.agirlik = 100
							self.gorev_asamasi_degistir()
							if self.baglanti_modu != "USB":
								self.anlik_kare_idx = 0
					else:
						# Eger c'ye daha yaklasmadiysak ya da alani cok kucukse agirligi azalt
						self.agirlik = max(self.agirlik -1 , 0)

				elif self.gorev_asamasi == 1:
					self.agirlik = min(self.agirlik +1 , 50) # istenilen ozelliklere sahip c gorulurse agirligi arttir
					self.hedef_kilit = 1 # Hedefi buldugumuzu ve araci kondurmaya basladigimizi kaydedelim

					# Merkez noktasinin kameranin orta noktasina gore nerede kaldigini hesapla
					dx , dy = cx - self.cap_hx , cy - self.cap_hy
					# dx degeri ileri gitme , dy degeri yatay hizi ayarliyor

					if abs(dx) < 100 and abs(dy) < 100:  #Testlerde deneyip iyi bir deger bulmak lazim
						dz = 100 # Eger hedef ortalandiysa araci indir
		else:
			if not TKM:
				if self.gorev_asamasi == 1: # Gorev asamasını kontrol et
					# Bu kisim 2.gorev asamasının ilk ve son kismini ayarliyor
					if self.hedef_kilit == 0:
						# 1. Kısım (Alt kameraya gecilen ilk an)
						# Bu kisimda araci ilerletirken alt kamera ile hedefi ariyoruz.
						# Eger belli bir sure icinde hedefi bulamazsak 1. gorev asamasina geri donmeliyiz

						dx = 100 # Hedefi goresiye kadar araci ilerletmeye devam et
						self.agirlik -= 1
						if self.agirlik == 0:
							self.gorev_asamasi_degistir()

					elif self.hedef_kilit == 1:
						# 2. Kısım (Hedefin bulundugu ve ustune konmaya baslanilan an)
						# Hedefi bulup ustune konmaya basladıgımızda hedefe cok yaklastıgımızda
						# hedef kameranin tamamini kaplayacagi ya da golge olusması sebebiyle
						# istenilen bir c bulunamayabilir.Eger hedef_kilit degiskenimiz 1 ise
						# hedef ile arac arasinda cok az bir mesafe kaldigini bildigimizden
						# dz degiskenini arttirarak hedefe konmasini ve orada kalmasini saglayabiliriz.
						dz = 200

		cv2.imshow("Renk Tespit", dispframe)
		cv2.waitKey(2)
		print(str(dx),"-",str(dy),"-",str(dz),"-",str(dr))
		return dx , dy , dz , dr

	def frame_al_SITL(self):
		yol = "/home/oem/Documents/Kod/Python/DolunaySuAlti-23/Gorev1/Frame_TK/"
		dosya = str(yol)+((str(self.anlik_kare_idx)+".png").zfill(8))
		if os.path.isfile(dosya):
			frame = cv2.imread(dosya)
			self.anlik_kare_idx+=1
		else:
			self.anlik_kare_idx = 1
			dosya = str(yol)+((str(self.anlik_kare_idx)+".png").zfill(8))
			frame = cv2.imread(dosya)
		return frame

	def cizgiciz(self,frame,c,cx,cy):
		s = ""
		if cx >= self.cap_hx:
			s = "Sag-"
		elif cx < self.cap_hx:
			s = "Sol-"
		if cy <= self.cap_hy:
			s += "Ust"
		elif cy > self.cap_hy:
			s += "Alt"
		cv2.putText(frame, s,(self.cap_hx, self.cap_hy),
						cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
		cv2.line(frame, (int(cx), int(cy)), (self.cap_hx, self.cap_hy), (0, 255, 0), 2)
		cv2.drawContours(frame, [c], 0, (0, 255 ,self.hedef_kilit*255), 2)
		return frame

	def sonlandir(self):
		self.cap.release()

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

if __name__=="__main__":
	# Profiler ayarla (Kod performasını incelemek için)
	#profiler = cProfile.Profile()
	#profiler.enable()

	# Arac ile baglanma modlari
	#baglanti_modu = "USB" #Arac komutlarini ve gorev algoritmasini denemek icin
	#baglanti_modu = "SITL"
	baglanti_modu = "BAGLI_DEGIL" #Sadece gorev algoritmasini denemek icin

	#Araca baglan
	if baglanti_modu == "USB" or baglanti_modu == "SITL":
		arac = DolunaySuAlti.Dolunay(baglanti_modu)
		arac.set_mod("acro")
		arac.set_arm(1)

	# Gorev1 olustur
	gorev = Gorev1(baglanti_modu)
	try:
		if baglanti_modu == "USB" or baglanti_modu == "SITL":
			while True:
				dx, dy, dz, dr = gorev.calistir()
				arac.hareket_et_v2(dx, dy, dz, dr)
		elif baglanti_modu == "BAGLI_DEGIL":
			while True:
				gorev.calistir()
				time.sleep(0.1)
	except:
		print("Program kapatiliyor.")
		# Kamerayi kapat
		if baglanti_modu == "USB":
			gorev.cap_yoket()
		# Arac baglantısını kapat
		if baglanti_modu == "USB" or baglanti_modu == "SITL":
			# Araci disarm et
			arac.set_arm(0)
			#Baglantiyi kapat
			arac.kapat()
		# Profiler bitir ve sonuclari yazdir
		#profiler.disable()
		#profiler.print_stats("tottime")