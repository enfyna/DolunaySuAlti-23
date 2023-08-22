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
