import sys, time , cProfile
import Gorev1
#import Gorev2
import Gorev3
import DolunaySuAlti

def calistir(arac,baglanti_modu,secilengorev,gorev=None):
    try:
        if baglanti_modu == "USB":
            if secilengorev == 1:
                cap = cv2.VideoCapture(0)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)
                gorev = Gorev1.Gorev(baglanti_modu,None)
                gorev = gorev.modu
            if secilengorev == 2:
                pass
            if secilengorev == 3:
                gorev = Gorev3.Gorev(baglanti_modu,None)
                pass
            while True:
                """
                #Ornek kod : 
                sonuc = gorev.calistir()
                if sonuc[0] == None and sonuc[1] == None:
                    yunusbaligi()
                else:
                    x = sonuc[0]
                    y = sonuc[1]
                    z = sonuc[2]
                    r = sonuc[3]
                    arac.hareket_et(x,y,z,r)
                time.sleep(0.1)
                """
        elif baglanti_modu == "SITL":
            if secilengorev == 1: #Gorevlere gereken degerleri ilet
                gorev = Gorev1.Gorev(baglanti_modu,None)
                gorev = gorev.modu
                render_kare_sayisi = [28,350]
            if secilengorev == 2:
                render_kare_sayisi = [0,10]
                pass
            if secilengorev == 3:
                gorev = Gorev3.Gorev(baglanti_modu,None)
                render_kare_sayisi = [None,None]
                pass
            i = render_kare_sayisi[0] + 1

            forward = int(input("Forward : "))
            lateral = int(input("Lateral : "))
            throttle = int(input("Throttle : "))
            yaw = int(input("Yaw : "))
            roll_deg = int(input("Roll_Deg : "))
            pitch_deg = int(input("Pitch_Deg : "))
            
            while True:
                i += 1
                if i < render_kare_sayisi[0] or i > render_kare_sayisi[1]:
                    i = render_kare_sayisi[0] + 1
                
                arac.hareket_et_v2(set_forward=forward, set_lateral=lateral, set_throttle=throttle, set_yaw=yaw, set_roll_deg=roll_deg, set_pitch_deg=pitch_deg)
                time.sleep(1)
        
        elif baglanti_modu == "BAGLI_DEGIL":
            if secilengorev == 1: #Gorevlere gereken degerleri ilet
                gorev = Gorev1.Gorev(baglanti_modu,None)
                gorev = gorev.modu
                render_kare_sayisi = [28,350]
            if secilengorev == 2:
                render_kare_sayisi = [0,10]
                pass
            if secilengorev == 3:
                gorev = Gorev3.Gorev(baglanti_modu,None)
                render_kare_sayisi = [0,10]
                pass
            i = render_kare_sayisi[0] + 1
            while True:
                i += 1
                if i < render_kare_sayisi[0] :
                    i = render_kare_sayisi[0] + 1
                elif i > render_kare_sayisi[1]:
                    i = render_kare_sayisi[0] + 1
            pass
    except KeyboardInterrupt as e:
        s = str(input("Gorev sonlandirilsin mi ? [e/h]"))
        if s.lower() == "h":
            calistir(arac, baglanti_modu, secilengorev, gorev)
        

# Profiler ayarla (Kod performasını incelemek için)
profiler = cProfile.Profile()
profiler.enable()

# Calistirilacak olan gorevi sec
try:    
    secilengorev = sys.argv[1]
except:
    #secilengorev = int(input("gorev sec : "))
    secilengorev = 1
print("Secili gorev : " + str(secilengorev))

# Arac ile baglanma modlari 
baglanti_modu = "USB" #Arac komutlarini ve gorev algoritmasini denemek icin
baglanti_modu = "SITL"
#baglanti_modu = "BAGLI_DEGIL" #Sadece gorev algoritmasini denemek icin

if baglanti_modu == "USB" or baglanti_modu == "SITL":
    arac = DolunaySuAlti.Dolunay(baglanti_modu)
    arac.set_mod("acro")
    arac.set_arm(1)

calistir(arac,baglanti_modu,secilengorev)

# Kamerayi kapat
if secilengorev < 3 and baglanti_modu == "USB":
    gorev.cap_yoket()
# Arac baglantısını kapat
if baglanti_modu == "USB" or baglanti_modu == "SITL":
    # Araci disarm et
    arac.set_arm(0)
    #Baglantiyi kapat
    arac.kapat()
# Profiler bitir ve sonuclari yazdir
profiler.disable()
#profiler.print_stats("tottime")
