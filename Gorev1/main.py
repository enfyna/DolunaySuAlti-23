import sys , cv2 , time #, cProfile
import Gorev1
#import Gorev2
#import Gorev3
import DolunaySuAlti

def yunusbaligi():
    arac.hareket_et(400, 0, 0, 0)
    pass

# Profiler ayarla (Kod performasını incelemek için)
#profiler = cProfile.Profile()
#profiler.enable()

# Calistirilacak olan gorevi sec
try:    
    secilengorev = sys.argv[1]
except:
    #secilengorev = int(input("gorev sec : "))
    secilengorev = 1
print("Secili gorev : " + str(secilengorev))

# Arac ile baglan
#baglanti_modu = "USB"
baglanti_modu = "SITL"

arac = DolunaySuAlti.Dolunay(baglanti_modu)
arac.set_mod("Manual")
arac.set_arm(1)

try:
    if baglanti_modu == "USB":
        if secilengorev == 1:
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)
            cap_hx = int(cap.get(3)/2)
            gorev = Gorev1.Gorev(baglanti_modu,cap)
        if secilengorev == 2:
            pass
        if secilengorev == 3:
            pass
        while True:
            sonuc = gorev.calistir("USB")
            
            if sonuc[0] == None and sonuc[1] == None:
                yunusbaligi()
            else:
                x = cap_hx - sonuc[1]
                y = sonuc[0]
                z = sonuc[1]
                r = sonuc[0]
                arac.hareket_et(x,y,z,r)
            time.sleep(0.01)

    elif baglanti_modu == "SITL":
        if secilengorev == 1: #Gorevlere gereken degerleri ilet
            gorev = Gorev1.Gorev(baglanti_modu,None)
            render_kare_sayisi = [28,350]
        if secilengorev == 2:
            render_kare_sayisi = [None,None]
            pass
        if secilengorev == 3:
            render_kare_sayisi = [None,None]
            pass
        i = render_kare_sayisi[0] + 1
        while True:
            i += 1
            if i < render_kare_sayisi[0] or i > render_kare_sayisi[1]:
                i = render_kare_sayisi[0] + 1
            
            sonuc = gorev.calistir(i)
            
            if sonuc[0] == None:
                yunusbaligi()
            else:
                x = 250-sonuc[1]
                y = sonuc[0]
                z = sonuc[1]
                r = sonuc[0]*2
                arac.hareket_et(x,y,z,r)
            time.sleep(0.1)
except KeyboardInterrupt:
    print("Gorev sonlandirildi.")

# Araci disarm et
arac.set_arm(0)
# Kamerayi kapat
if secilengorev < 3 and baglanti_modu == "USB":
    gorev.cap_yoket()
# Arac baglantısını kapat
if baglanti_modu == "USB":
    arac.kapat()
# Profiler bitir ve sonuclari yazdir
#profiler.disable()
#profiler.print_stats("tottime")

