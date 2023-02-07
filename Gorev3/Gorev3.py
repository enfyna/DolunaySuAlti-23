import numpy as np
import pyaudio , time
import matplotlib.pyplot as plt
import cProfile

class Gorev():
    def __init__(self):
        sag_mikrofon = Mikrofon("1",None)
        sol_mikrofon = Mikrofon("2",None)

    def calistir(self):
        r1, r2 = self.sag_mikrofon.calistir() , self.sol_mikrofon.calistir() 
        """
        Sag ve sol mikrofondan gelen ses siddetlerini birbirinden cikar.
        Sag mikrofon buyuk ise sonuc pozitif , sol buyuk ise negatif cikacak.
        Mavlink hareket komutundada araci ayni sekilde pozitif deger ise saga negatif deger ise sola yonlendiriyor.
        k katsayisi hareketin siddetini ayarlamak icin testlerde deneyip iyi bir deger bulmak lazim

        Bu kod suanda aracin dogru yada ters gittigini hesaplamiyor onu eklemek lazim.
        Mesela sag ve sol birbirine esit oldugu durumda aracin yonu pingere bakiyor ama pingerin tersine gitme durumu kontrol edilmiyor.
        Duzeltmek icin r1 ve r2 nin onceki degerleriyle karsilastirilip eger 2si azaliyorsa aracin ters cevirilmesi lazim. 
        """
        k = 1
        r = (r1 - r2) * k
        return r  

class Mikrofon():
    def __init__(self,var1,var2):# var1 ve var2 ilerde mikrofon secerken filan lazım olabilir suanda kullanilmiyor
        self.FORMAT = pyaudio.paFloat32 #Mikrofondan gelen sesin tipi
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 4096 #Mikrofondan alinacak anlik ses parçasinin uzunlugu
        self.START = 0
        self.N = 4096

        self.wave_x = range(self.START, self.START + self.N)
        self.wave_y = 0
        self.spec_x = np.fft.rfftfreq(self.N, d = 1.0 / self.RATE)
        self.spec_y = 0

        self.R = 1000               # R Ohm
        self.C = 150e-9             # C Farad
        self.Ts = 1.0 / self.RATE   # sampling Frequency

        self.data = [0] * self.CHUNK

        self.y  = [0] * self.CHUNK
        self.K1 = (self.Ts / (self.Ts + 2 * self.R * self.C))
        self.K2 = (self.Ts / (self.Ts + 2 * self.R * self.C))
        self.K3 = ((self.Ts - 2 * self.R * self.C) / (self.Ts + 2 * self.R * self.C))

        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format = self.FORMAT,
            channels = self.CHANNELS,
            rate = self.RATE,
            input = True,
            output = False,
            frames_per_buffer = self.CHUNK)#,
            #input_device_index=22) #Hidrofonlari secerken lazim olacak
        pass

    def calistir(self):
        spec_y , y = self.fft_v3(self.CHUNK,self.START,self.N,self.RATE,self.data)
        self.graphplot(spec_y)
        return y

    def fft_v3(self,CHUNK,START,N,RATE,data):
        """
        Gelen ses dalgasina FFT uygula.
        """
        #Mikrofondan ses al.
        x = self.stream.read(CHUNK,False)
        x = np.frombuffer(x, np.float32)
        #LPF uygula.
        for i in range(CHUNK):
            data[i] = (x[i] * self.K1) + (x[i - 1] * self.K2) - (self.y[i - 1] * self.K3)
        wave_y = data[START:START + N]
        #FFT uygula.
        y = np.fft.rfft(wave_y)
        spec_y = [(c.real ** 2 + c.imag ** 2) for c in y]

        #İstenilen frekans araligindaki maks genligi bul
        max_spec_y = max(spec_y[500:1000]) # -> 2000 ve 4000 frekans araligindaki maks genligi buluyor    
        print(max_spec_y)

        return spec_y , max_spec_y

    def graphplot(self,spec_y):
        """
	       Grafik ciz.
	      """
        plt.clf()
        plt.subplot(111)
        plt.plot(self.spec_x, spec_y, marker= '', linestyle='-')
        plt.axis([0, 8000, 0, 100])
        plt.xlabel("Frekans [Hz]")
        plt.ylabel("Genlik")
        plt.pause(0.1)

    def get_input_devices(self):
        """
        Bilgisayara takili mikrofonlari yazdir
        """
        numdevices = self.pa.get_host_api_info_by_index(0).get('deviceCount')
        for i in range(0, numdevices):
            if (self.pa.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                print("Input Device id ", i, " - ", self.pa.get_device_info_by_host_api_device_index(0, i).get('name'))

    
profiler = cProfile.Profile()
profiler.enable()

#g = Gorev() # Aracta 2 mikrofon olunca boyle calistirilacak
g = Mikrofon(None, None) # Tek mikrofon oldugu icin simdilik boyle

try:
    while True:
        g.calistir()
        time.sleep(0.1)
except KeyboardInterrupt:
    profiler.disable()
    profiler.print_stats("tottime")
    pass
