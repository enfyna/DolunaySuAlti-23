import matplotlib.pyplot as plt
import numpy as np
import pyaudio

class Gorev():
    def __init__(self,baglanti_modu,var):
        self.FORMAT = pyaudio.paFloat32
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 4096
        self.START = 0
        self.N = 4096

        self.wave_x = range(self.START, self.START + self.N)
        self.wave_y = 0
        self.spec_x = np.fft.fftfreq(self.N, d = 1.0 / self.RATE)        
        self.spec_y = 0
        
        self.R = 1000            # R Ohm
        self.C = 150e-9          # C Farad
        self.Ts = 1.0 / self.RATE     # sampling Frequency

        self.data = [0] * self.CHUNK

        self.y  = [0] * self.CHUNK
        self.K1 = (self.Ts / (self.Ts + 2 * self.R * self.C))
        self.K2 = (self.Ts / (self.Ts + 2 * self.R * self.C))
        self.K3 = ((self.Ts - 2 * self.R * self.C) / (self.Ts + 2 * self.R * self.C))

        self.pa = pyaudio.PyAudio()
        self.get_input_devices()
        
        self.stream = self.pa.open(
            format = self.FORMAT,
            channels = self.CHANNELS,
            rate = self.RATE,
            input = True,
            output = False,
            frames_per_buffer = self.CHUNK)#,
            #input_device_index=22)
        self.file = open("veriler.txt","w")
        pass

    def calistir(self):
        self.fft(self.get_LPF(self.get_audio()))
        self.graphplot()

    def get_audio(self):
        ret = self.stream.read(self.CHUNK,False)
        return np.frombuffer(ret, np.float32)
 
    def fft(self,data):
        self.wave_y = data[self.START:self.START + self.N]
        
        y = np.fft.fft(self.wave_y)
        self.spec_y = [(c.real ** 2 + c.imag ** 2) for c in y]

        fft_spectrum = np.fft.rfft(np.array(self.wave_y))
        freq = np.fft.rfftfreq(self.N, d=1./self.RATE)
        fft_spectrum_abs = np.abs(fft_spectrum)
        for i,f in enumerate(fft_spectrum_abs):
            if f > 10: #looking at amplitudes of the spikes higher than 350 (Şiddet Filtresi) 
                if (np.round(freq[i],1))>10: #Frekans Filtresi
                    stat = str('frequency = {} Hz with amplitude {} '.format(np.round(freq[i],1),  np.round(f)))
                    print(stat)
                    self.dosyayaz(stat)
    
    def graphplot(self):    
        plt.clf()
        #Spectrum
        plt.subplot(111)
        plt.plot(self.spec_x, self.spec_y, marker= '', linestyle='-')
        plt.axis([0, 10000, 0, 100])
        plt.xlabel("Frekans [Hz]")
        plt.ylabel("Genlik")
        #Pause
        plt.pause(0.1)
    
    def get_LPF(self, x):
        """
        Low Pass Filtre
        """
        for i in range(self.CHUNK):
            self.y[i] = (x[i] * self.K1) + (x[i - 1] * self.K1) - (self.y[i - 1] * self.K3)
        return (self.y)
    
    def dosyayaz(self,str_data):
        self.file.write(str_data+" \n")#Alttaki grafik dosyaya yazdır

    def get_input_devices(self):
        """
        Bilgisayara takili mikrofonlari yazdir
        """
        numdevices = self.pa.get_host_api_info_by_index(0).get('deviceCount')
        for i in range(0, numdevices):
            if (self.pa.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                print("Input Device id ", i, " - ", self.pa.get_device_info_by_host_api_device_index(0, i).get('name'))

    """
    def GetFrequency(self):
        return (1 / (2 * pi * self.R * self.C))

    def GetWarping(self):
        return (2 / self.T) * arctan(2 * pi * self.GetFrequency() * self.T / 2) / (2 * pi)
     
    """
