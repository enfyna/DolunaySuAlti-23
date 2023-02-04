import numpy as np
import pyaudio
import matplotlib.pyplot as plt
import sys, csv

class Gorev:
    def __init__(self,baglanti_modu,var):
        self.FORMAT = pyaudio.paFloat32
        self.CHANNELS = 1
        self.RATE = 8000
        self.CHUNK = 2048
        self.START = 0
        self.N = 2048

        self.wave_x = range(self.START, self.START + self.N)
        self.wave_y = 0
        
        self.spec_x = np.fft.fftfreq(self.N, d = 1.0 / self.RATE)        
        self.spec_y = 0
        
        self.R = 1000            # R Ohm
        self.C = 150e-9          # C Farad
        self.Ts = 1.0 / self.RATE     # sampling Frequency

        self.data = [0] * self.CHUNK
        self.filteredata = []

        self.myFilter = LowPassFilter(self.data, self.R, self.C, self.Ts)
        self.pa = pyaudio.PyAudio()
        info = self.pa.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        for i in range(0, numdevices):
            if (self.pa.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                print("Input Device id ", i, " - ", self.pa.get_device_info_by_host_api_device_index(0, i).get('name'))
        self.stream = self.pa.open(format = self.FORMAT,
            channels = self.CHANNELS,
            rate = self.RATE,
            input = True,
            output = False,
            frames_per_buffer = self.CHUNK,
            input_device_index=22)
        self.file = open("veriler.txt","w")
        pass

    def calistir(self):
        self.data = self.myFilter.FilterApply(self.audioinput())
        self.fft()
        self.graphplot()
        
    def audioinput(self):
        ret = self.stream.read(self.CHUNK)
        ret = np.frombuffer(ret, np.float32)
        return ret
 
    def fft(self):
        self.wave_y = self.data[self.START:self.START + self.N]
        
        y = np.fft.fft(self.wave_y)
        self.spec_y = [np.sqrt(c.real ** 2 + c.imag ** 2) for c in y]

        signal = np.array(self.wave_y)
        fft_spectrum = np.fft.rfft(signal)
        freq = np.fft.rfftfreq(signal.size, d=1./self.RATE)
        fft_spectrum_abs = np.abs(fft_spectrum)
        for i,f in enumerate(fft_spectrum_abs):
            if f > 10: #looking at amplitudes of the spikes higher than 350 (Şiddet Filtresi) 
                if (np.round(freq[i],1))>10: #Frekans Filtresi
                    stat = str('frequency = {} Hz with amplitude {} '.format(np.round(freq[i],1),  np.round(f)))
                    print(stat)
                    self.dosyayaz(stat)
    
    def graphplot(self):    
        plt.clf()
        # wave
        plt.subplot(311)
        
        plt.plot(self.wave_x, self.wave_y)
        
        plt.axis([self.START, self.START + self.N, -1, 1])
        plt.xlabel("time [sample]")
        plt.ylabel("amplitude")
        #Spectrum
        plt.subplot(312)
        plt.plot(self.spec_x, self.spec_y, marker= '', linestyle='-')
        plt.axis([0, 10000, 0, 100])
        plt.xlabel("frequency [Hz]")
        plt.ylabel("amplitude spectrum")
        
        #Pause
        plt.pause(.05)
        
    def dosyayaz(self,str_data):
        self.file.write(str_data+" \n")#Alttaki grafik dosyaya yazdır

        
class LowPassFilter(object):
    def __init__(self, x, R, C, period):
        self.__R = R
        self.__C = C
        self.__T = period
        self.__y = [0] * len(x)

        self.__K1 = (self.__T / (self.__T + 2 * self.__R * self.__C))
        self.__K2 = (self.__T / (self.__T + 2 * self.__R * self.__C))
        self.__K3 = ((self.__T - 2 * self.__R * self.__C) / (self.__T + 2 * self.__R * self.__C))

    def FilterApply(self, __x):
        for i in range(len(__x)):
            self.__y[i] = (__x[i] * self.__K1) + (__x[i - 1] * self.__K2) - (self.__y[i - 1] * self.__K3)
        return (self.__y)

    def GetFrequency(self):
        return (1 / (2 * pi * self.__R * self.__C))

    def GetWarping(self):
        return (2 / self.__T) * arctan(2 * pi * self.GetFrequency() * self.__T / 2) / (2 * pi)
