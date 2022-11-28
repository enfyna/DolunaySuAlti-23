# encoding: utf-8

## Module infomation ###
# Python (3.4.4)
# numpy (1.10.2)
# PyAudio (0.2.9)
# matplotlib (1.5.1)
# All 32bit edition
########################
import chunk
from pickle import GLOBAL
from typing_extensions import Self
import wave
import numpy as np
import pyaudio
import matplotlib.pyplot as plt
import sys
import csv
import Filter

filterState = 0

x = 0
file = open("veriler.txt","w")

class SpectrumAnalyzer:
    FORMAT = pyaudio.paFloat32
    CHANNELS = 1
    RATE = 16000
    CHUNK = 2048
    START = 0
    N = 2048
 
    wave_x = 0
    wave_y = 0
    spec_x = 0
    spec_y = 0
    
    R = 1000            # R Ohm
    C = 150e-9          # C Farad
    Ts = 1.0 / RATE     # sampling Frequency

    data = [0] * CHUNK
    filteredata = []
    a = 0

    myFilter = Filter.LowPassFilter(data, R, C, Ts)

    fig = plt.figure()
    
    def on_close(event):
        global x
        print(x)
        x = 1
        print(x)
        file.close()
        print('Dosyaya kaydedildi...')

    fig.canvas.mpl_connect('close_event', on_close)

    def __init__(self):
        self.pa = pyaudio.PyAudio()
 
        self.stream = self.pa.open(format = self.FORMAT,
            channels = self.CHANNELS,
            rate = self.RATE,
            input = True,
            output = False,
            frames_per_buffer = self.CHUNK)
        
        # Main loop
        global x
        while x == 0:
            self.loop()
        if x == 1:
            self.pa.close(self.stream)

    def loop(self):
        self.data = self.audioinput()
        if filterState == 1:
            self.filteredata = self.myFilter.FilterApply(self.data)
            
            self.data = self.filteredata
        self.fft()
        self.dosyayaz()
        self.graphplot()
    
    def dosyayaz(self):
        #print(self.spec_x)
        #print("***")
        max_deger = 0
        for i in range(len(self.spec_x)):#Alttaki grafik dosyaya yazdır
            if self.spec_x[i] > max_deger:
                max_deger = self.spec_x[i]
        self.a += 1
        file.write(str(max_deger)+" \n")#Alttaki grafik dosyaya yazdır

    def audioinput(self):
        ret = self.stream.read(self.CHUNK)
        ret = np.frombuffer(ret, np.float32)
        return ret
 
    def fft(self):
        self.wave_x = range(self.START, self.START + self.N)
        self.wave_y = self.data[self.START:self.START + self.N]
        self.spec_x = np.fft.fftfreq(self.N, d = 1.0 / self.RATE)
        y = np.fft.fft(self.data[self.START:self.START + self.N])
        self.spec_y = [np.sqrt(c.real ** 2 + c.imag ** 2) for c in y]

        signal = np.array(self.data[self.START:self.START + self.N])
        fft_spectrum = np.fft.rfft(signal)
        freq = np.fft.rfftfreq(signal.size, d=1./self.RATE)
        fft_spectrum_abs = np.abs(fft_spectrum)
        for i,f in enumerate(fft_spectrum_abs):
            if f > 10: #looking at amplitudes of the spikes higher than 350 (Şiddet Flitresi) 
                if (np.round(freq[i],1))>10 and (np.round(freq[i],1))<250: #Frekans Flitresi
                    print('frequency = {} Hz with amplitude {} '.format(np.round(freq[i],1),  np.round(f)))
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
        plt.axis([0, 1000, 0, 50])
        plt.xlabel("frequency [Hz]")
        plt.ylabel("amplitude spectrum")
        
        #Pause
        plt.pause(.05)
        
 
if __name__ == "__main__":
    spec = SpectrumAnalyzer()

