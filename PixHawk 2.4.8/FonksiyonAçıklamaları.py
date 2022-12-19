from pymavlink import mavutil as m
import time

#link adında pixhawk ile baglanti olustur
link = m.mavlink_connection('/dev/ttyACM0',baud=57600)

# 1. parametre olarak pixhawkin bulundugu port yazılması gerekli
#Serial,uzerinden baglantı icin '/dev/ttyTHS1'
#USB,uzerinden baglantı icin '/dev/ttyUSB' ya da '/dev/ttyACM'
#UDP,IP(internet) uzerinden baglantı icin 'udp:localhost:14540'

#!!!Not: ttyUSB yada ttyACM sonundaki sayı her baglantı kapatılıp acıldıgında 1 artar
#Yani bilgisayar acildiginda 0 ile basliyorsa baglantı kopup tekrar baglanılırsa 1 olur

# 2. parametre baudrate 
#pixhawk ile iletisime gecilecek baudrate hızı
#yazılması zorunlu degil 

#Baglandıgımız pixhawktan kalp atisi bilgisi al
link.wait_heartbeat()
#Mavlink ile baglanan cihazlar kalp atisi sinyali ile birbirlerine bagli oldugunu gosterir

#Pixhawka arm etme komutu ver
link.mav.command_long_send(link.target_system,link.target_component, #her emir icin sabit
m.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # emir kodu int olarakda verilebilir boylede yazılabilir
0, #confirmation eger uzun sure cmd_ack alınamazsa 1 yapılıp tekrar gonderilir
1,0,0,0,0,0,0) #emir parametreleri
#İlk iki parametre sabit target system ve component bilgisi
#3. parametre verilecek emrin kodu 
#4. parametre confirmation 0 yazılır ilk once
#Geriye kalan parametreler emrin kendi parametreleri her emir için farklı anlamları vardır
# https://mavlink.io/en/messages/common.html#mav_commands buradan bakılabilir

time.sleep(1)

i = 0
dosya = open("mesaj_id.txt",mode="w") #mesaj idlerini kaydetmek icin yaptım onemli degil 

while True:
    try:
        msg1 = link.messages

        print(msg1)
        # print(msg1['MAV'])
        # print(msg1['HOME'])

        link.mav.command_long_send(link.target_system,link.target_component,
        m.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        i,0,0,0,0,0,0) 
        
        msg2 = link.recv_match(type='COMMAND_ACK',blocking=True).to_dict()
        if msg2['result'] == 0 or msg2['result'] == 4:
            msg2 = link.recv_match(blocking=True).to_dict()
            if msg2['mavpackettype'] != 'HEARTBEAT' and msg2['mavpackettype'] != 'TIMESYNC'\
                and msg2['mavpackettype'] != 'STATUSTEXT': # bunlara gerek yok
                s1 = 25 - len(msg2['mavpackettype'])
                s = msg2['mavpackettype']+"_"*s1+str(i)+"\n"
                print(s,end="\n")
                dosya.write(s)
        i+=1 # tum mesaj idlerini teker teker dene
        # recv_match fonksiyonu pixhawk durumu hakkında bilgileri dict olarak verir
        # Type parametresi
        # istediginiz bilgiyi verir eger bos bırakılırsa tum bilgileri verir
        
        if msg2:
            print(msg2,end="--\n")
        time.sleep(1)
        pass
    except Exception as e:
        print(type(e))
        time.sleep(5)
        pass
    except KeyboardInterrupt:
        print("bruh")
        time.sleep(0.5)
        breaK
#Pixhawk disarm et
link.mav.command_long_send(link.target_system,link.target_component, 
m.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
0,
0,0,0,0,0,0,0)







